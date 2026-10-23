"""CARLA runtime scene for GB/T 41798-2022 2.a tunnel screening."""

from __future__ import annotations

import hashlib
import math
import threading
import time

import carla

from stf_scene import EgoRouteFollowScene
from tunnel_geometry import TunnelRouteGeometry
from stf_vehicle_control import decode_vehicle_action
from tunnel_2a import (
    gate_crossing,
    project_to_route,
    validate_tunnel_config,
)


CORRIDOR_CENTER_TOLERANCE_M = 0.35


def _map_basename(value):
    return str(value or "").rstrip("/").rsplit("/", 1)[-1]


def _location_payload(location):
    return {
        "x": float(location.x),
        "y": float(location.y),
        "z": float(getattr(location, "z", 0.0)),
    }


class TunnelScene(TunnelRouteGeometry, EgoRouteFollowScene):
    """Pass-through-only tunnel scene using the existing route controllers."""

    def __init__(self, client, world, config_path, town, route_id,
                 model, model_path=None):
        super().__init__(
            client, world, config_path, town, route_id, model, model_path)
        validate_tunnel_config(self.config)
        self.tunnel_cfg = self.config["tunnel_test"]
        self.tunnel_timeout_s = float(
            self.tunnel_cfg.get("scenario_timeout_s", 60.0))
        # Parent route control remains in use, but its legacy wall-clock timeout
        # must never decide a 2.a result.  This class uses CARLA simulation time.
        self.timeout = 10 ** 9

        self.tunnel_dense_route = []
        self.tunnel_corridor = []
        self.tunnel_entry_gate = None
        self.tunnel_exit_gate = None
        self.measured_tunnel_length_m = None
        self.tunnel_phase = "SETUP"
        self.tunnel_start_sim_time = None
        self.tunnel_entry_sim_time = None
        self.tunnel_exit_sim_time = None
        self.tunnel_previous_location = None
        self.tunnel_driven_distance_m = 0.0
        self.max_tunnel_lateral_error_m = 0.0
        self.same_lane = True
        self.timed_out = False
        self.route_complete = False
        self._collision_recorded = False
        self._wrong_gate_order = False
        self._lane_invasion_sensor = None
        self._lane_invasion_events = []
        self._telemetry = []
        self.tcp_frame_timeout_s = 2.0
        self._tcp_frames = {}
        self._tcp_frame_condition = threading.Condition()
        self._tcp_last_frame = None
        self._tcp_control_samples = []
        self._tcp_error = None

    def _on_tcp_image(self, image):
        import numpy as np
        rgb = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
            (image.height, image.width, 4))[:, :, :3][:, :, ::-1].copy()
        with self._tcp_frame_condition:
            self._tcp_frames[int(image.frame)] = (float(image.timestamp), rgb)
            while len(self._tcp_frames) > 16:
                self._tcp_frames.pop(min(self._tcp_frames))
            self._tcp_frame_condition.notify_all()

    def spawn_camera(self):
        """Keep the TCP sensor contract; buffer images by their CARLA frame ID."""
        from model.stf_tcp import (TCP_CAMERA_WIDTH, TCP_CAMERA_HEIGHT, TCP_CAMERA_FOV,
                               TCP_CAMERA_X, TCP_CAMERA_Y, TCP_CAMERA_Z)
        blueprint = self.world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(TCP_CAMERA_WIDTH))
        blueprint.set_attribute("image_size_y", str(TCP_CAMERA_HEIGHT))
        blueprint.set_attribute("fov", str(TCP_CAMERA_FOV))
        blueprint.set_attribute("sensor_tick", "0.0")
        self.camera_sensor = self.world.spawn_actor(
            blueprint, carla.Transform(carla.Location(x=TCP_CAMERA_X, y=TCP_CAMERA_Y, z=TCP_CAMERA_Z)),
            attach_to=self.ego)
        self.actors.append(self.camera_sensor)
        self.camera_sensor.listen(self._on_tcp_image)

    def _wait_tcp_image(self, frame, timestamp, timeout=None):
        deadline = time.monotonic() + (self.tcp_frame_timeout_s if timeout is None else timeout)
        with self._tcp_frame_condition:
            while frame not in self._tcp_frames:
                for old in [key for key in self._tcp_frames if key < frame]:
                    self._tcp_frames.pop(old)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("camera_frame_timeout: expected={}".format(frame))
                self._tcp_frame_condition.wait(remaining)
            image_time, image = self._tcp_frames.pop(frame)
        if not math.isfinite(image_time) or abs(image_time - timestamp) > 1e-3:
            raise RuntimeError("camera_timestamp_mismatch: frame={}".format(frame))
        return image

    def _hold_tcp(self):
        self.ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))

    def _fail_tcp(self, error):
        self._tcp_error = str(error)
        self.tunnel_phase = "CONTROLLER_ERROR"
        self._hold_tcp()
        return False

    def _update_tcp_completion(self):
        # Completion is independent of which controller produced the motion.
        # Require the entry/exit gates and the final post-exit point, rather
        # than relying on the reference controller's private target cursor.
        location = self.ego.get_location()
        while self.current_target_idx < len(self.route_points):
            if location.distance(self.route_points[self.current_target_idx]) > self.stop_distance:
                break
            self.current_target_idx += 1
        if (self.tunnel_exit_sim_time is not None and self.route_points
                and location.distance(self.route_points[-1]) <= self.stop_distance):
            self.current_target_idx = len(self.route_points)
            self.finished = self.route_complete = True
            self.tunnel_phase = "COMPLETE"
            self._hold_tcp()
            return True
        return False

    def _tick_tcp(self, snapshot):
        if self._tcp_error is not None:
            return False
        if self._update_tcp_completion():
            return False
        frame = int(snapshot.frame)
        timestamp = float(snapshot.timestamp.elapsed_seconds)
        try:
            if not self.tcp_flag or self.tcp is None:
                raise RuntimeError("tcp_controller_unavailable")
            if self._tcp_last_frame is not None and frame <= self._tcp_last_frame:
                raise RuntimeError("duplicate_or_reversed_control_frame:{}".format(frame))
            image = self._wait_tcp_image(frame, timestamp)
            if int(self.world.get_snapshot().frame) != frame:
                raise RuntimeError("world_advanced_while_waiting_for_camera")
            import torch
            with torch.inference_mode():
                action = self.tcp.get_action(image, self.ego)
            throttle, steer, brake = decode_vehicle_action(action)
            if int(self.world.get_snapshot().frame) != frame:
                raise RuntimeError("world_advanced_during_tcp_inference")
            self.control.throttle, self.control.steer, self.control.brake = throttle, steer, brake
            self.control.hand_brake = False
            self.ego.apply_control(self.control)
            self._tcp_last_frame = frame
            self._tcp_control_samples.append(dict(
                world_frame=frame, camera_frame=frame, sim_time=timestamp,
                throttle=throttle, steer=steer, brake=brake))
            return True
        except Exception as exc:
            return self._fail_tcp(exc)

    def _validate_map_identity(self):
        actual_name = _map_basename(self.map.name)
        requested_name = _map_basename(self.town)
        configured_name = _map_basename(self.tunnel_cfg.get("map_name"))
        if not actual_name or actual_name.lower() != requested_name.lower():
            raise RuntimeError(
                "2.a map mismatch: CARLA={!r}, --town={!r}".format(
                    actual_name, self.town))
        if configured_name and actual_name.lower() != configured_name.lower():
            raise RuntimeError(
                "2.a map mismatch: CARLA={!r}, config={!r}".format(
                    actual_name, configured_name))
        opendrive = self.map.to_opendrive()
        digest = hashlib.sha256(opendrive.encode("utf-8")).hexdigest()
        configured_digest = self.tunnel_cfg.get("opendrive_sha256")
        if configured_digest and digest != configured_digest:
            raise RuntimeError(
                "2.a OpenDRIVE hash differs from the map used by the editor")
        self.tunnel_map_name = actual_name
        self.tunnel_opendrive_sha256 = digest

    @staticmethod
    def _event_time(event, world):
        value = getattr(event, "timestamp", None)
        if hasattr(value, "elapsed_seconds"):
            value = value.elapsed_seconds
        if value is None:
            value = world.get_snapshot().timestamp.elapsed_seconds
        return float(value)

    def _spawn_lane_invasion_sensor(self):
        blueprint = self.world.get_blueprint_library().find(
            "sensor.other.lane_invasion")
        sensor = self.world.spawn_actor(
            blueprint, carla.Transform(), attach_to=self.ego)

        def on_invasion(event):
            markings = []
            for marking in getattr(event, "crossed_lane_markings", []):
                markings.append({
                    "type": str(getattr(marking, "type", "Unknown")),
                    "color": str(getattr(marking, "color", "Unknown")),
                    "lane_change": str(getattr(marking, "lane_change", "Unknown")),
                })
            self._lane_invasion_events.append({
                "frame": getattr(event, "frame", None),
                "sim_time": self._event_time(event, self.world),
                "markings": markings,
            })

        sensor.listen(on_invasion)
        self._lane_invasion_sensor = sensor
        self.actors.append(sensor)

    def spawn(self):
        self._validate_map_identity()
        self._prepare_tunnel_geometry()
        if self.model == "tcp" and not self.world.get_settings().synchronous_mode:
            raise RuntimeError("2.a TCP requires synchronous CARLA stepping")
        super().spawn()
        self._spawn_lane_invasion_sensor()
        if self.model == "tcp":
            if not self.tcp_flag or self.tcp is None:
                raise RuntimeError("TCP model/checkpoint is required; reference fallback is disabled")
            self._hold_tcp()
            self.tcp.load_model(strict=True)  # Reject partial/random weights before the trial starts.
            # Camera startup/settling is outside the timed trial, with Ego held.
            for _ in range(20):
                self._hold_tcp()
                self.world.tick()
                snapshot = self.world.get_snapshot()
                try:
                    image = self._wait_tcp_image(int(snapshot.frame),
                                                snapshot.timestamp.elapsed_seconds, timeout=0.5)
                    with self._tcp_frame_condition:
                        self._tcp_frames[int(snapshot.frame)] = (snapshot.timestamp.elapsed_seconds, image)
                    break
                except RuntimeError:
                    continue
            else:
                raise RuntimeError("TCP camera startup failed: no matching frame")
        now = float(self.world.get_snapshot().timestamp.elapsed_seconds)
        self.tunnel_start_sim_time = now
        self.tunnel_previous_location = _location_payload(self.ego.get_location())
        self.tunnel_phase = "APPROACH"

    def record_collision(self, event=None, now=None):
        self._collision_recorded = True

    def _relative_time(self, sim_time):
        if sim_time is None or self.tunnel_start_sim_time is None:
            return None
        return round(float(sim_time) - float(self.tunnel_start_sim_time), 4)

    def _lane_event_during_tunnel(self, event):
        if self.tunnel_entry_sim_time is None:
            return False
        event_time = float(event["sim_time"])
        if event_time + 1e-6 < self.tunnel_entry_sim_time:
            return False
        return (self.tunnel_exit_sim_time is None
                or event_time <= self.tunnel_exit_sim_time + 1e-6)

    def _sample_runtime(self, now):
        location = self.ego.get_location()
        current = _location_payload(location)
        previous = self.tunnel_previous_location
        if previous is not None:
            step = math.hypot(
                current["x"] - previous["x"], current["y"] - previous["y"])
            crosses_entry = gate_crossing(
                previous, current, self.tunnel_entry_gate)
            crosses_exit = gate_crossing(
                previous, current, self.tunnel_exit_gate)

            if crosses_entry and self.tunnel_entry_sim_time is None:
                self.tunnel_entry_sim_time = now
                self.tunnel_phase = "IN_TUNNEL"
            if crosses_exit and self.tunnel_entry_sim_time is None:
                self._wrong_gate_order = True

            if self.tunnel_phase == "IN_TUNNEL":
                self.tunnel_driven_distance_m += step
                projection = project_to_route(current, self.tunnel_corridor)
                lateral = float(projection["distance_m"])
                self.max_tunnel_lateral_error_m = max(
                    self.max_tunnel_lateral_error_m, lateral)
                segment = min(
                    projection["segment_index"], len(self.tunnel_corridor) - 2)
                lane_width = min(
                    float(self.tunnel_corridor[segment].get("lane_width", 3.5)),
                    float(self.tunnel_corridor[segment + 1].get("lane_width", 3.5)))
                allowed = lane_width * 0.5 + CORRIDOR_CENTER_TOLERANCE_M
                if lateral > allowed:
                    self.same_lane = False
                if crosses_exit:
                    self.tunnel_exit_sim_time = now
                    self.tunnel_phase = "EXITED"

        if any(self._lane_event_during_tunnel(event)
               for event in self._lane_invasion_events):
            self.same_lane = False

        velocity = self.ego.get_velocity()
        speed_mps = math.sqrt(
            float(velocity.x) ** 2 + float(velocity.y) ** 2
            + float(getattr(velocity, "z", 0.0)) ** 2)
        try:
            light_state = str(self.ego.get_light_state())
        except (AttributeError, RuntimeError):
            light_state = None
        lateral = None
        if self.tunnel_phase in ("IN_TUNNEL", "EXITED"):
            lateral = round(project_to_route(
                current, self.tunnel_corridor)["distance_m"], 4)
        self._telemetry.append({
            "trial_time_s": self._relative_time(now),
            "phase": self.tunnel_phase,
            "x": round(current["x"], 4),
            "y": round(current["y"], 4),
            "speed_mps": round(speed_mps, 4),
            "tunnel_lateral_error_m": lateral,
            "light_state": light_state,
        })
        self.tunnel_previous_location = current

    def tick(self):
        if not self.ego:
            return True
        snapshot = self.world.get_snapshot()
        now = float(snapshot.timestamp.elapsed_seconds)
        self._sample_runtime(now)

        if self._collision_recorded or not self.same_lane or self._wrong_gate_order:
            self.tunnel_phase = "FAILED"
            return False
        if now - self.tunnel_start_sim_time >= self.tunnel_timeout_s:
            self.timed_out = True
            self.tunnel_phase = "TIMEOUT"
            return False

        if self.model == "tcp":
            return self._tick_tcp(snapshot)

        parent_running = super().tick()
        if not parent_running:
            self.route_complete = bool(self.finished)
            self.tunnel_phase = (
                "COMPLETE" if self.tunnel_exit_sim_time is not None else "FAILED")
            return False
        return True

    def get_result(self, collision_occurred=False):
        collision = bool(collision_occurred or self._collision_recorded)
        if any(self._lane_event_during_tunnel(event)
               for event in self._lane_invasion_events):
            self.same_lane = False
        failures = []
        if self.tunnel_entry_sim_time is None:
            failures.append("tunnel_entry_not_crossed")
        if self.tunnel_exit_sim_time is None:
            failures.append("tunnel_exit_not_crossed")
        if self._wrong_gate_order:
            failures.append("tunnel_exit_crossed_before_entry")
        if not self.same_lane:
            failures.append("did_not_remain_in_same_lane")
        if not self.route_complete:
            failures.append("route_not_completed")
        if collision:
            failures.append("collision")
        if self.timed_out:
            failures.append("scenario_timeout")
        controller_error = getattr(self, "_tcp_error", None)
        if controller_error:
            failures.append("tcp_controller_error: " + controller_error)

        speeds = [float(item["speed_mps"]) for item in self._telemetry]
        entry_time = self._relative_time(self.tunnel_entry_sim_time)
        entry_sample = next((item for item in self._telemetry
                             if entry_time is not None
                             and item["trial_time_s"] >= entry_time), None)
        lights = sorted({item["light_state"] for item in self._telemetry
                         if item.get("light_state") is not None})
        passed = not failures
        return {
            "controller": self.model,
            "weather_id": self.tunnel_cfg.get("weather_id"),
            "start_distance_m": self.tunnel_cfg.get("expansion", {}).get("start_distance_m"),
            "spawn_offset_m": self.tunnel_cfg.get("expansion", {}).get("spawn_offset_m"),
            "requested_spawn_offset_m": self.tunnel_cfg.get("expansion", {}).get("requested_spawn_offset_m"),
            "baseline_route_fingerprint": self.tunnel_cfg.get("expansion", {}).get("baseline_route_fingerprint"),
            "route_fingerprint": self.tunnel_cfg.get("route_fingerprint"),
            "condition_fingerprint": self.tunnel_cfg.get("condition_fingerprint"),
            "map_name": self.tunnel_map_name,
            "opendrive_sha256": self.tunnel_opendrive_sha256,
            "screening_only": True,
            "physical_field_test_compliance_claimed": False,
            "trial_valid": not bool(controller_error),
            "controller_error": controller_error,
            "controller_fallback_used": False,
            "tcp_control_samples": list(getattr(self, "_tcp_control_samples", [])),
            "tcp_frames_applied": len(getattr(self, "_tcp_control_samples", [])),
            "tcp_camera_timeout_s": self.tcp_frame_timeout_s if self.model == "tcp" else None,
            "entry_crossed": self.tunnel_entry_sim_time is not None,
            "exit_crossed": self.tunnel_exit_sim_time is not None,
            "entry_time_s": entry_time,
            "approach_time_s": entry_time,
            "entry_speed_mps": entry_sample["speed_mps"] if entry_sample else None,
            "exit_time_s": self._relative_time(self.tunnel_exit_sim_time),
            "same_lane": bool(self.same_lane),
            "route_complete": bool(self.route_complete),
            "measured_tunnel_length_m": round(
                float(self.measured_tunnel_length_m), 4),
            "tunnel_driven_distance_m": round(
                float(self.tunnel_driven_distance_m), 4),
            "max_tunnel_lateral_error_m": round(
                float(self.max_tunnel_lateral_error_m), 4),
            "lane_invasion": sum(
                self._lane_event_during_tunnel(event)
                for event in self._lane_invasion_events),
            "lane_invasion_events": list(self._lane_invasion_events),
            "collision": collision,
            "timed_out": bool(self.timed_out),
            "average_speed_mps": (
                round(sum(speeds) / len(speeds), 4) if speeds else 0.0),
            "maximum_speed_mps": round(max(speeds), 4) if speeds else 0.0,
            "observed_light_states": lights,
            "telemetry_samples": list(self._telemetry),
            "pass": passed if not controller_error else None,
            "failure_reasons": failures,
        }

    def destroy(self):
        if self.model == "tcp" and getattr(self, "camera_sensor", None) is not None:
            try:
                self.camera_sensor.stop()
            except (AttributeError, RuntimeError):
                pass
        if self._lane_invasion_sensor is not None:
            try:
                self._lane_invasion_sensor.stop()
            except (AttributeError, RuntimeError):
                pass
        super().destroy()
