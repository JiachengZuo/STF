import carla
import json
import pygame
import argparse
import os
import hashlib
import platform
import subprocess
import sys
import imageio
import numpy as np
import pandas as pd
import random
import atexit
import threading
import time
from datetime import datetime, timezone
from scene import PedestrianCrossScene, BicycleCrossScene,StaticPedestrianCrossScene, CarCrossScene, StaticCarCrossScene, StaticObstacleScene, OccludedPedestrianScene, CarCutOutScene, CarCutInScene, CarOncomingPassScene, CarStopandGoScene, CarCutOutandStaticScene, CarGoandStopScene, EgoRouteFollowScene, MotorVehicleTrafficLightScene
from render import Visualizer
from collision_enhancer import load_collision_config
from traffic_light_1d import (
    canonical_hash,
    file_sha256,
    load_existing_result_records as load_generic_result_records,
    validate_signal_matrix,
    write_result_artifacts as write_generic_result_artifacts,
)
from roundabout_2b import (
    load_existing_roundabout_records,
    roundabout_record_summary,
    roundabout_route_fingerprint,
    validate_roundabout_config,
    validate_roundabout_matrix,
    write_roundabout_result_artifacts,
)
# 待开发功能
# 1. 33个场景的专用类（完成）
# 2. 24种天气（完成）
# 3. 摆放agent（完成）
# 4. 衔接TCP
# ====================== 全局配置 ======================
HOST = '127.0.0.1'
PORT = 2000
FPS = 20

SCENE_CLASSES = {
    '1d': MotorVehicleTrafficLightScene,
    '3a': PedestrianCrossScene,
    '2b': EgoRouteFollowScene,
    '2c': CarCrossScene,
    '2d': CarCrossScene,
    '2e': CarCrossScene,
    '2g': StaticCarCrossScene,
    '2f': StaticObstacleScene,
    '3b_1': PedestrianCrossScene,
    '3b_2': PedestrianCrossScene,
    '3c': BicycleCrossScene,
    '3d': OccludedPedestrianScene,
    '4a': CarCutInScene,
    '4b': CarCutOutScene,
    '4c': CarOncomingPassScene,
    '4d': CarStopandGoScene,
    '5a': PedestrianCrossScene,
    '5b': BicycleCrossScene,
    '5c': CarCutOutandStaticScene,
    '5d': CarGoandStopScene,
    '5e': CarCrossScene,
    '5f': StaticPedestrianCrossScene,
    '5g': PedestrianCrossScene,
    '6a': EgoRouteFollowScene,
    '6b': EgoRouteFollowScene,
    '6c': EgoRouteFollowScene,
}

def get_sorted_scenario_files(input_dir, scenario_code=None):
    files = []
    prefix = "scenario_"
    print(prefix)
    for f in os.listdir(input_dir):
        if f.startswith(prefix) and f.endswith('.json'):
            files.append(f)
    files.sort()
    return [os.path.join(input_dir, f) for f in files]


def load_scenario_document(json_path):
    with open(json_path, 'r', encoding='utf-8') as stream:
        return json.load(stream)


def apply_reproducible_seed(seed, client=None):
    """Seed local inference and CARLA Traffic Manager for one trial."""
    if seed is None:
        return
    seed = int(seed) % (2 ** 32)
    random.seed(seed)
    np.random.seed(seed)
    try:
        import torch
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        if hasattr(torch.backends, "cudnn"):
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
    except ImportError:
        pass
    if client is not None:
        try:
            client.get_trafficmanager(8000).set_random_device_seed(seed)
        except RuntimeError:
            pass


def load_result_records(output_dir, scenario_code):
    """Load the scenario-specific durable ledger."""
    if scenario_code == "2b":
        return load_existing_roundabout_records(output_dir, scenario_code)
    return load_generic_result_records(output_dir, scenario_code)


def persist_result_records(records, output_dir, scenario_code, metadata=None):
    """Write the scenario-specific report and structured result files."""
    if scenario_code == "2b":
        return write_roundabout_result_artifacts(
            records, output_dir, scenario_code, metadata)
    return write_generic_result_artifacts(
        records, output_dir, scenario_code, metadata)


def _safe_path_component(value, fallback):
    value = str(value or "").strip().lower()
    value = "".join(
        character if character.isalnum() or character in "-_" else "-"
        for character in value).strip("-_")
    return value or fallback


def _roundabout_output_identity(scenario_config):
    test = scenario_config.get("roundabout_test", {}) or {}
    generation = test.get("generation", {}) or {}
    try:
        route_fingerprint = roundabout_route_fingerprint(scenario_config)
    except ValueError:
        route_fingerprint = test.get("topology_hash") or "unknown"
    route_id = generation.get("route_id")
    if not route_id:
        name = str(scenario_config.get("name") or "")
        route_token = next(
            (token for token in name.split("_") if token.isdigit()), None)
        route_id = "route_{}".format(route_token or str(route_fingerprint)[:10])
    condition_fingerprint = (
        generation.get("condition_fingerprint")
        or (test.get("reproducibility") or {}).get("condition_fingerprint")
        or (test.get("reproducibility") or {}).get("route_fingerprint")
        or canonical_hash(scenario_config))
    condition_id = generation.get("condition_id") or "condition"
    capability = "capable" if test.get("roundabout_capable", True) else "incapable"
    route_directory = "{}_exit{}_{}".format(
        _safe_path_component(route_id, "route_unknown"),
        test.get("planned_exit", "unknown"), capability)
    condition_directory = "{}__c_{}".format(
        _safe_path_component(condition_id, "condition"),
        str(condition_fingerprint)[:10])
    return {
        "drawn_route_id": route_id,
        "route_definition_fingerprint": route_fingerprint,
        "condition_id": condition_id,
        "condition_fingerprint": condition_fingerprint,
        "route_directory": route_directory,
        "condition_directory": condition_directory,
    }


def _vector_json(vector):
    return {
        "x": round(float(getattr(vector, "x", 0.0)), 6),
        "y": round(float(getattr(vector, "y", 0.0)), 6),
        "z": round(float(getattr(vector, "z", 0.0)), 6),
    }


def _actor_runtime_snapshot(actor):
    if actor is None:
        return None

    # CARLA actor proxies remain as Python objects after their simulator actor
    # has been destroyed.  Identity fields may still be cached, but RPC-backed
    # calls such as get_transform() raise RuntimeError.  A target fixture that
    # has completed its post-exit clearance is intentionally destroyed before
    # the terminal record is assembled, so represent that lifecycle state
    # instead of turning an otherwise valid attempt into a runner INVALID.
    try:
        alive = bool(actor.is_alive)
    except (AttributeError, RuntimeError):
        alive = False
    try:
        actor_id = getattr(actor, "id", None)
    except RuntimeError:
        actor_id = None
    try:
        type_id = getattr(actor, "type_id", None)
    except RuntimeError:
        type_id = None
    try:
        attributes = dict(getattr(actor, "attributes", {}) or {})
    except (RuntimeError, TypeError, ValueError):
        attributes = {}
    if not alive:
        return {
            "actor_id": actor_id,
            "type_id": type_id,
            "attributes": attributes,
            "alive": False,
            "transform": None,
            "bounding_box_extent_m": None,
        }

    try:
        transform = actor.get_transform()
        bbox = getattr(actor, "bounding_box", None)
        extent = getattr(bbox, "extent", None)
    except (AttributeError, RuntimeError):
        return {
            "actor_id": actor_id,
            "type_id": type_id,
            "attributes": attributes,
            "alive": False,
            "transform": None,
            "bounding_box_extent_m": None,
        }
    return {
        "actor_id": actor_id,
        "type_id": type_id,
        "attributes": attributes,
        "alive": True,
        "transform": {
            "location": _vector_json(transform.location),
            "rotation": {
                "roll": round(float(transform.rotation.roll), 6),
                "pitch": round(float(transform.rotation.pitch), 6),
                "yaw": round(float(transform.rotation.yaw), 6),
            },
        },
        "bounding_box_extent_m": _vector_json(extent) if extent is not None else None,
    }


def _weather_snapshot(weather):
    fields = (
        "cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
        "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "fog_distance",
        "wetness", "fog_falloff", "scattering_intensity", "mie_scattering_scale",
        "rayleigh_scattering_scale", "dust_storm")
    return {
        field: round(float(getattr(weather, field)), 6)
        for field in fields if hasattr(weather, field)
    }


def _git_runtime_snapshot(repo_dir):
    snapshot = {"commit": None, "dirty": None}
    try:
        snapshot["commit"] = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo_dir,
            stderr=subprocess.DEVNULL, text=True).strip()
        snapshot["dirty"] = bool(subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=repo_dir,
            stderr=subprocess.DEVNULL, text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        pass
    return snapshot


def _runtime_environment(client, world, args):
    opendrive = world.get_map().to_opendrive()
    packages = {}
    try:
        from importlib import metadata as importlib_metadata
        for package in ("numpy", "pandas", "pygame", "imageio", "torch", "torchvision"):
            try:
                packages[package] = importlib_metadata.version(package)
            except importlib_metadata.PackageNotFoundError:
                packages[package] = None
    except ImportError:
        pass
    cuda = {"available": False, "version": None, "device": None}
    try:
        import torch
        cuda["available"] = bool(torch.cuda.is_available())
        cuda["version"] = getattr(torch.version, "cuda", None)
        if cuda["available"]:
            cuda["device"] = torch.cuda.get_device_name(0)
    except ImportError:
        pass
    return {
        "repo": _git_runtime_snapshot(os.path.dirname(os.path.abspath(__file__))),
        "launcher_config_path": args.launcher_config_path,
        "launcher_config_sha256": args.launcher_config_sha256,
        "batch_id": args.batch_id,
        "python": sys.version,
        "executable": sys.executable,
        "packages": packages,
        "platform": platform.platform(),
        "processor": platform.processor(),
        "cuda": cuda,
        "carla_client_version": client.get_client_version(),
        "carla_server_version": client.get_server_version(),
        "map_name": world.get_map().name,
        "opendrive_sha256": hashlib.sha256(opendrive.encode("utf-8")).hexdigest(),
        "synchronous_mode": True,
        "fixed_delta_seconds": 0.05,
        "telemetry_frequency_hz": 20,
        "behavior_implementation": (
            "scene.EgoRouteFollowScene.follow_route; repository reference controller, "
            "not CARLA BehaviorAgent"),
        "tcp_implementation": "model.tcp.TCPAgent.get_action",
    }


def _enrich_video_artifact(record):
    path = record.get("video_path")
    metadata = dict(record.get("video_metadata") or {})
    metadata.update({
        "path": os.path.abspath(path) if path else None,
        "width_px": 512,
        "height_px": 256,
        "fps": FPS,
        "codec": "libx264",
        "purpose": "simulation_visualization",
        "complete": bool(path and os.path.isfile(path)),
    })
    if metadata["complete"]:
        metadata["sha256"] = file_sha256(path)
        metadata["size_bytes"] = os.path.getsize(path)
    record["video_metadata"] = metadata

# ======================
# ✅ 新增：从JSON加载天气并设置CARLA
# ======================
def load_weather_from_json(world, json_path, town_name, route_id="route_01"):
    """Apply configured weather and return ``(success, error_message)``."""
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        scenario = data[town_name][route_id][0]
        if "weather" not in scenario:
            print("ℹ️ No weather in JSON, use default")
            return False, "weather_missing"

        w = scenario["weather"]
        weather = carla.WeatherParameters(
            cloudiness=w.get("cloudiness", 0.0),
            dust_storm=w.get("dust_storm", 0.0),
            fog_density=w.get("fog_density", 0.0),
            fog_distance=w.get("fog_distance", 100.0),
            fog_falloff=w.get("fog_falloff", 1.0),
            mie_scattering_scale=w.get("mie_scattering_scale", 0.03),
            precipitation=w.get("precipitation", 0.0),
            precipitation_deposits=w.get("precipitation_deposits", 0.0),
            rayleigh_scattering_scale=w.get("rayleigh_scattering_scale", 0.0331),
            scattering_intensity=w.get("scattering_intensity", 1.0),
            sun_altitude_angle=w.get("sun_altitude_angle", 60.0),
            sun_azimuth_angle=w.get("sun_azimuth_angle", 120.0),
            wetness=w.get("wetness", 0.0),
            wind_intensity=w.get("wind_intensity", 5.0)
        )
        world.set_weather(weather)
        print(f"✅ Weather loaded from JSON: {os.path.basename(json_path)}")
        return True, None
    except Exception as e:
        error = "{}: {}".format(type(e).__name__, e)
        print(f"❌ Weather load failed: {error}")
        return False, error


def _sensor_event_time(event, world):
    """Return the simulator timestamp carried by a sensor event when available."""
    timestamp = getattr(event, "timestamp", None)
    if timestamp is not None:
        if hasattr(timestamp, "elapsed_seconds"):
            return float(timestamp.elapsed_seconds)
        try:
            return float(timestamp)
        except (TypeError, ValueError):
            pass
    return float(world.get_snapshot().timestamp.elapsed_seconds)


def _roundabout_record_metadata(scenario_document, town, route_id):
    """Extract stable 2.b identifiers for setup failures and retry bookkeeping."""
    try:
        test_config = scenario_document[town][route_id][0].get("roundabout_test", {})
        reproducibility = test_config.get("reproducibility", {}) or {}
        topology = test_config.get("topology", {}) or {}
        return {
            "matrix_id": test_config.get("matrix_id")
            or reproducibility.get("matrix_id"),
            "planned_exit": test_config.get("planned_exit"),
            "roundabout_capable": test_config.get("roundabout_capable", True),
            "trial_index": reproducibility.get("trial_index"),
            "base_seed": reproducibility.get("base_seed"),
            "matrix_seed": reproducibility.get("matrix_seed"),
            "trial_seed": reproducibility.get("trial_seed"),
            "route_fingerprint": reproducibility.get("route_fingerprint"),
            "topology_hash": reproducibility.get("topology_hash")
            or test_config.get("topology_hash")
            or (topology.get("topology_hash") if isinstance(topology, dict) else None),
        }
    except (KeyError, IndexError, TypeError):
        return {}


def _roundabout_invalid_record(
        scenario_name, run_id, cfg_path, config_sha256, attempt_index,
        scenario_document, town, route_id, reason):
    """Build an INVALID setup/harness result without assigning an SUT verdict."""
    record = {
        "scenario": scenario_name,
        "run_id": run_id,
        "input_file": os.path.abspath(cfg_path),
        "config_sha256": config_sha256,
        "attempt_index": int(attempt_index),
        "pass": None,
        "valid_trial": False,
        "trial_valid": False,
        "precondition_valid": False,
        "phase": "INVALID",
        "route_complete": False,
        "invalid_reason": reason,
        "invalid_reasons": [reason],
        "failure_reasons": ["invalid_precondition:{}".format(reason)],
        "collision": False,
        "collision_events": [],
        "lane_invasion_events": [],
    }
    record.update(_roundabout_record_metadata(
        scenario_document, town, route_id))
    return record


def _safe_sensor_stop(sensor):
    if sensor is None:
        return
    try:
        sensor.stop()
    except Exception:
        pass


def _safe_destroy(obj):
    if obj is None:
        return
    try:
        obj.destroy()
    except Exception:
        pass


def spawn_traffic_npcs(world, client, total_npcs=0, car_ratio=0.5,
                       cyclist_ratio=0.3):
    """Spawn optional Traffic Manager vehicles and AI walkers for one attempt."""
    if total_npcs <= 0:
        return []
    if car_ratio < 0.0 or cyclist_ratio < 0.0 or car_ratio + cyclist_ratio > 1.0:
        raise ValueError("NPC ratios must be non-negative and sum to at most 1.0")

    blueprint_library = world.get_blueprint_library()
    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)
    if not spawn_points:
        raise RuntimeError("CARLA map has no vehicle spawn points for NPC traffic")

    car_count = int(total_npcs * car_ratio)
    cyclist_count = int(total_npcs * cyclist_ratio)
    pedestrian_count = total_npcs - car_count - cyclist_count
    actors = []
    vehicles = []
    car_blueprints = [
        blueprint for blueprint in blueprint_library.filter("vehicle.*")
        if all(token not in blueprint.id for token in (
            "bicycle", "motorcycle", "ambulance", "firetruck"))
    ]
    cyclist_blueprints = [
        blueprint for blueprint in blueprint_library.filter("vehicle.*")
        if "bicycle" in blueprint.id or "motorcycle" in blueprint.id
    ]

    requested_vehicles = (
        [("car", car_blueprints)] * car_count
        + [("cyclist", cyclist_blueprints)] * cyclist_count)
    for index, (kind, choices) in enumerate(requested_vehicles):
        if index >= len(spawn_points) or not choices:
            print("⚠️ NPC {} skipped: no blueprint/spawn point".format(kind))
            continue
        blueprint = random.choice(choices)
        if blueprint.has_attribute("color"):
            colors = blueprint.get_attribute("color").recommended_values
            if colors:
                blueprint.set_attribute("color", random.choice(colors))
        actor = world.try_spawn_actor(blueprint, spawn_points[index])
        if actor is not None:
            vehicles.append(actor)
            actors.append(actor)

    if vehicles:
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        for vehicle in vehicles:
            vehicle.set_autopilot(True, traffic_manager.get_port())
            traffic_manager.vehicle_percentage_speed_difference(
                vehicle, random.uniform(-20.0, 20.0))

    walker_blueprints = list(
        blueprint_library.filter("walker.pedestrian.*"))
    for _ in range(pedestrian_count):
        if not walker_blueprints:
            break
        location = world.get_random_location_from_navigation()
        if location is None:
            continue
        walker = world.try_spawn_actor(
            random.choice(walker_blueprints), carla.Transform(location))
        if walker is None:
            continue
        actors.append(walker)
        controller = world.try_spawn_actor(
            blueprint_library.find("controller.ai.walker"),
            carla.Transform(), attach_to=walker)
        if controller is None:
            continue
        actors.append(controller)
        controller.start()
        destination = world.get_random_location_from_navigation()
        if destination is not None:
            controller.go_to_location(destination)
        controller.set_max_speed(1.4 + random.random() * 0.6)

    print("🏙️ NPC traffic: requested={}, spawned={}".format(
        total_npcs, len(actors)))
    return actors


def _destroy_traffic_npcs(actors):
    """Stop walker controllers first, then destroy all optional traffic actors."""
    for actor in reversed(list(actors or [])):
        try:
            if str(getattr(actor, "type_id", "")).startswith(
                    "controller.ai.walker"):
                actor.stop()
        except (RuntimeError, AttributeError):
            pass
        _safe_destroy(actor)


def _completed_scenario_names(records, scenario_code):
    """Return scenarios with a durable completed result for --resume."""
    completed = set()
    for record in records:
        if not isinstance(record, dict) or not record.get("scenario"):
            continue
        if scenario_code == "2b" and record.get("trial_valid") is not True:
            continue
        completed.add(str(record["scenario"]))
    return completed


def _run_roundabout_attempt(
        client, world, cfg_path, town, route_id, model, model_path,
        video_dir, scenario_name, run_id, config_sha256, attempt_index,
        scenario_document, runtime_environment, execution_mode,
        npc_total=0, npc_car_ratio=0.5, npc_cyclist_ratio=0.3):
    """Run one 2.b attempt with frame-bounded sensors and deterministic cleanup."""
    scene = None
    writer = None
    viz = None
    collision_sensor = None
    lane_invasion_sensor = None
    fixture_collision_sensors = []
    heartbeat_sensor = None
    npc_actors = []
    sensor_stop_completed = False
    evidence_cutoff_frame = None
    evidence_barrier_ok = True
    callback_condition = threading.Condition()
    heartbeat_frame = -1
    last_evidence_callback_time = 0.0
    spawn_complete = False
    shutdown_requested = False
    collision_occurred = False
    collision_events = []
    lane_invasion_events = []
    fixture_collision_events = []
    previous_location = None
    total_distance = 0.0
    speed_sum = 0.0
    speed_sample_count = 0
    max_ego_speed = 0.0
    start_frame = None
    terminal_frame = None
    video_start_sim_time = None
    video_end_sim_time = None
    recorded_video_frames = 0

    ego_velocity = 0.0
    ego_acc_x = ego_acc_y = ego_acc_z = 0.0
    ego_x = ego_y = ego_z = 0.0
    ego_roll = ego_pitch = ego_yaw = 0.0
    scenario_config = scenario_document[town][route_id][0]
    output_identity = _roundabout_output_identity(scenario_config)
    reproducibility = scenario_config.get("roundabout_test", {}).get(
        "reproducibility", {}) or {}
    if execution_mode == "screening":
        attempt_directory = os.path.join(
            video_dir, output_identity["route_directory"],
            output_identity["condition_directory"], "screening",
            "attempt_{:02d}".format(int(attempt_index)))
    else:
        attempt_directory = os.path.join(
            video_dir, output_identity["route_directory"],
            output_identity["condition_directory"],
            "trial_{:02d}".format(int(reproducibility.get("trial_index") or 0)),
            "attempt_{:02d}".format(int(attempt_index)))
    os.makedirs(attempt_directory, exist_ok=True)
    video_path = os.path.join(attempt_directory, "visualization.mp4")

    def stop_sensor_listening(drain=False, terminal_frame=None):
        nonlocal sensor_stop_completed, evidence_cutoff_frame, evidence_barrier_ok
        if sensor_stop_completed:
            return evidence_barrier_ok
        sensors = [collision_sensor, lane_invasion_sensor, heartbeat_sensor] \
            + list(fixture_collision_sensors)
        sensors = [sensor for sensor in sensors if sensor is not None]
        if not sensors:
            sensor_stop_completed = True
            return not drain
        if drain:
            evidence_cutoff_frame = int(
                terminal_frame if terminal_frame is not None
                else world.get_snapshot().frame)
            try:
                if scene is not None and getattr(scene, "ego", None) is not None:
                    scene.ego.apply_control(carla.VehicleControl(
                        throttle=0.0, steer=0.0, brake=1.0))
                target_frame = evidence_cutoff_frame + 2
                while int(world.get_snapshot().frame) < target_frame:
                    world.tick()
                deadline = time.monotonic() + 1.0
                with callback_condition:
                    while heartbeat_frame < target_frame:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            evidence_barrier_ok = False
                            break
                        callback_condition.wait(remaining)
                    while evidence_barrier_ok:
                        quiet_for = time.monotonic() - last_evidence_callback_time
                        if quiet_for >= 0.02:
                            break
                        remaining = min(0.02 - quiet_for, deadline - time.monotonic())
                        if remaining <= 0.0:
                            evidence_barrier_ok = False
                            break
                        callback_condition.wait(remaining)
            except (RuntimeError, AttributeError, TypeError, ValueError):
                evidence_barrier_ok = False
        for sensor in sensors:
            _safe_sensor_stop(sensor)
        sensor_stop_completed = True
        return evidence_barrier_ok

    def base_record():
        average_speed = speed_sum / speed_sample_count if speed_sample_count else None
        record = {
            "scenario": scenario_name,
            "run_id": run_id,
            "input_file": os.path.abspath(cfg_path),
            "config_sha256": config_sha256,
            "attempt_index": int(attempt_index),
            "video_path": os.path.abspath(video_path),
            "ego_velocity": round(float(ego_velocity), 2),
            "ego_acceleration_x": round(float(ego_acc_x), 2),
            "ego_acceleration_y": round(float(ego_acc_y), 2),
            "ego_acceleration_z": round(float(ego_acc_z), 2),
            "ego_x": round(float(ego_x), 2),
            "ego_y": round(float(ego_y), 2),
            "ego_z": round(float(ego_z), 2),
            "ego_roll": round(float(ego_roll), 2),
            "ego_pitch": round(float(ego_pitch), 2),
            "ego_yaw": round(float(ego_yaw), 2),
            "current_game_time": round(
                float(world.get_snapshot().timestamp.elapsed_seconds), 1),
            "driven_distance": round(float(total_distance), 4),
            "driven_distance_m": round(float(total_distance), 4),
            "average_velocity": round(float(average_speed), 4)
            if average_speed is not None else None,
            "average_speed_mps": round(float(average_speed), 4)
            if average_speed is not None else None,
            "maximum_ego_speed_mps": round(float(max_ego_speed), 4),
            "lane_invasion": len(lane_invasion_events),
            "collision": bool(collision_occurred),
            "run_red_light": None,
            "run_stop": None,
            "collision_events": list(collision_events),
            "lane_invasion_events": list(lane_invasion_events),
            "fixture_collision_events": list(fixture_collision_events),
            "sensor_frame_barrier_ok": bool(evidence_barrier_ok),
            "scenario_snapshot": scenario_config,
            "runtime_environment": runtime_environment,
            "generation": scenario_config.get("roundabout_test", {}).get(
                "generation", {}),
            "execution_mode": execution_mode,
            "evidence_profile": "stf_carla_simulation",
            "kinematics_source": "carla_ground_truth",
            "physical_field_test_compliance_claimed": False,
            "parameter_correspondence": {
                "standard_test_definition": "GB/T 41798-2022 6.2.2",
                "standard_general_recording": "GB/T 41798-2022 4.2.3",
                "formal_repetition_rule": "GB/T 41798-2022 5.2.1",
                "simulation_engineering_parameters": "roundabout_test.engineering",
                "cabin_video_audio": {
                    "status": "not_applicable",
                    "reason": "CARLA simulation has no physical cabin or cabin audio source",
                },
            },
            "actual_weather": _weather_snapshot(world.get_weather()),
            "actual_actors": {
                "vut": _actor_runtime_snapshot(getattr(scene, "ego", None)),
                "vt1": _actor_runtime_snapshot(getattr(scene, "vt1_actor", None)),
                "vt2": _actor_runtime_snapshot(getattr(scene, "vt2_actor", None)),
            },
            "video_metadata": {
                "start_frame": start_frame,
                "end_frame": terminal_frame,
                "start_sim_time": round(video_start_sim_time, 4)
                if video_start_sim_time is not None else None,
                "end_sim_time": round(video_end_sim_time, 4)
                if video_end_sim_time is not None else None,
                "duration_s": round(
                    max(0.0, video_end_sim_time - video_start_sim_time), 4)
                if (video_start_sim_time is not None
                    and video_end_sim_time is not None) else None,
                "frame_count": int(recorded_video_frames),
                "recording_scope": "roundabout_trial_clock",
            },
        }
        record.update(output_identity)
        return record

    try:
        scene_class = SCENE_CLASSES.get("2b")
        if scene_class is None:
            raise ValueError("Unsupported scenario: 2b")
        scene = scene_class(
            client, world, cfg_path, town, route_id, model, model_path)
        scene.spawn()
        npc_actors = spawn_traffic_npcs(
            world, client, npc_total, npc_car_ratio, npc_cyclist_ratio)
        spawn_complete = True

        def on_collision(event):
            nonlocal collision_occurred, last_evidence_callback_time
            if (evidence_cutoff_frame is not None
                    and int(getattr(event, "frame", -1)) > evidence_cutoff_frame):
                return
            with callback_condition:
                last_evidence_callback_time = time.monotonic()
                callback_condition.notify_all()
            collision_occurred = True
            event_time = _sensor_event_time(event, world)
            impulse = getattr(event, "normal_impulse", None)
            impulse_magnitude = None
            if impulse is not None:
                impulse_magnitude = float(np.sqrt(
                    impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2))
            other_actor = getattr(event, "other_actor", None)
            collision_events.append({
                "frame": getattr(event, "frame", None),
                "sim_time": round(event_time, 4),
                "other_actor_id": getattr(other_actor, "id", None),
                "other_actor_type": getattr(other_actor, "type_id", None),
                "impulse": round(impulse_magnitude, 4)
                if impulse_magnitude is not None else None,
            })
            if hasattr(scene, "record_collision"):
                scene.record_collision(event)

        def on_lane_invasion(event):
            nonlocal last_evidence_callback_time
            if (evidence_cutoff_frame is not None
                    and int(getattr(event, "frame", -1)) > evidence_cutoff_frame):
                return
            with callback_condition:
                last_evidence_callback_time = time.monotonic()
                callback_condition.notify_all()
            event_time = _sensor_event_time(event, world)
            markings = []
            for marking in getattr(event, "crossed_lane_markings", []):
                markings.append({
                    "type": str(getattr(marking, "type", "Unknown")),
                    "color": str(getattr(marking, "color", "Unknown")),
                    "lane_change": str(getattr(marking, "lane_change", "Unknown")),
                })
            lane_invasion_events.append({
                "frame": getattr(event, "frame", None),
                "sim_time": round(event_time, 4),
                "markings": markings,
            })
            if hasattr(scene, "record_lane_invasion"):
                scene.record_lane_invasion(event)

        def on_fixture_collision(role, event):
            nonlocal last_evidence_callback_time
            if (evidence_cutoff_frame is not None
                    and int(getattr(event, "frame", -1)) > evidence_cutoff_frame):
                return
            with callback_condition:
                last_evidence_callback_time = time.monotonic()
                callback_condition.notify_all()
            other_actor = getattr(event, "other_actor", None)
            fixture_collision_events.append({
                "role": role,
                "frame": getattr(event, "frame", None),
                "sim_time": round(_sensor_event_time(event, world), 4),
                "other_actor_id": getattr(other_actor, "id", None),
                "other_actor_type": getattr(other_actor, "type_id", None),
            })
            if hasattr(scene, "record_fixture_collision"):
                scene.record_fixture_collision(role, event)

        def on_heartbeat(event):
            nonlocal heartbeat_frame
            with callback_condition:
                heartbeat_frame = max(heartbeat_frame, int(getattr(event, "frame", -1)))
                callback_condition.notify_all()

        bp_lib = world.get_blueprint_library()
        collision_sensor = world.spawn_actor(
            bp_lib.find('sensor.other.collision'), carla.Transform(), attach_to=scene.ego)
        collision_sensor.listen(on_collision)
        lane_invasion_sensor = world.spawn_actor(
            bp_lib.find('sensor.other.lane_invasion'), carla.Transform(), attach_to=scene.ego)
        lane_invasion_sensor.listen(on_lane_invasion)
        for role, actor in (("vt1", getattr(scene, "vt1_actor", None)),
                            ("vt2", getattr(scene, "vt2_actor", None))):
            if actor is None:
                raise RuntimeError("2.b {} actor unavailable for collision evidence".format(
                    role.upper()))
            sensor = world.spawn_actor(
                bp_lib.find('sensor.other.collision'), carla.Transform(), attach_to=actor)
            sensor.listen(lambda event, role=role: on_fixture_collision(role, event))
            fixture_collision_sensors.append(sensor)
        heartbeat_bp = bp_lib.find('sensor.other.gnss')
        if heartbeat_bp.has_attribute('sensor_tick'):
            heartbeat_bp.set_attribute('sensor_tick', '0.0')
        heartbeat_sensor = world.spawn_actor(
            heartbeat_bp, carla.Transform(), attach_to=scene.ego)
        heartbeat_sensor.listen(on_heartbeat)

        viz = Visualizer(world, scene.ego)
        capable = bool(getattr(scene, "roundabout_capable", True))
        print(
            "[2.b] 证据键：O=人工确认真实 HMI 已发出 ODD 提示；"
            "I=人工确认被测车已开启驶出右转灯。")
        if not capable:
            print(
                "[2.b] 当前为不具备环岛能力分支：观察到真实 HMI 提示后按 O；"
                "VUT 必须停在入口前。")

        running = True
        while running:
            # 2.b evaluates the frame that has just been simulated.  This keeps
            # state, actor samples and sensor evidence on one CARLA clock/frame.
            world.tick()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    shutdown_requested = True
                    if hasattr(scene, "abort_roundabout"):
                        scene.abort_roundabout("operator_quit")
                    running = False
                    print("[2.b] 窗口关闭：本次试验记为 INVALID，停止后续试验。")
                    break
                if event.type != pygame.KEYDOWN:
                    continue
                if event.key == pygame.K_o:
                    if capable:
                        print("[2.b] O 已忽略：当前是具备环岛能力分支。")
                    elif hasattr(scene, "set_odd_alert"):
                        scene.set_odd_alert(True, source="operator_keyboard")
                        print("[2.b] 已记录人工观察到的真实 ODD/HMI 提示。")
                elif event.key == pygame.K_i:
                    if hasattr(scene, "record_turn_signal_observation"):
                        scene.record_turn_signal_observation(
                            True, source="operator_keyboard")
                        print("[2.b] 已记录人工确认的驶出右转灯证据；未改变车辆灯光。")

            if not running:
                break

            keep_running = scene.tick()

            snapshot = world.get_snapshot()
            trans = scene.ego.get_transform()
            vel = scene.ego.get_velocity()
            acc = scene.ego.get_acceleration()
            ego_velocity = float(np.sqrt(vel.x ** 2 + vel.y ** 2))
            ego_acc_x, ego_acc_y, ego_acc_z = acc.x, acc.y, acc.z
            ego_x, ego_y, ego_z = (
                trans.location.x, trans.location.y, trans.location.z)
            ego_roll, ego_pitch, ego_yaw = (
                trans.rotation.roll, trans.rotation.pitch, trans.rotation.yaw)

            trial_started = getattr(
                scene, "rb_trial_start_sim_time", None) is not None
            if trial_started:
                if previous_location is not None:
                    dx = trans.location.x - previous_location.x
                    dy = trans.location.y - previous_location.y
                    total_distance += float(np.sqrt(dx ** 2 + dy ** 2))
                previous_location = trans.location
                speed_sum += ego_velocity
                speed_sample_count += 1
                max_ego_speed = max(max_ego_speed, ego_velocity)

            waypoints = scene.get_future_waypoints(12)
            viz.render(waypoints)
            if trial_started:
                if writer is None:
                    start_frame = int(snapshot.frame)
                    video_start_sim_time = float(
                        snapshot.timestamp.elapsed_seconds)
                    writer = imageio.get_writer(
                        video_path, fps=FPS, codec='libx264')
                    print(
                        "[2.b] 试验时钟已启动；开始记录正式实验视频：{}".format(
                            video_path))
                frame = pygame.surfarray.array3d(viz.screen).swapaxes(0, 1)
                writer.append_data(frame)
                recorded_video_frames += 1
                terminal_frame = int(snapshot.frame)
                video_end_sim_time = float(snapshot.timestamp.elapsed_seconds)
            running = bool(keep_running)

        if terminal_frame is None:
            terminal_frame = int(world.get_snapshot().frame)
        barrier_ok = stop_sensor_listening(
            drain=True, terminal_frame=terminal_frame)
        record = base_record()
        if hasattr(scene, "get_result"):
            record.update(scene.get_result(collision_occurred=collision_occurred))
        if shutdown_requested:
            invalid_reasons = list(record.get("invalid_reasons") or [])
            if "operator_quit" not in invalid_reasons:
                invalid_reasons.append("operator_quit")
            record.update({
                "pass": None,
                "valid_trial": False,
                "trial_valid": False,
                "precondition_valid": False,
                "phase": "INVALID",
                "invalid_reason": invalid_reasons[0],
                "invalid_reasons": invalid_reasons,
                "failure_reasons": [
                    "invalid_precondition:{}".format(item)
                    for item in invalid_reasons
                ],
            })
        if not barrier_ok:
            invalid_reasons = list(record.get("invalid_reasons") or [])
            if "sensor_frame_barrier_timeout" not in invalid_reasons:
                invalid_reasons.append("sensor_frame_barrier_timeout")
            record.update({
                "pass": None,
                "valid_trial": False,
                "trial_valid": False,
                "precondition_valid": False,
                "phase": "INVALID",
                "invalid_reason": invalid_reasons[0],
                "invalid_reasons": invalid_reasons,
                "failure_reasons": [
                    "invalid_precondition:{}".format(item)
                    for item in invalid_reasons],
                "sensor_frame_barrier_ok": False,
            })
        record["attempt_index"] = int(attempt_index)
        return record, shutdown_requested

    except Exception as exc:
        failure_kind = "roundabout_runner_error" if spawn_complete else "spawn_failed"
        reason = "{}:{}: {}".format(failure_kind, type(exc).__name__, exc)
        print("❌ 2.b attempt failed: {}".format(reason))
        if spawn_complete and scene is not None and hasattr(scene, "abort_roundabout"):
            try:
                scene.abort_roundabout(reason)
            except Exception as abort_exc:
                print("❌ 2.b abort hook failed: {}".format(abort_exc))
        stop_sensor_listening(
            drain=spawn_complete,
            terminal_frame=int(world.get_snapshot().frame) if spawn_complete else None)

        if spawn_complete and scene is not None and hasattr(scene, "get_result"):
            try:
                record = base_record()
                record.update(scene.get_result(collision_occurred=collision_occurred))
                invalid_reasons = list(record.get("invalid_reasons") or [])
                if reason not in invalid_reasons:
                    invalid_reasons.append(reason)
                record.update({
                    "attempt_index": int(attempt_index),
                    "pass": None,
                    "valid_trial": False,
                    "trial_valid": False,
                    "precondition_valid": False,
                    "phase": "INVALID",
                    "invalid_reason": invalid_reasons[0],
                    "invalid_reasons": invalid_reasons,
                    "failure_reasons": [
                        "invalid_precondition:{}".format(item)
                        for item in invalid_reasons
                    ],
                })
                return record, shutdown_requested
            except Exception as result_exc:
                reason = "{}; result_error:{}: {}".format(
                    reason, type(result_exc).__name__, result_exc)

        return _roundabout_invalid_record(
            scenario_name, run_id, cfg_path, config_sha256, attempt_index,
            scenario_document, town, route_id, reason), shutdown_requested
    finally:
        stop_sensor_listening(drain=False)
        _safe_destroy(lane_invasion_sensor)
        _safe_destroy(collision_sensor)
        for sensor in fixture_collision_sensors:
            _safe_destroy(sensor)
        _safe_destroy(heartbeat_sensor)
        _destroy_traffic_npcs(npc_actors)
        if writer is not None:
            try:
                writer.close()
            except Exception:
                pass
        if viz is not None:
            _safe_destroy(viz)
        if scene is not None:
            _safe_destroy(scene)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default=HOST)
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--town', type=str, required=True)
    parser.add_argument('--route_id', type=str, default='route_01')
    parser.add_argument('--input_dir', type=str, required=True)
    parser.add_argument('--scenario', type=str, default='3a')
    parser.add_argument('--video_dir', type=str, default='videos')
    parser.add_argument('--model', choices=('behavior', 'tcp'), default='behavior')
    parser.add_argument('--model_path', type=str, default='./tcp/best_model.ckpt')
    parser.add_argument('--collision_config', type=str, default=None,
                        help='Path to collision enhancement config YAML (default: collision_config.yaml in tools dir)')
    parser.add_argument('--allow_partial_matrix', action='store_true',
                        help='Allow a complete 3-trial 1.d matrix for fewer than all three maneuvers')
    parser.add_argument(
        '--max_invalid_retries', type=int, default=2,
        help='Maximum automatic retries for each INVALID 2.b trial (default: 2)')
    parser.add_argument(
        '--screening', action='store_true',
        help='Run each standalone 2.b scenario definition once for screening')
    parser.add_argument('--batch_id', type=str, default=None,
                        help='Launcher batch identifier; enables exact structured output root')
    parser.add_argument('--launcher_config_path', type=str, default=None)
    parser.add_argument('--launcher_config_sha256', type=str, default=None)
    parser.add_argument('--npc_total', type=int, default=0,
                        help='Optional background NPC count (default: 0)')
    parser.add_argument('--npc_car_ratio', type=float, default=0.50,
                        help='Car ratio among optional NPCs (default: 0.50)')
    parser.add_argument('--npc_cyclist_ratio', type=float, default=0.30,
                        help='Cyclist ratio; the remainder are pedestrians')
    parser.add_argument('--resume', action='store_true', default=False,
                        help='Skip scenarios that already have durable results')
    parser.add_argument(
        '--headless', action='store_true',
        help=('Render Pygame off-screen without creating or focusing a visible '
              'window; video recording remains enabled'))

    args = parser.parse_args()
    args.scenario = args.scenario.lower().replace('.', '')
    # The repository Behavior controller is a non-interactive screening
    # baseline.  Always keep its Pygame visualizer off-screen, including when
    # run.py is invoked directly without the launcher-provided flag.
    if args.model == "behavior":
        args.headless = True
    if args.max_invalid_retries < 0:
        parser.error('--max_invalid_retries must be zero or greater')
    if args.npc_total < 0:
        parser.error('--npc_total must be zero or greater')
    if (args.npc_car_ratio < 0.0 or args.npc_cyclist_ratio < 0.0
            or args.npc_car_ratio + args.npc_cyclist_ratio > 1.0):
        parser.error('NPC ratios must be non-negative and sum to at most 1.0')
    if args.model == 'tcp' and not os.path.isfile(args.model_path):
        raise FileNotFoundError("TCP model checkpoint not found: {}".format(args.model_path))

    # SDL selects its video backend when pygame is initialized.  The dummy
    # backend still provides the display Surface consumed by Visualizer and
    # imageio, but it never creates a desktop window or captures input focus.
    if args.headless:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

    # ======================
    # Load collision enhancement config
    # ======================
    _collision_cfg = load_collision_config(args.collision_config)
    is_enabled = _collision_cfg.get('global', {}).get('enabled', True)
    print(f"[COLLISION] Enhancement {'ENABLED' if is_enabled else 'DISABLED'}")

    VIDEO_DIR = args.video_dir if args.batch_id and args.scenario == "2b" \
        else os.path.join(args.video_dir, args.scenario)
    os.makedirs(VIDEO_DIR, exist_ok=True)
    scenario_files = get_sorted_scenario_files(args.input_dir, args.scenario)
    if not scenario_files:
        print("❌ No scenarios found")
        return

    if args.scenario == '1d':
        signal_configs = []
        for path in scenario_files:
            document = load_scenario_document(path)
            signal_configs.append(document[args.town][args.route_id][0].get("signal_test", {}))
        matrix_summary = validate_signal_matrix(
            signal_configs, require_all_maneuvers=not args.allow_partial_matrix)
        print("[1.d] Matrix validated: {}".format(matrix_summary))
    elif args.scenario == '2b':
        roundabout_configs = []
        for path in scenario_files:
            document = load_scenario_document(path)
            roundabout_configs.append(document[args.town][args.route_id][0])
        if args.screening:
            for config in roundabout_configs:
                validate_roundabout_config(
                    config, require_reproducibility=False)
            print("[2.b] Screening definitions validated: {}".format(
                len(roundabout_configs)))
        else:
            matrix_summary = validate_roundabout_matrix(roundabout_configs)
            print("[2.b] Matrix validated: {}".format(matrix_summary))

    test_records = load_result_records(VIDEO_DIR, args.scenario)
    if args.resume:
        completed_scenarios = _completed_scenario_names(
            test_records, args.scenario)
        original_count = len(scenario_files)
        scenario_files = [
            path for path in scenario_files
            if os.path.basename(path).replace(".json", "")
            not in completed_scenarios
        ]
        print("🔄 Resume: skipped {} completed scenario(s); {} remain".format(
            original_count - len(scenario_files), len(scenario_files)))
        if not scenario_files:
            print("✅ Resume found no pending scenarios")
            return
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")

    pygame.init()
    if args.headless:
        print(
            "[VIDEO] Off-screen Pygame rendering enabled "
            "(driver={}); MP4 recording remains active".format(
                pygame.display.get_driver()),
            flush=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(12.0)
    world = client.get_world()
    original_world_settings = world.get_settings()
    world_settings_restored = [False]

    def restore_world_settings():
        if world_settings_restored[0]:
            return
        try:
            world.apply_settings(original_world_settings)
            world_settings_restored[0] = True
        except Exception as exc:
            print("❌ Failed to restore CARLA world settings: {}".format(exc))

    atexit.register(restore_world_settings)
    world.apply_settings(carla.WorldSettings(synchronous_mode=True, fixed_delta_seconds=0.05))
    if args.scenario in ('1d', '2b'):
        actual_town = world.get_map().name.rsplit('/', 1)[-1]
        if actual_town.lower() != args.town.lower():
            raise RuntimeError(
                "{} map mismatch: CARLA loaded {!r}, but --town is {!r}".format(
                    args.scenario, actual_town, args.town))

    runtime_environment = _runtime_environment(client, world, args)
    runtime_environment["pygame_window_mode"] = (
        "headless" if args.headless else "visible")
    runtime_environment["pygame_video_driver"] = pygame.display.get_driver()
    runtime_environment["ads"] = {
        "name": args.model,
        "implementation": (
            "scene.EgoRouteFollowScene.follow_route"
            if args.model == "behavior" else "model.tcp.TCPAgent.get_action"),
        "checkpoint_path": (
            os.path.abspath(args.model_path)
            if args.model == "tcp" and args.model_path else None),
        "checkpoint_sha256": (
            file_sha256(args.model_path)
            if args.model == "tcp" and args.model_path else None),
    }
    runtime_environment["collision_configuration"] = {
        "path": (os.path.abspath(args.collision_config)
                 if args.collision_config else None),
        "sha256": (file_sha256(args.collision_config)
                   if args.collision_config else None),
        "resolved": _collision_cfg,
    }
    result_metadata = {
        "town": args.town,
        "host": args.host,
        "port": args.port,
        "route_id": args.route_id,
        "model": args.model,
        "model_path": (os.path.abspath(args.model_path)
                       if args.model == "tcp" and args.model_path else None),
        "model_sha256": file_sha256(args.model_path) if args.model == "tcp" else None,
        "carla_client_version": client.get_client_version(),
        "carla_server_version": client.get_server_version(),
        "fixed_delta_seconds": 0.05,
        "latest_run_id": run_id,
        "max_invalid_retries": args.max_invalid_retries
        if args.scenario == "2b" else None,
        "execution_mode": "screening" if args.screening else "formal",
        "batch_id": args.batch_id,
        "evidence_profile": "stf_carla_simulation",
        "telemetry_frequency_hz": 20,
        "video_purpose": "simulation_visualization",
        "pygame_window_mode": "headless" if args.headless else "visible",
        "runtime_environment": runtime_environment,
    }

    pending_attempts = [
        (idx, cfg_path, 1) for idx, cfg_path in enumerate(scenario_files)]
    shutdown_requested = False

    def store_roundabout_attempt(record):
        """Append every attempt; the 2.b aggregator selects the latest valid one."""
        test_records.append(record)
        persist_result_records(
            test_records, VIDEO_DIR, args.scenario, result_metadata)
        # The complete timeline has already been written to this attempt's
        # telemetry.csv.gz.  Retain only the lightweight batch index in memory
        # so large weather/vehicle expansions remain bounded.
        test_records[-1] = roundabout_record_summary(record)

    while pending_attempts and not shutdown_requested:
        idx, cfg_path, attempt_index = pending_attempts.pop(0)
        scenario_name = os.path.basename(cfg_path).replace(".json", "")
        attempt_suffix = "" if args.scenario != "2b" else \
            " | attempt {}/{}".format(attempt_index, args.max_invalid_retries + 1)
        print(
            f"\n======= Running {scenario_name} "
            f"({idx+1}/{len(scenario_files)}){attempt_suffix} =======")
        scenario_document = load_scenario_document(cfg_path)
        config_sha256 = canonical_hash(scenario_document)
        scenario_config = scenario_document[args.town][args.route_id][0]
        test_config_key = "signal_test" if args.scenario == "1d" else "roundabout_test"
        reproducibility = scenario_config.get(test_config_key, {}).get("reproducibility", {})
        generation = scenario_config.get("roundabout_test", {}).get(
            "generation", {})
        trial_seed = reproducibility.get("trial_seed")
        if args.screening and trial_seed is None:
            base_seed = generation.get("base_seed", 41798)
            trial_seed = int(canonical_hash({
                "base_seed": base_seed,
                "config_sha256": config_sha256,
                "mode": "screening",
            })[:8], 16)
        apply_reproducible_seed(trial_seed, client)

        # ======================
        # ✅ 自动加载天气（关键行）
        # ======================
        weather_ok, weather_error = load_weather_from_json(
            world, cfg_path, args.town, args.route_id)

        if args.scenario == "2b":
            if not weather_ok:
                record = _roundabout_invalid_record(
                    scenario_name, run_id, cfg_path, config_sha256, attempt_index,
                    scenario_document, args.town, args.route_id,
                    "weather_setup_failed:{}".format(weather_error))
                attempt_shutdown = False
            else:
                record, attempt_shutdown = _run_roundabout_attempt(
                    client, world, cfg_path, args.town, args.route_id,
                    args.model, args.model_path, VIDEO_DIR, scenario_name,
                    run_id, config_sha256, attempt_index, scenario_document,
                    runtime_environment,
                    "screening" if args.screening else "formal",
                    args.npc_total, args.npc_car_ratio,
                    args.npc_cyclist_ratio)

            if args.screening:
                record["execution_mode"] = "screening"
                record["screening_seed"] = trial_seed
            record["random_seeds"] = {
                "base_seed": reproducibility.get("base_seed", generation.get("base_seed")),
                "matrix_seed": reproducibility.get("matrix_seed"),
                "trial_seed": trial_seed,
                "python_numpy_torch_traffic_manager_seed": trial_seed,
            }
            if "scenario_snapshot" not in record:
                record["scenario_snapshot"] = scenario_config
                record["runtime_environment"] = runtime_environment
                record.update(_roundabout_output_identity(scenario_config))
                record["execution_mode"] = (
                    "screening" if args.screening else "formal")
            _enrich_video_artifact(record)
            store_roundabout_attempt(record)
            verdict = (
                "PASS" if record.get("pass") is True
                else "FAIL" if record.get("pass") is False
                else "INVALID")
            print(
                "✅ {} | {} | attempt={} | collision={}".format(
                    scenario_name, verdict, attempt_index, record.get("collision")))

            shutdown_requested = bool(attempt_shutdown)
            if shutdown_requested:
                pending_attempts.clear()
                continue
            if record.get("trial_valid") is False:
                if attempt_index <= args.max_invalid_retries:
                    pending_attempts.append((idx, cfg_path, attempt_index + 1))
                    print(
                        "[2.b] INVALID，已加入自动重试队列：attempt {}/{}".format(
                            attempt_index + 1, args.max_invalid_retries + 1))
                else:
                    if args.screening:
                        print("[2.b] 筛选记录为 INVALID；按配置不自动补测。")
                    else:
                        print(
                            "[2.b] INVALID，已达到最大重试次数；"
                            "该三次试验矩阵保持 incomplete。")
            continue

        video_path = os.path.join(VIDEO_DIR, f"{scenario_name}.mp4")
        writer = imageio.get_writer(video_path, fps=FPS, codec='libx264')
        scene = None
        npc_actors = []
        try:
            scene_class = SCENE_CLASSES.get(args.scenario)
            if scene_class is None:
                raise ValueError("Unsupported scenario: {}".format(args.scenario))
            scene = scene_class(
                client, world, cfg_path, args.town, args.route_id, args.model, args.model_path)
            scene.spawn()
            npc_actors = spawn_traffic_npcs(
                world, client, args.npc_total,
                args.npc_car_ratio, args.npc_cyclist_ratio)
        except Exception as e:
            print(f"❌ Spawn failed: {e}")
            failed_record = {
                "scenario": scenario_name,
                "run_id": run_id,
                "input_file": os.path.abspath(cfg_path),
                "config_sha256": config_sha256,
                "pass": False,
                "route_complete": False,
                "failure_reasons": ["spawn_failed: {}".format(e)],
            }
            if args.scenario == "2b":
                # A scene/setup failure means the prescribed test conditions
                # were not established; it is not a tested-system verdict.
                failed_record.update({
                    "pass": None,
                    "trial_valid": False,
                    "precondition_valid": False,
                    "invalid_reason": "spawn_failed: {}".format(e),
                })
            try:
                test_config = scenario_document[args.town][args.route_id][0].get(
                    "signal_test" if args.scenario == "1d" else "roundabout_test", {})
                reproducibility = test_config.get("reproducibility", {})
                topology = test_config.get("topology", {})
                failed_record.update({
                    "matrix_id": test_config.get("matrix_id")
                    or reproducibility.get("matrix_id"),
                    "maneuver": test_config.get("maneuver"),
                    "planned_exit": test_config.get("planned_exit"),
                    "trial_index": reproducibility.get("trial_index"),
                    "signal_case": test_config.get("signal_case"),
                    "base_seed": reproducibility.get("base_seed"),
                    "matrix_seed": reproducibility.get("matrix_seed"),
                    "trial_seed": reproducibility.get("trial_seed"),
                    "route_fingerprint": reproducibility.get("route_fingerprint"),
                    "topology_hash": reproducibility.get("topology_hash")
                    or test_config.get("topology_hash")
                    or (topology.get("topology_hash")
                        if isinstance(topology, dict) else None),
                })
            except (KeyError, IndexError, TypeError):
                pass
            test_records.append(failed_record)
            persist_result_records(
                test_records, VIDEO_DIR, args.scenario, result_metadata)
            if scene is not None and getattr(scene, "ego", None):
                scene.destroy()
            _destroy_traffic_npcs(npc_actors)
            writer.close()
            continue

        # ======================
        # 碰撞传感器
        # ======================
        collision_occurred = False
        collision_events = []

        def on_collision(event):
            nonlocal collision_occurred
            collision_occurred = True
            event_time = getattr(event, "timestamp", None)
            if event_time is None:
                event_time = world.get_snapshot().timestamp.elapsed_seconds
            impulse = getattr(event, "normal_impulse", None)
            impulse_magnitude = None
            if impulse is not None:
                impulse_magnitude = float(np.sqrt(
                    impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2))
            other_actor = getattr(event, "other_actor", None)
            collision_events.append({
                "frame": getattr(event, "frame", None),
                "sim_time": round(float(event_time), 4),
                "other_actor_id": getattr(other_actor, "id", None),
                "other_actor_type": getattr(other_actor, "type_id", None),
                "impulse": round(impulse_magnitude, 4)
                if impulse_magnitude is not None else None,
            })
            if hasattr(scene, "record_collision"):
                scene.record_collision(event)

        bp_lib = world.get_blueprint_library()
        collision_bp = bp_lib.find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=scene.ego)
        collision_sensor.listen(on_collision)

        lane_invasion_sensor = None
        lane_invasion_events = []
        if args.scenario == "2b":
            lane_invasion_bp = bp_lib.find('sensor.other.lane_invasion')
            lane_invasion_sensor = world.spawn_actor(
                lane_invasion_bp, carla.Transform(), attach_to=scene.ego)

            def on_lane_invasion(event):
                event_time = getattr(event, "timestamp", None)
                if event_time is None:
                    event_time = world.get_snapshot().timestamp.elapsed_seconds
                markings = []
                for marking in getattr(event, "crossed_lane_markings", []):
                    markings.append({
                        "type": str(getattr(marking, "type", "Unknown")),
                        "color": str(getattr(marking, "color", "Unknown")),
                        "lane_change": str(getattr(marking, "lane_change", "Unknown")),
                    })
                lane_invasion_events.append({
                    "frame": getattr(event, "frame", None),
                    "sim_time": round(float(event_time), 4),
                    "markings": markings,
                })
                if hasattr(scene, "record_lane_invasion"):
                    scene.record_lane_invasion(event)

            lane_invasion_sensor.listen(on_lane_invasion)

        # ======================
        # 距离统计
        # ======================
        previous_location = None
        total_distance = 0.0

        viz = Visualizer(world, scene.ego)
        running = True

        ego_velocity = 0.0
        ego_acc_x = ego_acc_y = ego_acc_z = 0.0
        ego_x = ego_y = ego_z = 0.0
        ego_roll = ego_pitch = ego_yaw = 0.0
        current_game_time = 0.0

        while running:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False

            if not scene.tick():
                running = False

            trans = scene.ego.get_transform()
            vel = scene.ego.get_velocity()
            acc = scene.ego.get_acceleration()

            if previous_location is not None:
                dx = trans.location.x - previous_location.x
                dy = trans.location.y - previous_location.y
                dist = np.sqrt(dx**2 + dy**2)
                total_distance += dist
            previous_location = trans.location

            ego_velocity = np.sqrt(vel.x**2 + vel.y**2)
            ego_acc_x, ego_acc_y, ego_acc_z = acc.x, acc.y, acc.z
            ego_x, ego_y, ego_z = trans.location.x, trans.location.y, trans.location.z
            ego_roll, ego_pitch, ego_yaw = trans.rotation.roll, trans.rotation.pitch, trans.rotation.yaw

            waypoints = scene.get_future_waypoints(12)
            viz.render(waypoints)
            frame = pygame.surfarray.array3d(viz.screen).swapaxes(0, 1)
            writer.append_data(frame)
            world.tick()

        running_status = "no running" if collision_occurred else "running"

        record = {
            "scenario": scenario_name,
            "run_id": run_id,
            "input_file": os.path.abspath(cfg_path),
            "config_sha256": config_sha256,
            "ego_velocity": round(ego_velocity, 2),
            "ego_acceleration_x": round(ego_acc_x, 2),
            "ego_acceleration_y": round(ego_acc_y, 2),
            "ego_acceleration_z": round(ego_acc_z, 2),
            "ego_x": round(ego_x, 2),
            "ego_y": round(ego_y, 2),
            "ego_z": round(ego_z, 2),
            "ego_roll": round(ego_roll, 2),
            "ego_pitch": round(ego_pitch, 2),
            "ego_yaw": round(ego_yaw, 2),
            "current_game_time": round(world.get_snapshot().timestamp.elapsed_seconds, 1),
            "driven_distance": round(total_distance, 2),
            "average_velocity": round(ego_velocity, 2),
            "lane_invasion": 0,
            "off_road": 0,
            "collision": running_status,
            "run_red_light": 0,
            "run_stop": 0,
            "distance_to_route": 0.0,
            "route_complete": False,
            "collision_events": collision_events,
            "lane_invasion_events": lane_invasion_events,
        }
        if hasattr(scene, "get_result"):
            record.update(scene.get_result(collision_occurred=collision_occurred))
        test_records.append(record)
        persist_result_records(
            test_records, VIDEO_DIR, args.scenario, result_metadata)
        verdict = "PASS" if record.get("pass") is True else "FAIL" if record.get("pass") is False else "DONE"
        print(f"✅ {scenario_name} | {verdict} | collision: {record['collision']}")

        collision_sensor.stop()
        collision_sensor.destroy()
        if lane_invasion_sensor is not None:
            lane_invasion_sensor.stop()
            lane_invasion_sensor.destroy()
        _destroy_traffic_npcs(npc_actors)
        writer.close()
        viz.destroy()
        scene.destroy()
        world.tick()
        pygame.time.wait(1000)

    artifacts = persist_result_records(
        test_records, VIDEO_DIR, args.scenario, result_metadata)
    df = pd.DataFrame(test_records)
    print(f"\n🎉 ALL DONE! Markdown report: {artifacts['markdown']}")
    print(f"Structured results: {artifacts['json']}, {artifacts['csv']}, {artifacts['pickle']}")
    print("\nPreview:")
    preview_columns = [column for column in (
        "scenario", "maneuver", "signal_case", "planned_exit",
        "trial_index", "attempt_index", "trial_valid", "pass", "collision")
                       if column in df.columns]
    print(df[preview_columns])
    restore_world_settings()
    if world_settings_restored[0]:
        atexit.unregister(restore_world_settings)
    pygame.quit()

if __name__ == '__main__':
    main()

    # python tools/run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a
    # input_dir 读取json文件，town和scenario决定使用哪个场景类，video_dir决定视频输出目录
    # python tools/run.py --input_dir ./save_scenarios/ --town TOWN10HD_Opt --scenario 3a --model tcp
    # python tools/run.py --input_dir ./save_scenarios/ --town roadside_1 --scenario 3a --model behavior
