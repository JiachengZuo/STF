"""6c CARLA execution. Observer and tested controllers have separate ownership."""
from __future__ import annotations
from contextlib import ExitStack
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
from pathlib import Path
import time

import carla

from emergency_brake_5d_runtime import CameraFrames, crosses_solid
from station_stop_6c import (StationStopEvaluator, angle_delta, digest, distance, frame_coordinates, reference_control, reference_path, seal)


class SUTFailure(RuntimeError):
    pass


def transform(p, lift=0.):
    return carla.Transform(carla.Location(x=p['x'], y=p['y'], z=p['z'] + lift),
                           carla.Rotation(yaw=p['yaw']))


def pose_of(actor):
    t = actor.get_transform()
    return dict(x=t.location.x, y=t.location.y, z=t.location.z, yaw=t.rotation.yaw)


def planar_speed(actor):
    v = actor.get_velocity()
    return math.hypot(v.x, v.y)


def read_vehicle_geometry(actor):
    """Archive the selected actor's actual box, axle spacing and steering limit."""
    bb = actor.bounding_box
    geometry = {group: {axis: getattr(getattr(bb, group), axis) for axis in ('x', 'y', 'z')}
                for group in ('extent', 'location')}
    wheels = actor.get_physics_control().wheels
    if len(wheels) != 4:
        raise ValueError('6c vehicle geometry requires a measured four-wheel vehicle')
    front = tuple((getattr(wheels[0].position, axis) + getattr(wheels[1].position, axis)) / 2
                  for axis in ('x', 'y'))
    rear = tuple((getattr(wheels[2].position, axis) + getattr(wheels[3].position, axis)) / 2
                 for axis in ('x', 'y'))
    wheelbase = math.hypot(front[0] - rear[0], front[1] - rear[1]) / 100.
    steering = [float(w.max_steer_angle) for w in wheels if w.max_steer_angle > 0]
    if not 1.5 < wheelbase < 10 or not steering or not 0 < min(steering) < 90:
        raise ValueError('6c vehicle has invalid measured wheelbase or steering limit')
    geometry.update(wheelbase_m=wheelbase, wheelbase_source='carla_physics_axle_centres',
                    max_steer_angle_deg=min(steering), steering_source='carla_physics_wheels')
    if actor.type_id == 'vehicle.mercedes.sprinter':
        # The former passenger-car mounts sit inside the Sprinter body.
        # Keep the existing camera intrinsics; mount both views ahead of the box.
        pose = dict(x=geometry['location']['x'] + geometry['extent']['x'] + .15,
                    y=0., z=geometry['location']['z'] + .55 * geometry['extent']['z'],
                    yaw=0., roll=0.)
        geometry['camera_mounts'] = dict(video=dict(pose, pitch=-5.), tcp=dict(pose, pitch=0.))
        geometry['camera_mount_source'] = 'sprinter_front_bbox_clearance'
    return geometry


def vehicle_camera_transform(baseline, camera, fallback):
    pose = baseline['vehicle_geometry'].get('camera_mounts', {}).get(camera)
    if pose is None:
        return fallback
    return carla.Transform(carla.Location(x=pose['x'], y=pose['y'], z=pose['z']),
                           carla.Rotation(pitch=pose.get('pitch', 0.),
                                          yaw=pose.get('yaw', 0.), roll=pose.get('roll', 0.)))


class CoordinatePlanner:
    """Raw world coordinates, never map.get_waypoint(goal). No STOP command exists in TCP.

    raw_goal exposes the approach plus goal with LANEFOLLOW and records that
    limitation. hybrid hands control to the named reference module at Pn.
    """
    def __init__(self, route, goal):
        import numpy as np
        self.np = np
        self.points = list(route) + [goal]
        self.cursor = 0

    def run_step(self, location):
        p = dict(x=location.x, y=location.y)
        while self.cursor < len(self.points) - 1 and distance(p, self.points[self.cursor]) < 4:
            self.cursor += 1
        target = self.points[self.cursor]
        return self.np.array([target['x'], target['y']]), 4


class StationStopScene:
    def __init__(self, world, definition, model, checkpoint, tcp_mode):
        self.world, self.d, self.model, self.tcp_mode = world, definition, model, tcp_mode
        self.checkpoint = checkpoint
        self.b, self.eng = definition['baseline'], definition['engineering']
        self.trial = StationStopEvaluator(definition)
        self.actors, self.sensors = [], []
        self.collisions, self.lanes, self.telemetry, self.controls = [], [], [], []
        self.camera, self.video = CameraFrames(), CameraFrames()
        self.ego = self.tcp = None
        self.initialization = None
        self.handoff_frame = self.terminal_frame = None
        self.docking = False
        self.path = reference_path(definition)
        self.operator_interruption = None

    def sensor(self, blueprint, callback, pose=None):
        actor = self.world.spawn_actor(blueprint, pose or carla.Transform(), attach_to=self.ego)
        self.actors.append(actor); self.sensors.append(actor); actor.listen(callback)
        return actor

    def camera_blueprint(self, width, height, fov):
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        for key, value in dict(image_size_x=width, image_size_y=height, fov=fov, sensor_tick=0.).items():
            bp.set_attribute(key, str(value))
        return bp

    def spawn(self, video=True):
        lib = self.world.get_blueprint_library()
        bp = lib.find(self.d['ego_blueprint'])
        bp.set_attribute('role_name', '6c_ego')
        self.ego = self.world.try_spawn_actor(bp, transform(self.d['ego_start'], .35))
        if self.ego is None:
            raise RuntimeError('spawn_failed')
        self.actors.append(self.ego)
        self.ego.set_autopilot(False)
        self.ego.apply_control(carla.VehicleControl(brake=1.))
        actual = self.ego.bounding_box
        for group in ('extent', 'location'):
            for axis in ('x', 'y', 'z'):
                if abs(getattr(getattr(actual, group), axis) - self.b['vehicle_geometry'][group][axis]) > .02:
                    raise RuntimeError('vehicle_geometry_changed')
        if 'max_steer_angle_deg' in self.b['vehicle_geometry']:
            measured_geometry = read_vehicle_geometry(self.ego)
            for key in ('wheelbase_m', 'max_steer_angle_deg'):
                if abs(measured_geometry[key] - self.b['vehicle_geometry'][key]) > .02:
                    raise RuntimeError('vehicle_physics_changed:' + key)
        self.sensor(lib.find('sensor.other.collision'), lambda e: self.collisions.append(
            dict(frame=e.frame, sim_time=e.timestamp, other_id=e.other_actor.id,
                 other_type=e.other_actor.type_id)))
        self.sensor(lib.find('sensor.other.lane_invasion'), lambda e: self.lanes.append(
            dict(frame=e.frame, sim_time=e.timestamp, markings=[str(m.type) for m in e.crossed_lane_markings])))
        if self.model == 'tcp':
            from model.stf_tcp import (TCPAgent, TCP_CAMERA_WIDTH, TCP_CAMERA_HEIGHT, TCP_CAMERA_FOV,
                                   TCP_CAMERA_X, TCP_CAMERA_Y, TCP_CAMERA_Z)
            planner = CoordinatePlanner(self.d['ego_route'], self.b['stop_pose'])
            self.tcp = TCPAgent(self.checkpoint, planner)
            self.tcp.load_model(strict=True)
            self.sensor(self.camera_blueprint(TCP_CAMERA_WIDTH, TCP_CAMERA_HEIGHT, TCP_CAMERA_FOV),
                        self.camera.callback, vehicle_camera_transform(self.b, 'tcp',
                            carla.Transform(carla.Location(x=TCP_CAMERA_X, y=TCP_CAMERA_Y, z=TCP_CAMERA_Z))))
        self.record_video = video
        if video:
            # Match the shared run.py Visualizer's right-hand front camera.
            self.sensor(self.camera_blueprint(256, 256, 90), self.video.callback,
                        vehicle_camera_transform(self.b, 'video',
                            carla.Transform(carla.Location(x=.5, z=1.6), carla.Rotation(pitch=-5))))
        # Parked setup and policy warmup are excluded from scoring.
        for _ in range(30):
            self.world.tick()
        if self.collisions:
            raise RuntimeError('initialization_collision')
        if self.tcp:
            for _ in range(int(self.tcp.config.seq_len)):
                snap = self.world.get_snapshot()
                rgb = self.camera.wait(snap.frame, snap.timestamp.elapsed_seconds, self.eng['camera_timeout_s'])
                self.tcp.get_action(rgb, self.ego)
                if self.world.get_snapshot().frame != snap.frame:
                    raise RuntimeError('world_advanced_during_warmup')
                self.world.tick()
        self.ego.apply_control(carla.VehicleControl())
        self.world.tick()
        before = self.world.get_snapshot()
        v = self.d['initial_speed_kmh'] / 3.6
        forward = self.ego.get_transform().get_forward_vector()
        self.ego.set_target_angular_velocity(carla.Vector3D())
        self.ego.set_target_velocity(carla.Vector3D(x=forward.x * v, y=forward.y * v, z=0.))
        self.world.tick()
        first = self.world.get_snapshot()
        # CARLA's first physics frame after a parked brake can undershoot the
        # assigned speed. Correct it during setup and verify the next frame;
        # no velocity assignment occurs after control handoff.
        measured = planar_speed(self.ego)
        corrected = v + max(0., v - measured) * .2
        self.ego.set_target_velocity(carla.Vector3D(x=forward.x * corrected,
                                                    y=forward.y * corrected, z=0.))
        self.world.tick()
        snap = self.world.get_snapshot()
        actual_pose, actual_speed = pose_of(self.ego), planar_speed(self.ego) * 3.6
        self.initialization = dict(requested_pose=self.d['ego_start'], actual_pose=actual_pose,
                                   requested_speed_kmh=self.d['initial_speed_kmh'], actual_speed_kmh=actual_speed,
                                   first_velocity_frame=before.frame, velocity_set_frame=first.frame,
                                   handoff_frame=snap.frame,
                                   handoff_sim_time=snap.timestamp.elapsed_seconds)
        if (distance(actual_pose, self.d['ego_start']) > self.eng['initial_position_tolerance_m']
            or abs(actual_speed - self.d['initial_speed_kmh']) > self.eng['initial_speed_tolerance_kmh']
            or abs(angle_delta(actual_pose['yaw'], self.d['ego_start']['yaw'])) > self.eng['initial_heading_tolerance_deg']):
            raise RuntimeError('initial_state_not_established')
        if self.collisions:
            raise RuntimeError('initialization_collision')
        # Check sensors at the handoff frame before starting the valid trial.
        for queue, required in ((self.camera, self.tcp is not None), (self.video, video)):
            if required:
                rgb = queue.wait(snap.frame, snap.timestamp.elapsed_seconds, self.eng['camera_timeout_s'])
                with queue.condition:
                    queue.frames[snap.frame] = (snap.timestamp.elapsed_seconds, rgb)
        self.handoff_frame = snap.frame
        self.trial.start(snap.timestamp.elapsed_seconds, snap.frame)

    def relevant(self, events, until=None):
        return [e for e in events if self.handoff_frame is not None and e['frame'] > self.handoff_frame
                and (until is None or e['frame'] <= until)]

    def observation(self):
        pose = pose_of(self.ego)
        vel = self.ego.get_velocity()
        forward = self.ego.get_transform().get_forward_vector()
        speed = math.hypot(vel.x, vel.y)
        signed = vel.x * forward.x + vel.y * forward.y
        tol = self.d['evaluation']['boundary_tolerance_m']
        vertices = self.ego.bounding_box.get_world_vertices(self.ego.get_transform())
        docking_start, _ = frame_coordinates(self.d['ego_route'][-1], self.b['stop_pose'])
        if frame_coordinates(pose, self.b['stop_pose'])[0] < docking_start:
            # Before Pn the route may include a genuine curved connector. Use
            # the Driving corridor at each vertex. After Pn the vehicle may
            # choose its own parking path to the explicitly supplied G.
            from emergency_brake_5d import project
            route = self.d['ego_route']
            route_length = project(route[-1], route)['length']
            outside = False
            for v in vertices:
                p = dict(x=v.x, y=v.y)
                pr = project(p, route, extend_ends=True)
                nearest = min(route, key=lambda q: distance(p, q))
                if (pr['lateral'] > nearest['width'] / 2 + tol
                        or pr['s'] < -self.b['vehicle_geometry']['extent']['x'] - 2
                        or pr['s'] > route_length + self.b['vehicle_geometry']['extent']['x'] + 2):
                    outside = True
                    break
        else:
            # A/B defines the station reference, not a parking-area boundary.
            # The first stop is scored against G; collisions and solid-line
            # crossings remain independent failure conditions.
            outside = False
        rot = self.ego.get_transform().rotation
        outside = outside or abs(rot.roll) > 30 or abs(rot.pitch) > 30
        return pose, speed, signed, outside

    def step(self, snap):
        now, frame = snap.timestamp.elapsed_seconds, snap.frame
        if self.world.get_snapshot().frame != frame:
            raise RuntimeError('world_advanced_before_observation')
        pose, speed, signed, outside = self.observation()
        self.trial.update(now, frame, pose, speed, signed, outside=outside,
                          collision=bool(self.relevant(self.collisions, frame)),
                          solid_line=any(crosses_solid(e) for e in self.relevant(self.lanes, frame)))
        row = dict(frame=frame, sim_time=now, pose=pose, speed_mps=speed, signed_speed_mps=signed,
                   phase=self.trial.phase, outside=outside, **self.trial.errors(pose))
        self.telemetry.append(row)
        self.terminal_frame = frame
        if self.trial.status != 'RUNNING':
            return False
        longitudinal, _ = frame_coordinates(pose, self.b['stop_pose'])
        docking_start, _ = frame_coordinates(self.d['ego_route'][-1], self.b['stop_pose'])
        if self.model == 'tcp' and self.tcp_mode == 'hybrid' and longitudinal >= docking_start:
            if not self.docking:
                self.trial.events.append(dict(event='TCP_TO_REFERENCE_PARKING', frame=frame, sim_time=now))
            self.docking = True
        controller = 'reference_parking' if self.model == 'behavior' or self.docking else 'tcp'
        if controller == 'tcp':
            rgb = self.camera.wait(frame, now, self.eng['camera_timeout_s'])
            try:
                from stf_vehicle_control import decode_vehicle_action
                action = self.tcp.get_action(rgb, self.ego)
                throttle, turn, brake = decode_vehicle_action(action, preserve_throttle=True)
            except Exception as exc:
                raise SUTFailure('tcp_control_failure:' + str(exc)) from exc
        else:
            try:
                throttle, turn, brake = reference_control(pose, speed, self.d, self.path)
            except Exception as exc:
                raise SUTFailure('reference_control_failure:' + str(exc)) from exc
        if self.world.get_snapshot().frame != frame:
            raise RuntimeError('world_advanced_during_control')
        control = carla.VehicleControl(throttle=float(throttle), steer=float(turn), brake=float(brake))
        self.ego.apply_control(control)
        self.controls.append(dict(observation_frame=frame, applies_to_frame=frame + 1, sim_time=now,
                                  source=controller, throttle=throttle, steer=turn, brake=brake))
        return True

    def finish(self):
        # CARLA's sparse collision sensor has no per-frame completion callback.
        # Exact camera-frame waits plus a bounded drain are recorded honestly;
        # this is a screening implementation, not a strict sensor barrier proof.
        time.sleep(self.eng['evidence_drain_s'])
        for sensor in self.sensors:
            try:
                sensor.stop()
            except RuntimeError:
                pass
        if self.handoff_frame is not None and any(e['frame'] <= self.handoff_frame for e in self.collisions):
            self.trial.invalidate_initialization('late_initialization_collision')
        elif self.terminal_frame is not None:
            if self.relevant(self.collisions, self.terminal_frame):
                self.trial.end('FAILED', 'collision')
            if any(crosses_solid(e) for e in self.relevant(self.lanes, self.terminal_frame)):
                self.trial.end('FAILED', 'crossed_solid_line')
        if self.trial.status == 'RUNNING':
            self.trial.end('INVALID', 'trial_interrupted')
        result = self.trial.result()
        result.update(initialization=self.initialization, telemetry_samples=self.telemetry,
                      control_samples=self.controls, collision_events=self.collisions, lane_events=self.lanes,
                      evidence=dict(terminal_frame=self.terminal_frame, collision_barrier='bounded_callback_drain',
                                    drain_wall_time_s=self.eng['evidence_drain_s']),
                      task_interface=dict(goal_coordinates_preserved=True,
                                          tcp_stop_command_supported=False,
                                          tcp_mode=self.tcp_mode if self.model == 'tcp' else None))
        return result

    def cleanup(self):
        errors = []
        for sensor in self.sensors:
            try:
                if sensor.is_listening:
                    sensor.stop()
            except RuntimeError as exc:
                errors.append(str(exc))
        for actor in reversed(self.actors):
            try:
                if actor.is_alive:
                    actor.destroy()
            except RuntimeError as exc:
                errors.append(str(exc))
        if self.actors:
            try:
                self.world.tick()  # Process deferred destruction before checking actor IDs.
            except RuntimeError as exc:
                errors.append(str(exc))
        remaining = []
        for actor in self.actors:
            try:
                # CARLA 0.9.16 can return a stale handle from get_actor(id)
                # after destroy() succeeds; the handle's is_alive is current.
                if actor.is_alive:
                    remaining.append(actor.id)
            except RuntimeError as exc:
                errors.append(str(exc))
        return dict(remaining_actor_ids=remaining, errors=errors)


def finalize_video_evidence(scene, record, path, frames, enabled, close_errors=()):
    """A scored trial with incomplete required video cannot be a recorded PASS."""
    from station_stop_6c_cli import file_hash
    errors = list(close_errors)
    try:
        present = path.is_file() and path.stat().st_size > 0
    except OSError as exc:
        present = False
        errors.append('video_file:' + str(exc))
    if enabled and scene.handoff_frame is not None:
        if not present or frames <= 0:
            errors.append('video_missing')
        if frames != len(scene.telemetry):
            errors.append('video_frame_coverage_incomplete')
    checksum = None
    if enabled and present and frames > 0 and not errors:
        try:
            checksum = file_hash(path)
        except OSError as exc:
            errors.append('video_checksum:' + str(exc))
    if errors:
        scene.trial.end('INVALID', 'video_evidence_incomplete')
        record.update(scene.trial.result())
    record.update(video_path=str(path.resolve()) if present and frames > 0 else None,
                  video_frame_count=frames, video_sha256=checksum,
                  video_evidence_errors=errors)
    return errors


def run_connected(args, cfg, definitions, manifest):
    from station_stop_6c_cli import (ROOT, artifact_paths, batch_outcome, file_hash,
                                     latest_records, persist, recover_records, write_json)
    from station_stop_6c_video import StationStopVideo, VIDEO_METADATA, VIDEO_PROFILE
    if not args.no_video and any(abs(d['engineering']['fixed_delta_seconds'] - .05) > 1e-9 for d in definitions):
        raise ValueError('6c standard 20 fps video requires fixed_delta_seconds = 0.05')
    mode = args.tcp_mode or cfg['tcp_mode']
    controller = ('behavior_reference_parking' if args.model == 'behavior' else
                  'tcp_plus_reference_parking' if mode == 'hybrid' else 'tcp_raw_goal_experimental')
    checkpoint = str(Path(args.model_path or cfg['model_path']).resolve())
    if args.model == 'tcp' and not Path(checkpoint).is_file():
        raise ValueError('TCP checkpoint not found: ' + checkpoint)
    selected = [d['condition_fingerprint'] for d in definitions]
    signature = dict(controller=controller, manifest=manifest['manifest_fingerprint'],
                     video_enabled=not args.no_video,
                     video_profile=VIDEO_PROFILE if not args.no_video else None,
                     artifact_layout='flat_scenario_weather_named_videos_v1',
                     checkpoint=file_hash(checkpoint) if args.model == 'tcp' else None,
                     runtime={p: file_hash(ROOT / p) for p in ('station_stop_6c.py', 'station_stop_6c_runtime.py',
                                                             'station_stop_6c_cli.py', 'station_stop_6c_video.py',
                                                             'emergency_brake_5d_video.py', 'render.py',
                                                             'model/stf_tcp.py', 'stf_vehicle_control.py',
                                                             'emergency_brake_5d_runtime.py', 'emergency_brake_5d.py',
                                                             'curve_emergency_5e.py', 'agent/base_policy.py', 'TCP/TCP/model.py',
                                                             'TCP/TCP/config.py', 'TCP/TCP/resnet.py')})
    if args.resume and not args.output_dir:
        raise ValueError('--resume requires --output_dir pointing to the original run')
    out = Path(args.output_dir or Path(cfg['output_root']) / controller /
               datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    if out.exists() and not args.resume:
        raise ValueError('output exists; use a new directory or --resume')
    out.mkdir(parents=True, exist_ok=True)
    host, port = args.host or cfg['host'], args.port or cfg['port']
    with ExitStack() as stack:
        for path in (Path('/tmp') / ('stf_6c_' + digest([host, port])[:16] + '.lock'), out / 'run.lock'):
            lock = stack.enter_context(path.open('w'))
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.resume:
            old = json.loads((out / 'run_metadata.json').read_text())
            if old['signature'] != signature:
                raise ValueError('resume differs in controller, manifest, checkpoint or implementation')
            if not set(old['selected']).issubset(selected):
                raise ValueError('resume selection must include all conditions already selected in this batch')
        records = json.loads((out / 'results.json').read_text()) if (out / 'results.json').exists() else []
        records = recover_records(out, records, definitions, controller, require_video=not args.no_video)
        write_json(out / 'run_metadata.json', dict(signature=signature, selected=selected,
                   controller=controller, tcp_mode=mode if args.model == 'tcp' else None,
                   screening_only=True, automatic_failed_retries=False))
        write_json(out / 'artifact_layout.json', dict(
            layout='flat_scenario_weather_named_videos_v1',
            video_filename='<scenario>_attempt_<N>.mp4',
            details_directory='details', video_format=VIDEO_METADATA,
            video_enabled=not args.no_video))
        persist(out, records, len(definitions))
        completed = {r['scenario'] for r in latest_records(records).values()
                     if r['result_status'] in ('PASS', 'FAILED')}
        if all(d['scenario'] in completed for d in definitions):
            write_json(out / 'batch_status.json', dict(status=batch_outcome(records, len(definitions)),
                       restoration_errors=[], results_path=str((out / 'results.json').resolve())))
            return 0
        client = carla.Client(host, port); client.set_timeout(20.)
        world = client.get_world()
        if world.get_settings().synchronous_mode:
            raise RuntimeError('CARLA already in synchronous mode; another runner may own the clock')
        baseline = definitions[0]['baseline']
        requested = args.town or cfg.get('town') or baseline['map_name']
        from curve_emergency_5e import require_map
        require_map(world.get_map().name, requested)
        if hashlib.sha256(world.get_map().to_opendrive().encode()).hexdigest() != baseline['opendrive_sha256']:
            raise ValueError('CARLA map differs from baseline; no actors spawned')
        if list(world.get_actors().filter('vehicle.*')) or list(world.get_actors().filter('walker.pedestrian.*')):
            raise RuntimeError('world contains traffic actors; use an idle test world')
        if client.get_client_version() != client.get_server_version():
            raise RuntimeError('CARLA client/server version mismatch')
        lib = world.get_blueprint_library()
        lib.find(baseline['ego_blueprint'])
        original_settings, original_weather = world.get_settings(), world.get_weather()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = definitions[0]['engineering']['fixed_delta_seconds']
        settings.no_rendering_mode = False
        stopped = False
        batch_completed = False
        batch_error = None
        write_json(out / 'batch_status.json', dict(status='running', selected_conditions=len(definitions),
                   results_path=str((out / 'results.json').resolve())))
        try:
            world.apply_settings(settings)
            print('6c controller: ' + controller, flush=True)
            print('OUTPUT ' + str(out), flush=True)
            for index, d in enumerate(definitions, 1):
                if d['scenario'] in completed:
                    continue
                attempt_started_monotonic = time.monotonic()
                attempt_started_utc = datetime.now(timezone.utc).isoformat()
                existing_attempts = [int(p.name.rsplit('_attempt_', 1)[1].split('.')[0])
                                     for p in (out / 'details').glob(d['scenario'] + '_attempt_*.json')]
                attempt = 1 + max([r['attempt'] for r in records if r['scenario'] == d['scenario']] + existing_attempts + [0])
                paths = artifact_paths(out, d, attempt)
                if any(p.exists() for p in paths.values()):
                    raise ValueError('attempt artifacts already exist: ' + d['scenario'])
                write_json(paths['condition'], d)
                print('START {} [{}/{}]'.format(d['scenario'], index, len(definitions)), flush=True)
                scene = StationStopScene(world, d, args.model, checkpoint, mode)
                writer = viz = None
                video_close_errors = []
                frames = 0
                video_path = paths['video']
                record = None
                try:
                    weather = carla.WeatherParameters()
                    for key, value in d['weather'].items():
                        if not hasattr(weather, key):
                            raise RuntimeError('weather_attribute_unsupported:' + key)
                        setattr(weather, key, value)
                    world.set_weather(weather)
                    scene.spawn(video=not args.no_video)
                    if not args.no_video:
                        import imageio.v2 as imageio
                        viz = StationStopVideo(world, scene.ego, headless=args.headless)
                        writer = imageio.get_writer(str(video_path), fps=1. / settings.fixed_delta_seconds,
                                                    codec='libx264', pixelformat='yuv420p')
                    while True:
                        snap = world.get_snapshot()
                        running = scene.step(snap)
                        if writer:
                            rgb = scene.video.wait(snap.frame, snap.timestamp.elapsed_seconds, scene.eng['camera_timeout_s'])
                            writer.append_data(viz.frame(rgb, scene.path)); frames += 1
                        if not running:
                            break
                        world.tick()
                except SUTFailure as exc:
                    scene.trial.end('FAILED', str(exc))
                except KeyboardInterrupt:
                    scene.trial.end('INVALID', 'operator_interrupted'); stopped = True
                except Exception as exc:
                    # Camera/bridge/fixture failures are not policy failures.
                    scene.trial.end('INVALID', type(exc).__name__ + ':' + str(exc))
                finally:
                    try:
                        record = scene.finish()
                    except Exception as exc:
                        scene.trial.end('INVALID', 'evidence_finalization_failed:' + str(exc))
                        record = scene.trial.result()
                        record['initialization'] = scene.initialization
                    finally:
                        try:
                            cleanup = scene.cleanup()
                        except Exception as exc:
                            cleanup = dict(remaining_actor_ids=[a.id for a in scene.actors], errors=[str(exc)])
                        if writer:
                            try:
                                writer.close()
                            except Exception as exc:
                                video_close_errors.append('video_close:' + str(exc))
                        if viz:
                            try:
                                viz.close()
                            except Exception as exc:
                                cleanup['errors'].append('visualizer_close:' + str(exc))
                video_errors = finalize_video_evidence(scene, record, video_path, frames,
                                                       not args.no_video, video_close_errors)
                record.update(scenario=d['scenario'], profile_id=d['profile_id'], weather_id=d['weather_id'],
                              attempt=attempt, controller=controller, condition_fingerprint=d['condition_fingerprint'],
                              cleanup=cleanup, video_format=dict(VIDEO_METADATA) if not args.no_video else None,
                              started_at_utc=attempt_started_utc,
                              finished_at_utc=datetime.now(timezone.utc).isoformat(),
                              wall_time_s=time.monotonic() - attempt_started_monotonic)
                record = seal(record, 'result_fingerprint')
                detail = paths['result']
                write_json(detail, record)
                summary = {k: v for k, v in record.items() if k not in ('telemetry_samples', 'control_samples')}
                summary['detail_path'] = str(detail.resolve())
                records.append(summary)
                persist(out, records, len(definitions))
                print('{}: {} {}'.format(d['scenario'], record['result_status'], ','.join(record['reasons'])), flush=True)
                # Cleanup problems are operational faults; retain the observed verdict.
                if (stopped or cleanup['remaining_actor_ids'] or cleanup['errors'] or video_errors
                    or record['result_status'] == 'INVALID' and not args.continue_invalid):
                    stopped = True; break
            else:
                batch_completed = True
        except KeyboardInterrupt:
            stopped = True
            batch_error = 'operator_interrupted'
        except Exception as exc:
            stopped = True
            batch_error = type(exc).__name__ + ':' + str(exc)
            raise
        finally:
            restoration_errors = []
            for restore, value in ((world.set_weather, original_weather), (world.apply_settings, original_settings)):
                try:
                    restore(value)
                except Exception as exc:
                    restoration_errors.append(str(exc))
            if restoration_errors:
                stopped = True
            write_json(out / 'batch_status.json', dict(
                status=batch_outcome(records, len(definitions), stopped or not batch_completed),
                error=batch_error, restoration_errors=restoration_errors,
                results_path=str((out / 'results.json').resolve())))
        return 2 if stopped else 0
