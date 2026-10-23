"""CARLA adapter for 2d. No collision enhancer, GA or legacy crossing scene."""
from __future__ import annotations

import copy
import hashlib
import math
import time
import xml.etree.ElementTree as ET

import carla

from cross_right_2d import (CrossRightTrial, ENTRY_SPEEDS, angle_delta, at, make_baseline, project, route_intersection, stations,
                            reference_cruise_control)
from cross_right_2d_routes import extend_target_upstream, route_samples, trace_waypoints
from emergency_brake_5d_runtime import CameraFrames, crosses_solid, point, speed, steer


def transform(p, lift=0.):
    return carla.Transform(carla.Location(x=p['x'], y=p['y'], z=p['z']+lift),
                           carla.Rotation(yaw=p['yaw']))


def require_map(actual, requested):
    if requested and actual != requested and actual.rstrip('/').split('/')[-1] != requested:
        raise ValueError('map mismatch: current={}, requested={}; 2d uses the loaded map'.format(actual, requested))


def road_speed(roads, wp):
    """Read declared OpenDRIVE road/lane speeds without inventing a default."""
    road = roads.get(str(wp.road_id))
    if road is None:
        return None
    declared = None
    types = [x for x in road.findall('type') if float(x.get('s', 0)) <= wp.s]
    if types:
        declared = max(types, key=lambda x: float(x.get('s', 0))).find('speed')
    sections = [x for x in road.findall('./lanes/laneSection') if float(x.get('s', 0)) <= wp.s]
    if sections:
        section = max(sections, key=lambda x: float(x.get('s', 0)))
        lane = next((x for x in section.findall('./*/lane') if x.get('id') == str(wp.lane_id)), None)
        if lane is not None:
            speeds = [x for x in lane.findall('speed') if float(x.get('sOffset', 0))+float(section.get('s', 0)) <= wp.s]
            if speeds:
                declared = max(speeds, key=lambda x: float(x.get('sOffset', 0)))
    if declared is None:
        return None
    try:
        value = float(declared.get('max'))
    except (TypeError, ValueError):
        return None
    unit = declared.get('unit', 'm/s')
    if unit not in ('m/s', 'km/h', 'mph'):
        return None
    return value*{'m/s': 3.6, 'km/h': 1., 'mph': 1.609344}[unit]


def capture_baseline(world, ego_anchors, target_anchors, cfg):
    """Save the drawn routes; optional map metadata never blocks editing."""
    map_obj = world.get_map()
    waypoints = []
    for label, anchors in (('自车', ego_anchors), ('目标车', target_anchors)):
        try:
            waypoints.append(trace_waypoints(map_obj, anchors))
        except ValueError as exc:
            raise ValueError(label+'路线：'+str(exc)) from exc
    routes = [route_samples(ws) for ws in waypoints]
    ec, tc, _ = route_intersection(*routes)
    waiting_distance = cfg['target_speed_kmh']/3.6*cfg['ttc_window_s'][1]
    if tc < waiting_distance:
        waypoints[1] = extend_target_upstream(waypoints[1], waiting_distance-tc)
        routes[1] = route_samples(waypoints[1])
        ec, tc, _ = route_intersection(*routes)
    odr = map_obj.to_opendrive()
    try:
        roads = {r.get('id'): r for r in ET.fromstring(odr).findall('road')}
    except ET.ParseError:
        roads = {}
    for route, ws in zip(routes, waypoints):
        for p, w in zip(route, ws):
            p['signal_hit'] = None
            limit = road_speed(roads, w)
            if limit is not None:
                p['speed_limit_kmh'] = limit
    headings, junction_ids = [], []
    for route, ws, center in zip(routes, waypoints, (ec, tc)):
        pose = at(route, center)
        wp = min(ws, key=lambda w: math.hypot(w.transform.location.x-pose['x'], w.transform.location.y-pose['y']))
        try:
            junction = wp.get_junction()
            if junction is not None and junction.id not in junction_ids:
                junction_ids.append(junction.id)
                headings.extend(a.transform.rotation.yaw for a, _ in junction.get_waypoints(carla.LaneType.Driving))
        except RuntimeError:
            pass
    if headings:
        routes[0][0]['junction_entry_headings'] = headings
    # Traffic lights are recorded, not deleted or treated as a drawing error.
    try:
        lights = list(world.get_actors().filter('*traffic*light*'))
    except RuntimeError:
        lights = None
    if lights is not None:
        for route in routes:
            for p in route:
                p['signal_hit'] = False
        eng = cfg['engineering']
        for light in lights:
            location = light.get_location()
            for route, center in zip(routes, (ec, tc)):
                p = project(dict(x=location.x, y=location.y), route)
                if p['lateral'] <= eng['signal_lateral_m'] and abs(p['s']-center) <= eng['signal_longitudinal_window_m']:
                    route[0]['signal_hit'] = True
    return make_baseline(*routes, cfg, map_name=map_obj.name,
                         opendrive_sha256=hashlib.sha256(odr.encode()).hexdigest(),
                         ego_anchors=ego_anchors, target_anchors=target_anchors)


def validate_world(world, definition, cfg, requested_map=None):
    b = definition['baseline']
    require_map(world.get_map().name, requested_map)
    require_map(world.get_map().name, b['map_name'])
    if list(world.get_actors().filter('vehicle.*')) or list(world.get_actors().filter('walker.pedestrian.*')):
        raise RuntimeError('world_has_traffic')
    current = capture_baseline(world, b['ego_anchors'], b['target_anchors'], cfg)
    if current['opendrive_sha256'] != b['opendrive_sha256']:
        raise ValueError('baseline_geometry_mismatch: OpenDRIVE changed')
    for key in ('ego_route', 'target_route'):
        if len(current[key]) != len(b[key]) or any(
                math.dist([a[k] for k in ('x', 'y', 'z')], [c[k] for k in ('x', 'y', 'z')]) > .01
                for a, c in zip(current[key], b[key])):
            raise ValueError('baseline_geometry_mismatch: '+key)
    available = {bp.id for bp in world.get_blueprint_library()}
    if not {definition['ego_blueprint'], definition['target_blueprint']} <= available:
        raise ValueError('spawn_failed: vehicle blueprint missing')


class CrossRightScene:
    def __init__(self, world, definition, model='behavior', model_path=None):
        self.world, self.d = world, definition
        self.model, self.model_path = model, model_path
        self.b, self.eng = definition['baseline'], definition['engineering']
        self.route, self.target_route = self.b['ego_route'], self.b['target_route']
        self.trial = CrossRightTrial(definition, model)
        self.actors, self.sensors = [], []
        self.collisions, self.target_collisions, self.lane_events = [], [], []
        self.telemetry, self.controls = [], []
        self.camera, self.video = CameraFrames(), CameraFrames()
        self.ego = self.target = self.tcp = self.behavior = None
        self.previous_frame = None
        self.target_started = self.constant_velocity = False
        self.initialization = {}
        self.initialization_samples = []
        self.initialization_plan = None
        self.ego_velocity_assist = False
        self.trial_start_frame = None
        self.cruise_integral = 0.
        self.reference_brake_accel = self.eng['reference_brake_decel_mps2']
        self.drive_physics = None
        self.wrong_way_checked = False

    def spawn_actor(self, blueprint, pose):
        bp = self.world.get_blueprint_library().find(blueprint)
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'false')
        actor = self.world.try_spawn_actor(bp, pose)
        if actor is None:
            raise RuntimeError('spawn_failed: '+blueprint)
        self.actors.append(actor)
        actor.set_autopilot(False)
        actor.apply_control(carla.VehicleControl(brake=1.))
        return actor

    def sensor(self, bp, callback, pose=None, attach_to=None):
        actor = self.world.spawn_actor(bp, pose or carla.Transform(), attach_to=attach_to or self.ego)
        self.actors.append(actor)
        self.sensors.append(actor)
        actor.listen(callback)
        return actor

    def corners(self, actor):
        # CARLA returns all eight transformed box vertices; take the bottom face.
        vertices = sorted(actor.bounding_box.get_world_vertices(actor.get_transform()), key=lambda v: v.z)[:4]
        return [dict(x=v.x, y=v.y, z=v.z) for v in vertices]

    def prepare_approach(self):
        """Use one common entry location for all speeds, then add a run-in lane."""
        saved = self.d['baseline']
        eng = self.eng
        vmax = max(ENTRY_SPEEDS)/3.6
        confirm = eng['speed_hold_confirm_s']
        entry_distance = max(saved['geometry']['ego_conflict_s'],
                             vmax*(self.d['ttc_window_s'][1]+confirm))
        approach = entry_distance-saved['geometry']['ego_conflict_s']
        preparation = vmax*3*confirm
        required = approach+preparation
        first = saved['ego_route'][0]
        wp = self.world.get_map().get_waypoint(carla.Location(**{k: first[k] for k in ('x', 'y', 'z')}))
        ws = extend_target_upstream([wp], required)
        prefix = route_samples(ws)[:-1]
        route = prefix+copy.deepcopy(saved['ego_route'])
        added = stations(route)[len(prefix)]
        # The lane helper measures 3D distance; the TTC geometry uses planar arc.
        # Extend any remaining planar shortfall on the actual predecessor lane.
        while added+1e-6 < required:
            more = extend_target_upstream(ws, required-added)
            if len(more) == len(ws):
                raise RuntimeError('initialization_approach_unavailable')
            ws = more
            prefix = route_samples(ws)[:-1]
            route = prefix+copy.deepcopy(saved['ego_route'])
            added = stations(route)[len(prefix)]
        self.b = copy.deepcopy(saved)
        self.b['ego_route'] = route
        for key in ('ego_conflict_s', 'ego_conflict_near_s', 'ego_conflict_far_s',
                    'exit_gate_s', 'ego_length_m'):
            self.b['geometry'][key] += added
        self.b['geometry']['timeout_s'] = max(saved['geometry']['timeout_s'],
            (self.b['geometry']['ego_length_m']-preparation)/(min(ENTRY_SPEEDS)/3.6)+eng['timeout_margin_s'])
        self.route = route
        self.trial = CrossRightTrial(self.d, self.model, runtime_baseline=self.b)
        self.initialization_plan = dict(approach_added_m=approach, preparation_added_m=added-approach,
            drawn_start_s=added, handoff_gate_s=added-approach, entry_distance_m=entry_distance,
            handoff_gate_pose=at(route, added-approach))

    def cruise_feedforward(self, measured_mps):
        """Balance motor drag and grade using this vehicle's physical parameters."""
        physics = self.drive_physics
        if physics is None:
            return 0.
        radius = sum(w.radius for w in physics.wheels)/len(physics.wheels)*.01
        gear = max(1, self.ego.get_control().gear)
        ratio = physics.final_ratio*physics.forward_gears[min(gear, len(physics.forward_gears))-1].ratio
        omega = measured_mps/radius*ratio
        rpm = omega*30/math.pi
        curve = sorted((p.x, p.y) for p in physics.torque_curve)
        torque = curve[0][1] if rpm <= curve[0][0] else curve[-1][1]
        for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
            if x0 <= rpm <= x1:
                torque = y0+(y1-y0)*(rpm-x0)/(x1-x0)
                break
        gravity = -9.81*self.ego.get_transform().get_forward_vector().z
        damping = physics.damping_rate_zero_throttle_clutch_engaged
        load = damping*omega-gravity*physics.mass*radius/ratio
        if load >= 0:
            available = torque+(damping-physics.damping_rate_full_throttle)*omega
            return load/available
        brake = -load*ratio/(physics.mass*radius*self.reference_brake_accel)
        return -brake*self.eng['speed_hold_throttle_gain']/self.eng['speed_hold_brake_gain']

    def handoff(self):
        """Prepare a measured moving start, then release all speed assistance."""
        if self.initialization.get('verified'):
            raise RuntimeError('initialization_already_completed')
        required = self.d['required_speed_kmh']
        tolerance = self.eng['speed_hold_tolerance_kmh']
        confirm = self.eng['speed_hold_confirm_s']
        # At the speed cap, use the lower half of the permitted initial band so
        # a downhill physics substep does not create a setup overspeed.
        desired_kmh = min(required, self.d['section_speed_limit_kmh']-tolerance/2)
        self.initialization = dict(mode='verified_moving_start', verified=False,
            required_speed_kmh=required, requested_speed_kmh=required,
            preparation_setpoint_kmh=desired_kmh, plan=self.initialization_plan,
            speed_assistance_after_handoff=False)
        # Set an actual forward gear before rolling. A velocity-only start
        # can otherwise leave PhysX in neutral until the first throttle pulse.
        self.ego.apply_control(carla.VehicleControl(gear=1, manual_gear_shift=True))
        self.world.tick()
        self.ego.enable_constant_velocity(carla.Vector3D(x=desired_kmh/3.6))
        self.ego_velocity_assist = True
        begin = self.world.get_snapshot().timestamp.elapsed_seconds
        unlocked_at = stable_since = None
        previous = None
        try:
            while True:
                snap = self.world.get_snapshot()
                frame, now = snap.frame, snap.timestamp.elapsed_seconds
                if previous is not None and frame != previous+1:
                    raise RuntimeError('initialization_frame_discontinuity')
                previous = frame
                observed = speed(self.ego)*3.6
                pr = project(point(self.ego), self.route, extend_ends=True)
                if self.collisions or self.target_collisions:
                    raise RuntimeError('initialization_collision')
                if abs(observed-required) <= tolerance+1e-9 and observed <= self.d['section_speed_limit_kmh']:
                    if stable_since is None:
                        stable_since = now
                else:
                    stable_since = None
                held = 0. if stable_since is None else now-stable_since
                ready = (pr['s'] >= self.initialization_plan['handoff_gate_s']
                         and held+1e-9 >= confirm)
                row = dict(frame=frame, sim_time_s=now, ego=point(self.ego),
                    observed_speed_kmh=observed, ego_s=pr['s'], lateral_m=pr['lateral'],
                    held_s=held, constant_velocity_active=self.ego_velocity_assist,
                    phase='handoff' if ready else 'preparation')
                self.initialization_samples.append(row)
                if ready:
                    ttc = (self.b['geometry']['ego_conflict_s']-pr['s'])/(observed/3.6)
                    if ttc <= self.d['ttc_window_s'][1]:
                        raise RuntimeError('initialization_missed_ttc_window')
                    self.ego.disable_constant_velocity()
                    self.ego_velocity_assist = False
                    unlocked_at = now
                    row['constant_velocity_active'] = False
                    self.trial_start_frame = frame
                    drive = self.ego.get_control()
                    self.initialization.update(verified=True, frame=frame, sim_time_s=now,
                        gear=drive.gear, manual_gear_shift=drive.manual_gear_shift,
                        actual_speed_kmh=observed, held_s=held, ttc_s=ttc,
                        pose=point(self.ego), preparation_sim_time_s=now-begin,
                        constant_velocity_disabled_at_sim_time_s=unlocked_at)
                    # This exact measured snapshot is the first formal sample.
                    # No tick or velocity assignment occurs across the handoff.
                    return
                remaining = self.b['geometry']['ego_conflict_s']-pr['s']
                if remaining <= observed/3.6*self.d['ttc_window_s'][1]:
                    raise RuntimeError('initialization_speed_not_reached_before_window')
                if now-begin >= self.b['geometry']['timeout_s']:
                    raise RuntimeError('initialization_speed_not_reached')
                turn = steer(self.ego, self.route)
                throttle, brake, self.cruise_integral = reference_cruise_control(
                    desired_kmh/3.6, observed/3.6, self.eng, self.cruise_integral,
                    self.cruise_feedforward(observed/3.6))
                control = carla.VehicleControl(throttle=throttle, brake=brake, steer=turn)
                row['control'] = dict(throttle=control.throttle, brake=control.brake, steer=control.steer)
                self.ego.apply_control(control)
                self.world.tick()
        finally:
            if self.ego_velocity_assist:
                self.ego.disable_constant_velocity()
                self.ego_velocity_assist = False

    def spawn(self, video=True):
        self.prepare_approach()
        if self.b['geometry']['target_wait_s'] < -1e-6:
            raise RuntimeError('目标车等待位置超出可跟踪车道，地图上游道路不足')
        self.ego = self.spawn_actor(self.d['ego_blueprint'], transform(self.route[0], self.eng['spawn_lift_m']))
        physics = self.ego.get_physics_control()
        self.drive_physics = physics
        self.reference_brake_accel = sum(w.max_brake_torque/(w.radius*.01) for w in physics.wheels)/physics.mass
        self.target = self.spawn_actor(self.d['target_blueprint'],
                                       transform(self.b['geometry']['target_wait_pose'], self.eng['spawn_lift_m']))
        lib = self.world.get_blueprint_library()
        def collision_log(destination):
            return lambda e: destination.append(dict(frame=e.frame, sim_time=e.timestamp,
                other_id=e.other_actor.id, other_type=e.other_actor.type_id))
        self.sensor(lib.find('sensor.other.collision'), collision_log(self.collisions))
        self.sensor(lib.find('sensor.other.collision'), collision_log(self.target_collisions), attach_to=self.target)
        self.sensor(lib.find('sensor.other.lane_invasion'), lambda e: self.lane_events.append(
            dict(frame=e.frame, sim_time=e.timestamp, markings=[str(m.type) for m in e.crossed_lane_markings])))
        if self.model == 'tcp':
            from model.stf_tcp import (TCPAgent, TCPRoutePlanner, TCP_CAMERA_WIDTH, TCP_CAMERA_HEIGHT,
                                   TCP_CAMERA_FOV, TCP_CAMERA_X, TCP_CAMERA_Y, TCP_CAMERA_Z)
            planner = TCPRoutePlanner()
            ws = [self.world.get_map().get_waypoint(carla.Location(x=p['x'], y=p['y'], z=p['z'])) for p in self.route]
            if any(w is None for w in ws):
                raise RuntimeError('TCP route contains missing waypoints')
            planner.set_route(ws, maneuver='straight')
            self.tcp = TCPAgent(self.model_path, planner)
            self.tcp.load_model(strict=True)
            bp = lib.find('sensor.camera.rgb')
            for k, v in dict(image_size_x=TCP_CAMERA_WIDTH, image_size_y=TCP_CAMERA_HEIGHT,
                             fov=TCP_CAMERA_FOV, sensor_tick=0).items():
                bp.set_attribute(k, str(v))
            self.sensor(bp, self.camera.callback,
                        carla.Transform(carla.Location(x=TCP_CAMERA_X, y=TCP_CAMERA_Y, z=TCP_CAMERA_Z)))
        if video:
            bp = lib.find('sensor.camera.rgb')
            for k, v in dict(image_size_x=256, image_size_y=256, fov=90, sensor_tick=0).items():
                bp.set_attribute(k, str(v))
            self.sensor(bp, self.video.callback, carla.Transform(carla.Location(x=.5, z=1.6), carla.Rotation(pitch=-5)))
        for _ in range(20):
            self.world.tick()
        if self.collisions or self.target_collisions:
            raise RuntimeError('initialization_collision')
        if self.model == 'tcp':
            for _ in range(int(self.tcp.config.seq_len)):
                snap = self.world.get_snapshot()
                rgb = self.camera.wait(snap.frame, snap.timestamp.elapsed_seconds, self.eng['camera_timeout_s'])
                self.tcp.get_action(rgb, self.ego)
                if self.world.get_snapshot().frame != snap.frame:
                    raise RuntimeError('world_frame_discontinuity')
                self.world.tick()
        self.handoff()
        if self.model == 'behavior':
            from cross_right_2d_behavior import BehaviorController
            self.behavior = BehaviorController(self.ego, self.route, self.d['required_speed_kmh'],
                                               self.d['section_speed_limit_kmh'], self.eng['fixed_delta_seconds'])

    def target_step(self):
        if self.trial.released and not self.target_started:
            self.target.apply_control(carla.VehicleControl(steer=steer(self.target, self.target_route)))
            self.target.enable_constant_velocity(carla.Vector3D(x=20/3.6, y=0., z=0.))
            self.target_started = self.constant_velocity = True
        elif self.target_started and self.trial.t_clear is None:
            self.target.apply_control(carla.VehicleControl(steer=steer(self.target, self.target_route)))
        if self.trial.t_clear is not None:
            self.stop_target()

    def stop_target(self):
        if self.target is not None:
            if self.constant_velocity:
                self.target.disable_constant_velocity()
                self.constant_velocity = False
            self.target.apply_control(carla.VehicleControl(brake=1.))

    def event_failures(self, frame=None):
        cutoff = self.trial.gate_frame if self.trial.gate_frame is not None else frame
        events = lambda items: [e for e in list(items)
            if (self.trial_start_frame is None or e['frame'] >= self.trial_start_frame)
            and (cutoff is None or e['frame'] <= cutoff)]
        collision = (any(e['other_id'] == self.target.id for e in events(self.collisions)) or
                     any(e['other_id'] == self.ego.id for e in events(self.target_collisions)))
        reasons = []
        if any(e['other_id'] != self.target.id for e in events(self.collisions)):
            reasons.append('infrastructure_collision')
        if any(crosses_solid(e) for e in events(self.lane_events)):
            reasons.append('solid_line')
        return collision, reasons

    def target_collision_frame(self):
        cutoff = self.trial.gate_frame
        frames = [e['frame'] for events, other in ((self.collisions, self.target),
                                                   (self.target_collisions, self.ego))
                  for e in events if other is not None and e['other_id'] == other.id
                  and (self.trial_start_frame is None or e['frame'] >= self.trial_start_frame)
                  and (cutoff is None or e['frame'] <= cutoff)]
        return min(frames) if frames else None

    def observation(self, frame):
        corners = self.corners(self.ego)
        collision, reasons = self.event_failures(frame)
        if not self.trial.gate_crossed:
            try:
                checked = []
                for p in corners:
                    wp = self.world.get_map().get_waypoint(carla.Location(**p), project_to_road=False,
                                                          lane_type=carla.LaneType.Driving)
                    if wp is None:
                        continue
                    saved = min(self.route, key=lambda q: math.hypot(p['x']-q['x'], p['y']-q['y']))
                    checked.append(wp.road_id == saved['road_id'] and wp.lane_id*saved['lane_id'] < 0)
                self.wrong_way_checked = self.wrong_way_checked or len(checked) == len(corners)
                if any(checked):
                    reasons.append('wrong_way_lane')
            except RuntimeError:
                pass
        actual_control = self.ego.get_control()
        return dict(frame=frame, ego=point(self.ego), target=point(self.target), ego_corners=corners,
                    ego_gear=actual_control.gear, ego_manual_gear_shift=actual_control.manual_gear_shift,
                    target_corners=self.corners(self.target), ego_speed_mps=speed(self.ego),
                    target_speed_mps=speed(self.target), ego_yaw=self.ego.get_transform().rotation.yaw,
                    collision_target=collision, collision_target_frame=self.target_collision_frame(), failure_reasons=reasons)

    def ego_control(self, frame, now, obs):
        if self.model == 'tcp':
            import torch
            from stf_vehicle_control import decode_vehicle_action
            rgb = self.camera.wait(frame, now, self.eng['camera_timeout_s'])
            with torch.inference_mode():
                action = self.tcp.get_action(rgb, self.ego)
            throttle, turn, brake = decode_vehicle_action(action, preserve_throttle=True)
            return carla.VehicleControl(throttle=throttle, steer=turn, brake=brake)
        return self.behavior.run_step()

    def step(self):
        snap = self.world.get_snapshot()
        frame, now = snap.frame, snap.timestamp.elapsed_seconds
        if self.previous_frame is not None and frame != self.previous_frame+1:
            raise RuntimeError('world_frame_discontinuity')
        if not self.initialization.get('verified'):
            raise RuntimeError('ego_initial_speed_not_prepared')
        obs = self.observation(frame)
        transition = self.trial.update(now, obs)
        self.target_step()
        self.telemetry.append(dict(obs, sim_time_s=now, ttc_s=self.trial.previous_sample_ttc,
                                   release_transition=transition, target_started=self.target_started))
        if self.trial.done:
            self.ego.apply_control(carla.VehicleControl(brake=1.))
            return False
        control = self.ego_control(frame, now, obs)
        if self.world.get_snapshot().frame != frame:
            raise RuntimeError('world_frame_discontinuity')
        self.ego.apply_control(control)
        self.controls.append(dict(frame=frame, sim_time_s=now, throttle=control.throttle,
                                  steer=control.steer, brake=control.brake,
                                  **{self.model: dict(self.tcp.last_debug if self.model == 'tcp' else self.behavior.last_debug)}))
        self.previous_frame = frame
        return True

    def finish(self):
        self.stop_target()
        if self.ego is not None:
            self.ego.apply_control(carla.VehicleControl(brake=1.))
        time.sleep(.1)  # Drain callbacks for the final simulation frame.
        for sensor in self.sensors:
            if sensor.is_listening:
                sensor.stop()
        if self.trial_start_frame is not None and self.ego is not None and self.target is not None:
            self.trial.add_events(*self.event_failures(), collision_frame=self.target_collision_frame())
        if not self.trial.done:
            self.trial.latch('trial_interrupted', immediate=True)
        result = self.trial.result()
        result.update(initialization=self.initialization, initialization_samples=self.initialization_samples,
                      runtime_ego_route=self.route, runtime_geometry=self.b['geometry'],
                      telemetry_samples=self.telemetry,
                      control_samples=self.controls, collision_events=self.collisions,
                      target_collision_events=self.target_collisions, lane_events=self.lane_events,
                      controller_fallback_used=False, controller_reads_scenario_state=False,
                      controller_uses_simulator_actor_state=self.model == 'behavior',
                      controller_implementation='carla_behavior_agent' if self.model == 'behavior' else 'tcp',
                      behavior_configuration=self.behavior.metadata if self.behavior is not None else None,
                      section_checks=dict(lights='not_checked',
                          solid_line='event_observed' if any(self.trial_start_frame is not None and e['frame'] >= self.trial_start_frame for e in self.lane_events) else 'no_crossing_event_observed',
                          wrong_way='checked' if self.wrong_way_checked else 'not_checked',
                          infrastructure='collision_events_only'))
        return result

    def destroy(self):
        if self.ego_velocity_assist and self.ego is not None:
            try:
                self.ego.disable_constant_velocity()
            except RuntimeError:
                pass
            self.ego_velocity_assist = False
        try:
            self.stop_target()
        except RuntimeError:
            pass
        for sensor in self.sensors:
            try:
                if sensor.is_listening:
                    sensor.stop()
            except RuntimeError:
                pass
        for actor in reversed(self.actors):
            try:
                actor.destroy()
            except RuntimeError:
                pass
        self.actors.clear()
        self.sensors.clear()
        self.ego = self.target = None
