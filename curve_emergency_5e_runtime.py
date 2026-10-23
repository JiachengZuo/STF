"""CARLA adapter for the fixed-route 5e experiment; no collision enhancer or GA."""
from __future__ import annotations

import math
import time
import carla

from curve_emergency_5e import CurveTrial, TCP_STATIONARY_TIMEOUT_S, project, geometry_metrics, xy_distance
from emergency_brake_5d_runtime import CameraFrames, speed, point, steer


def transform(p, yaw=None, lift=0.):
    return carla.Transform(carla.Location(x=p['x'], y=p['y'], z=p['z']+lift),
                           carla.Rotation(yaw=p.get('yaw', 0.) if yaw is None else yaw))


class CurveScene:
    def __init__(self, world, definition, model, model_path):
        self.world, self.d, self.model, self.model_path = world, definition, model, model_path
        self.b = definition['baseline']
        self.eng, self.risk = definition['engineering'], definition['risk']
        self.route = self.b['ego_route']
        self.geometry = geometry_metrics(self.b, self.eng)
        self.crossing = [self.b['crossing_start'], self.b['crossing_end']]
        self.trial = CurveTrial(definition, stationary_timeout_s=
                                TCP_STATIONARY_TIMEOUT_S if model == 'tcp' else None)
        self.ego = self.target = self.tcp = None
        self.actors, self.sensors, self.collisions = [], [], []
        self.target_collisions = []
        self.telemetry, self.controls = [], []
        self.camera, self.video = CameraFrames(), CameraFrames()
        self.target_started = self.target_finished = self.constant_velocity = False
        self.target_start_time = None
        self.previous_frame = None
        self.initialization = {}

    def spawn_actor(self, blueprint, pose):
        bp = self.world.get_blueprint_library().find(blueprint)
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'false')
        actor = self.world.try_spawn_actor(bp, pose)
        if actor is None:
            raise RuntimeError('spawn_failed:' + blueprint)
        self.actors.append(actor)
        return actor

    def sensor(self, bp, callback, pose=None, attach_to=None):
        actor = self.world.spawn_actor(bp, pose or carla.Transform(), attach_to=attach_to or self.ego)
        self.actors.append(actor)
        self.sensors.append(actor)
        actor.listen(callback)
        return actor

    def vertices(self, actor, pose=None):
        return [dict(x=v.x, y=v.y, z=v.z)
                for v in actor.bounding_box.get_world_vertices(pose or actor.get_transform())]

    def crossing_extents(self, pose=None):
        values = [project(v, self.crossing, extend_ends=True)['s'] for v in self.vertices(self.target, pose)]
        return min(values), max(values)

    def conflict_limits(self):
        p = self.b['risk_position']
        width = min(self.route, key=lambda r: xy_distance(r, p))['width']
        half = (width/2+self.eng['target_clearance_m'])/self.geometry['crossing_sine']
        center = self.geometry['crossing_conflict_s']
        return center-half, center+half

    def target_is_clear(self):
        return self.target_started and self.crossing_extents()[0] > self.conflict_limits()[1]

    def target_at_end(self):
        if not self.target_started:
            return False
        pr = project(point(self.target), self.crossing, extend_ends=True)
        return (pr['s'] >= self.geometry['crossing_length_m']
                and pr['lateral'] <= self.eng['crossing_tolerance_m'])

    def check_crossing_space(self, start_pose, end_pose):
        low, high = self.conflict_limits()
        if self.crossing_extents(start_pose)[1] >= low:
            raise RuntimeError('crossing_start_too_close: move start farther from P for full target body')
        if self.crossing_extents(end_pose)[0] <= high:
            raise RuntimeError('crossing_end_too_close: leave room for the truck tail to clear')

    def spawn(self, video=True):
        self.ego = self.spawn_actor(self.d['ego_blueprint'], transform(self.b['ego_start'], lift=.35))
        self.ego.set_autopilot(False)
        self.ego.apply_control(carla.VehicleControl(brake=1.))
        if self.risk['kind'] == 'static':
            p = self.b['risk_position']
            yaw = p['yaw']+self.risk['yaw_offset_deg']
            self.target = self.spawn_actor(self.risk['blueprint'], transform(p, yaw, .5))
            bb = self.target.bounding_box
            if min(bb.extent.x, bb.extent.y, bb.extent.z) <= 0:
                raise RuntimeError('static_asset_has_empty_bounding_box')
            self.target.set_simulate_physics(False)
            self.target.set_transform(transform(p, yaw, bb.extent.z-bb.location.z+.02))
        else:
            self.target = self.spawn_actor(self.risk['blueprint'],
                                          transform(self.crossing[0], self.geometry['crossing_yaw'], .35))
            if self.risk['kind'] == 'vehicle':
                self.target.set_autopilot(False)
                self.target.apply_control(carla.VehicleControl(brake=1.))
            else:
                self.target.apply_control(carla.WalkerControl())
            # Actor transforms are snapshot-cached and remain zero until the
            # next tick. Validate the requested pose, then the settled pose.
            start_pose = transform(self.crossing[0], self.geometry['crossing_yaw'], .35)
            end_pose = transform(self.crossing[-1], self.geometry['crossing_yaw'], .35)
            self.check_crossing_space(start_pose, end_pose)
        lib = self.world.get_blueprint_library()
        self.sensor(lib.find('sensor.other.collision'), lambda e: self.collisions.append(
            dict(frame=e.frame, sim_time=e.timestamp, other_type=e.other_actor.type_id)))
        if self.risk['kind'] != 'static':
            self.sensor(lib.find('sensor.other.collision'), lambda e: self.target_collisions.append(
                dict(frame=e.frame,sim_time=e.timestamp,other_id=e.other_actor.id,
                     other_type=e.other_actor.type_id)),attach_to=self.target)
        if self.model == 'tcp':
            from model.stf_tcp import (TCPAgent, TCPRoutePlanner, TCP_CAMERA_WIDTH, TCP_CAMERA_HEIGHT,
                                   TCP_CAMERA_FOV, TCP_CAMERA_X, TCP_CAMERA_Y, TCP_CAMERA_Z)
            self.planner = TCPRoutePlanner()
            waypoints = [self.world.get_map().get_waypoint(carla.Location(**{k: p[k] for k in ('x','y','z')}))
                         for p in self.route]
            if any(w is None for w in waypoints):
                raise RuntimeError('TCP route contains missing waypoints')
            self.planner.set_route(waypoints)  # LANEFOLLOW on the drawn continuous bend.
            self.tcp = TCPAgent(self.model_path, self.planner)
            self.tcp.load_model(strict=True)
            bp = lib.find('sensor.camera.rgb')
            for k, v in dict(image_size_x=TCP_CAMERA_WIDTH, image_size_y=TCP_CAMERA_HEIGHT,
                             fov=TCP_CAMERA_FOV, sensor_tick=0).items():
                bp.set_attribute(k, str(v))
            self.sensor(bp, self.camera.callback,
                        carla.Transform(carla.Location(x=TCP_CAMERA_X,y=TCP_CAMERA_Y,z=TCP_CAMERA_Z)))
        if video:
            bp = lib.find('sensor.camera.rgb')
            for k, v in dict(image_size_x=256, image_size_y=256, fov=90, sensor_tick=0).items():
                bp.set_attribute(k, str(v))
            self.sensor(bp, self.video.callback, carla.Transform(carla.Location(x=.5,z=1.6),carla.Rotation(pitch=-5)))
        for _ in range(20):
            self.world.tick()
        if self.risk['kind'] != 'static':
            self.check_crossing_space(self.target.get_transform(), end_pose)
        if self.collisions or self.target_collisions:
            raise RuntimeError('initialization_collision: adjust the saved placements')
        if self.model == 'tcp':
            for _ in range(int(self.tcp.config.seq_len)):
                snap = self.world.get_snapshot()
                rgb = self.camera.wait(snap.frame,snap.timestamp.elapsed_seconds,self.eng['camera_timeout_s'])
                self.tcp.get_action(rgb,self.ego)  # Warm up while parked, before trial handoff.
                if self.world.get_snapshot().frame != snap.frame:
                    raise RuntimeError('world_advanced_during_warmup')
                self.world.tick()
        self.ego.apply_control(carla.VehicleControl())
        self.world.tick()
        forward = self.ego.get_transform().get_forward_vector()
        v = self.d['ego_speed_kmh']/3.6
        self.ego.set_target_velocity(carla.Vector3D(x=forward.x*v,y=forward.y*v,z=forward.z*v))
        self.world.tick()
        self.initialization = dict(mode='moving', requested_speed_kmh=self.d['ego_speed_kmh'],
                                   actual_speed_kmh=speed(self.ego)*3.6)

    def target_step(self, now):
        if self.risk['kind'] == 'static':
            return None
        if any(e['other_id'] != self.ego.id for e in self.target_collisions):
            return 'target_collision_with_environment'
        if self.trial.triggered is None or now < self.trial.triggered+self.risk['start_delay_s']:
            return None
        if not self.target_started:
            self.target_started = True
            self.target_start_time = now
            v = self.risk['speed_mps']
            if self.risk['kind'] == 'vehicle':
                self.target.apply_control(carla.VehicleControl())
                self.target.enable_constant_velocity(carla.Vector3D(x=v))
                self.constant_velocity = True
            else:
                yaw = math.radians(self.geometry['crossing_yaw'])
                self.target.apply_control(carla.WalkerControl(
                    direction=carla.Vector3D(x=math.cos(yaw),y=math.sin(yaw)),speed=v))
        pr = project(point(self.target), self.crossing, extend_ends=True)
        if self.target_at_end() and not self.target_finished:
            self.stop_target()
            self.target_finished = True
        if not self.target_finished:
            if pr['lateral'] > self.eng['crossing_tolerance_m']:
                return 'target_left_fixed_crossing_path'
            if (now-self.target_start_time > self.eng['target_speed_grace_s']
                    and abs(speed(self.target)-self.risk['speed_mps']) > self.eng['target_speed_tolerance_mps']):
                return 'target_speed_not_established'
        return None

    def stop_target(self):
        if self.target is None or self.risk['kind'] == 'static':
            return
        if self.risk['kind'] == 'vehicle':
            if self.constant_velocity:
                self.target.disable_constant_velocity()
                self.constant_velocity = False
            self.target.apply_control(carla.VehicleControl(brake=1.))
        else:
            self.target.apply_control(carla.WalkerControl())

    def observation(self):
        ep = project(point(self.ego), self.route)
        width = min(self.route, key=lambda p: xy_distance(p, point(self.ego)))['width']
        vertices = self.vertices(self.ego)
        outside = any(project(v,self.route,extend_ends=True)['lateral'] > width/2+self.eng['boundary_tolerance_m']
                      for v in vertices)
        rotation = self.ego.get_transform().rotation
        outside = outside or abs(rotation.roll)>55 or abs(rotation.pitch)>55
        front = max(project(v,self.route,extend_ends=True)['s'] for v in vertices)
        if self.risk['kind'] == 'static':
            target_front = min(project(v,self.route,extend_ends=True)['s'] for v in self.vertices(self.target))
        else:
            target_front = self.geometry['risk_s']
        # Repeated/overlapping route segments can project to an earlier s.
        # Reaching the drawn endpoint must still terminate immediately.
        at_end = xy_distance(point(self.ego), self.route[-1]) <= self.eng['end_tolerance_m']
        collision = bool(self.collisions) or any(e['other_id']==self.ego.id for e in self.target_collisions)
        return dict(ego_s=ep['s'],ego_speed=speed(self.ego),gap=target_front-front,
                    at_end=at_end,collision=collision,outside=outside,
                    target_finished=self.target_at_end() if self.risk['kind']!='static' else False,
                    target_clear=self.target_is_clear() if self.risk['kind']!='static' else False)

    def ego_control(self, frame, now, obs):
        if self.model == 'tcp':
            import torch
            from stf_vehicle_control import decode_vehicle_action
            rgb = self.camera.wait(frame,now,self.eng['camera_timeout_s'])
            with torch.inference_mode():
                action = self.tcp.get_action(rgb,self.ego)
            # TCPAgent has already applied its own brake/throttle rule. Keep
            # its final three channels intact after finite/range validation.
            throttle, turn, brake = decode_vehicle_action(action, preserve_throttle=True)
            return carla.VehicleControl(throttle=throttle,steer=turn,brake=brake)
        # Explicit ground-truth route/yield reference, not CARLA BehaviorAgent
        # and never a fallback for TCP. Keep a stopping margin ahead of P.
        desired = self.d['ego_speed_kmh']/3.6
        blocked = (self.risk['kind']=='static' or
                   (self.trial.triggered is not None and not obs['target_clear']))
        if blocked and obs['gap'] > -2:
            desired = min(desired, math.sqrt(max(0.,2*2.5*(obs['gap']-2.))))
        error = desired-obs['ego_speed']
        return carla.VehicleControl(steer=steer(self.ego,self.route),
                                    throttle=max(0.,min(.6,.35*error)),brake=max(0.,min(1.,-.5*error)))

    def step(self):
        snap = self.world.get_snapshot()
        frame, now = snap.frame, snap.timestamp.elapsed_seconds
        if self.previous_frame is not None and frame != self.previous_frame+1:
            raise RuntimeError('world_frame_discontinuity')
        obs = self.observation()
        self.trial.update(now, **obs)
        if self.trial.status == 'RUNNING':
            error = self.target_step(now)
            if error:
                self.trial.end('INVALID',error)
        self.telemetry.append(dict(frame=frame,sim_time=now,ego=point(self.ego),target=point(self.target),
                                   target_speed_mps=speed(self.target),target_started=self.target_started,
                                   ego_stationary_duration_s=self.trial.stationary_duration_s,
                                   **obs))
        if self.trial.status != 'RUNNING':
            return False
        control = self.ego_control(frame,now,obs)
        if self.world.get_snapshot().frame != frame:
            raise RuntimeError('world_advanced_during_control')
        self.ego.apply_control(control)
        self.controls.append(dict(frame=frame,throttle=control.throttle,steer=control.steer,brake=control.brake))
        self.previous_frame = frame
        return True

    def finish(self):
        self.stop_target()
        if self.ego is not None:
            self.ego.apply_control(carla.VehicleControl(brake=1.))
        time.sleep(.1)  # Drain collision callbacks without advancing simulation.
        for sensor in self.sensors:
            if sensor.is_listening:
                sensor.stop()
        ego_collision = bool(self.collisions) or any(
            e['other_id'] == self.ego.id for e in self.target_collisions) if self.ego is not None else False
        if ego_collision and self.initialization:
            self.trial.end('FAILED','collision')
        elif self.target_collisions and self.initialization:
            self.trial.end('INVALID','target_collision_with_environment')
        if self.trial.status == 'RUNNING':
            self.trial.end('INVALID','trial_interrupted')
        result = self.trial.result()
        result.update(initialization=self.initialization,telemetry_samples=self.telemetry,
                      control_samples=self.controls,collision_events=self.collisions,
                      target_collision_events=self.target_collisions,
                      controller_fallback_used=False,reference_uses_ground_truth=self.model=='behavior',
                      target_start_sim_time=self.target_start_time,screening_only=True,
                      physical_field_test_compliance_claimed=False)
        return result

    def destroy(self):
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
