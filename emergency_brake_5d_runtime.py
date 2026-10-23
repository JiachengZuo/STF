"""Isolated 5d actor lifecycle and complete three-channel ADS control."""
from __future__ import annotations
import math
import threading
import time
import numpy as np
import carla
from emergency_brake_5d import (BrakeTrial,project,validate_geometry,route_feasibility,
                                RouteLengthError,moving_center_gap,route_point_at)


class CameraFrames:
    def __init__(self):self.frames={};self.condition=threading.Condition()
    def callback(self,image):
        rgb=np.frombuffer(image.raw_data,dtype=np.uint8).reshape(image.height,image.width,4)[:,:,:3][:,:,::-1].copy()
        with self.condition:
            self.frames[image.frame]=(image.timestamp,rgb)
            while len(self.frames)>16:self.frames.pop(min(self.frames))
            self.condition.notify_all()
    def wait(self,frame,timestamp,timeout):
        deadline=time.monotonic()+timeout
        with self.condition:
            while frame not in self.frames:
                remaining=deadline-time.monotonic()
                if remaining<=0:raise RuntimeError('camera_frame_timeout:'+str(frame))
                self.condition.wait(remaining)
            t,rgb=self.frames.pop(frame)
            for old in list(self.frames):
                if old<frame:self.frames.pop(old)
        if not math.isfinite(t) or abs(t-timestamp)>.001:raise RuntimeError('camera_timestamp_mismatch')
        return rgb


def speed(actor):
    v=actor.get_velocity();return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)


def point(actor):
    p=actor.get_location();return {'x':p.x,'y':p.y,'z':p.z}


def crosses_solid(event):
    """CARLA may emit lane events with NONE at OpenDRIVE seams."""
    solid_types={'solid','solidsolid','solidbroken','brokensolid'}
    return any(str(marking).rsplit('.',1)[-1].replace('_','').replace(' ','').lower() in solid_types
               for marking in event.get('markings',[]))


def steer(actor,route):
    p=point(actor);proj=project(p,route);look=proj['s']+max(3.,speed(actor)*.5)
    accum=0.;target=route[-1]
    for a,b in zip(route,route[1:]):
        accum+=math.hypot(b['x']-a['x'],b['y']-a['y'])
        if accum>=look:target=b;break
    angle=math.degrees(math.atan2(target['y']-p['y'],target['x']-p['x']))
    error=(angle-actor.get_transform().rotation.yaw+180)%360-180
    return max(-1.,min(1.,error/45.))


class BrakeScene:
    def __init__(self,world,definition,model,model_path):
        self.world=world;self.d=definition;self.eng=definition['engineering'];self.model=model
        self.route,self.center_gap=validate_geometry(world.get_map(),definition['baseline'],self.eng)
        self.route_plan=route_feasibility(definition['test_speed_kmh'],self.center_gap,
                                         project(self.route[-1],self.route)['length'],self.eng)
        if not self.route_plan['route_length_sufficient']:
            raise RouteLengthError('available %.1f m, required %.1f m for %.1f km/h'%(
                self.route_plan['available_route_length_m'],self.route_plan['required_route_length_m'],definition['test_speed_kmh']))
        # Raw TCP is the tested controller. Failure to establish the standard
        # stable-following precondition is therefore a tested-system failure,
        # not an infrastructure-invalid attempt. The Behavior reference keeps
        # the older setup-screening semantics.
        self.trial=BrakeTrial(definition['test_speed_kmh'],self.eng,
                              precondition_failure_is_failure=(model=='tcp'))
        self.actors=[];self.sensors=[];self.collisions=[];self.lanes=[];self.telemetry=[];self.controls=[]
        self.camera=CameraFrames();self.video=CameraFrames();self.last_control_frame=None
        self.ego=None;self.lead=None;self.tcp=None;self.min_gap=math.inf;self.video_sensor=None
        self.brake_command=self.eng['lead_brake_feedforward'];self.previous_time=None
        self.initialization={};self.warmup_frames=[]
        self.lead_physics={}
        self.lead_constant_velocity=False
        self.ego_speed_integral=0.
        if model=='tcp':
            from model.stf_tcp import TCPAgent,TCPRoutePlanner
            self.planner=TCPRoutePlanner()
            self.planner.set_route([world.get_map().get_waypoint(carla.Location(x=p['x'],y=p['y'],z=p['z']),project_to_road=True) for p in self.route],maneuver='straight')
            self.tcp=TCPAgent(model_path,self.planner);self.tcp.load_model(strict=True)
    def _spawn_car(self,config):
        bp=self.world.get_blueprint_library().find('vehicle.tesla.model3')
        actor=self.world.try_spawn_actor(bp,carla.Transform(carla.Location(x=config['x'],y=config['y'],z=config['z']),carla.Rotation(yaw=config['yaw'])))
        if actor is None:raise RuntimeError('vehicle_spawn_failed')
        self.actors.append(actor);actor.set_autopilot(False);actor.apply_control(carla.VehicleControl(brake=1.))
        return actor
    def _sensor(self,blueprint,actor,callback,transform=None):
        sensor=self.world.spawn_actor(blueprint,transform or carla.Transform(),attach_to=actor)
        self.actors.append(sensor);self.sensors.append(sensor);sensor.listen(callback);return sensor
    def _configure_lead_brakes(self):
        """Give only the target vehicle measured braking authority before motion."""
        physics=self.lead.get_physics_control()
        wheels=physics.wheels
        if not wheels:raise RuntimeError('lead_vehicle_has_no_wheels')
        original=[float(w.max_brake_torque) for w in wheels]
        requested=float(self.eng['lead_max_brake_torque_nm'])
        for wheel in wheels:wheel.max_brake_torque=requested
        # CARLA exposes a copied wheel list; assign it back before applying.
        physics.wheels=wheels
        self.lead.apply_physics_control(physics)
        # ApplyVehiclePhysicsControl is asynchronous. Both cars are still
        # parked under brake; consume it before verifying the server's state.
        self.world.tick()
        applied=self.lead.get_physics_control()
        actual=[float(w.max_brake_torque) for w in applied.wheels]
        if len(actual)!=len(original) or any(not math.isfinite(v) or abs(v-requested)>.001 for v in actual):
            raise RuntimeError('lead_brake_physics_not_applied: requested {}, actual {}'.format(requested,actual))
        self.lead_physics={'blueprint':self.lead.type_id,'mass_kg':float(applied.mass),
            'original_max_brake_torque_nm':original,'requested_max_brake_torque_nm':requested,
            'applied_max_brake_torque_nm':actual,
            'wheel_radius_cm':[float(w.radius) for w in applied.wheels],
            'tire_friction':[float(w.tire_friction) for w in applied.wheels]}
    def spawn(self,video=True):
        self.ego=self._spawn_car(self.d['baseline']['ego_start']);self.lead=self._spawn_car(self.d['baseline']['lead_start'])
        self._configure_lead_brakes()
        length=self.ego.bounding_box.extent.x+self.lead.bounding_box.extent.x
        gap=moving_center_gap(self.d['test_speed_kmh'],self.center_gap,self.eng,length)
        if length>self.eng['vehicle_length_allowance_m']:
            raise RouteLengthError('actual vehicle length exceeds configured planning allowance')
        lead_position=route_point_at(self.route,gap)
        self.lead.set_transform(carla.Transform(carla.Location(x=lead_position['x'],y=lead_position['y'],z=lead_position['z']+.3),carla.Rotation(yaw=lead_position['yaw'])))
        lib=self.world.get_blueprint_library()
        for actor,label in ((self.ego,'ego'),(self.lead,'lead')):
            def collision(event,label=label):self.collisions.append({'actor':label,'frame':event.frame,'sim_time':event.timestamp,'other_type':event.other_actor.type_id})
            def lane(event,label=label):self.lanes.append({'actor':label,'frame':event.frame,'sim_time':event.timestamp,'markings':[str(m.type) for m in event.crossed_lane_markings]})
            self._sensor(lib.find('sensor.other.collision'),actor,collision)
            self._sensor(lib.find('sensor.other.lane_invasion'),actor,lane)
        if self.model=='tcp':
            from model.stf_tcp import TCP_CAMERA_WIDTH,TCP_CAMERA_HEIGHT,TCP_CAMERA_FOV,TCP_CAMERA_X,TCP_CAMERA_Y,TCP_CAMERA_Z
            bp=lib.find('sensor.camera.rgb')
            for key,value in {'image_size_x':TCP_CAMERA_WIDTH,'image_size_y':TCP_CAMERA_HEIGHT,'fov':TCP_CAMERA_FOV,'sensor_tick':0}.items():bp.set_attribute(key,str(value))
            self._sensor(bp,self.ego,self.camera.callback,carla.Transform(carla.Location(x=TCP_CAMERA_X,y=TCP_CAMERA_Y,z=TCP_CAMERA_Z)))
        if video:
            bp=lib.find('sensor.camera.rgb')
            for k,v in {'image_size_x':256,'image_size_y':256,'fov':90,'sensor_tick':0}.items():bp.set_attribute(k,str(v))
            self.video_sensor=self._sensor(bp,self.ego,self.video.callback,carla.Transform(carla.Location(x=0.5,z=1.6),carla.Rotation(pitch=-5)))
        for _ in range(10):
            self.world.tick()
            snap=self.world.get_snapshot();t=snap.timestamp.elapsed_seconds
            try:
                if self.model=='tcp':
                    rgb=self.camera.wait(snap.frame,t,self.eng['camera_timeout_s'])
                    with self.camera.condition:self.camera.frames[snap.frame]=(t,rgb)
                if video:
                    rgb=self.video.wait(snap.frame,t,self.eng['camera_timeout_s'])
                    with self.video.condition:self.video.frames[snap.frame]=(t,rgb)
                break
            except RuntimeError:pass
        else:raise RuntimeError('camera_startup_failed')
        for _ in range(25):
            # Allow the spawned suspension to settle while both vehicles are held.
            self.world.tick()
        self._initialize_moving_start()

    def _initialize_moving_start(self):
        if self.initialization:raise RuntimeError('moving initialization may only run once')
        # Consume TCP's initial zero-action calls at rest, only within this scene.
        # Do not modify TCPAgent, its counters, or the shared inference code.
        if self.model=='tcp':
            for _ in range(int(self.tcp.config.seq_len)):
                snap=self.world.get_snapshot()
                rgb=self.camera.wait(snap.frame,snap.timestamp.elapsed_seconds,self.eng['camera_timeout_s'])
                self.tcp.get_action(rgb,self.ego) # Ignored during parked initialization.
                self.warmup_frames.append(snap.frame)
                if self.world.get_snapshot().frame!=snap.frame:raise RuntimeError('world_advanced_during_warmup')
                self.world.tick()
        # Flush the parked brake command before assigning a moving state.
        for actor in (self.ego,self.lead):
            actor.apply_control(carla.VehicleControl())
        self.world.tick()
        before=self.world.get_snapshot()
        v=self.d['test_speed_kmh']/3.6
        for actor in (self.ego,self.lead):
            forward=actor.get_transform().get_forward_vector()
            actor.apply_control(carla.VehicleControl()) # Release the startup brake.
            actor.set_target_angular_velocity(carla.Vector3D())
            actor.set_target_velocity(carla.Vector3D(x=forward.x*v,y=forward.y*v,z=forward.z*v))
        self.lead.enable_constant_velocity(carla.Vector3D(x=v))
        self.lead_constant_velocity=True
        self.world.tick() # Obtain images/actual state after the velocity assignment.
        after=self.world.get_snapshot()
        self.initialization={'mode':'moving','velocity_set_frame':before.frame,
            'handoff_frame':after.frame,'handoff_sim_time':after.timestamp.elapsed_seconds,
            'requested_speed_kmh':self.d['test_speed_kmh'],'ego_speed_kmh':speed(self.ego)*3.6,
            'lead_speed_kmh':speed(self.lead)*3.6,'net_gap_m':self.gap(),
            'tcp_warmup_frames':list(self.warmup_frames)}
        if any(not math.isfinite(self.initialization[key])
               for key in ('ego_speed_kmh','lead_speed_kmh')):
            raise RuntimeError('nonfinite_initial_speed')
        # CARLA's first physics step may reduce the assigned velocity. Actual
        # target-speed attainment is verified continuously by ACQUIRE, not by
        # rejecting a finite transient on the handoff frame.
        # The stable-following timer starts in step(), after control handoff.
    def gap(self):
        # Measure longitudinal separation along the selected route, including
        # connector curvature; a fixed initial-heading plane is not sufficient.
        def ends(actor):
            return [project({'x':v.x,'y':v.y},self.route)['s']
                    for v in actor.bounding_box.get_world_vertices(actor.get_transform())]
        return min(ends(self.lead))-max(ends(self.ego))
    def step(self):
        snap=self.world.get_snapshot();now=snap.timestamp.elapsed_seconds;frame=snap.frame
        if self.last_control_frame is not None and frame<=self.last_control_frame:raise RuntimeError('duplicate_or_reversed_frame')
        ev,lv=speed(self.ego),speed(self.lead);gap=self.gap();self.min_gap=min(self.min_gap,gap)
        dt=.02 if self.previous_time is None else now-self.previous_time
        self.previous_time=now
        lf=self.lead.get_transform().get_forward_vector();acc=self.lead.get_acceleration()
        decel=-(acc.x*lf.x+acc.y*lf.y+acc.z*lf.z)
        ep,lp=project(point(self.ego),self.route),project(point(self.lead),self.route)
        # Physical envelope check supplements map-derived lane callbacks.
        def envelope_lateral(actor):
            return max(project({'x':v.x,'y':v.y},self.route,extend_ends=True)['lateral'] for v in actor.bounding_box.get_world_vertices(actor.get_transform()))
        ego_envelope=envelope_lateral(self.ego);lead_envelope=envelope_lateral(self.lead)
        ego_solid=any(x['actor']=='ego' and crosses_solid(x) for x in self.lanes)
        lead_solid=any(x['actor']=='lead' and crosses_solid(x) for x in self.lanes)
        ego_collision=any(x['actor']=='ego' for x in self.collisions)
        lead_collision=any(x['actor']=='lead' for x in self.collisions)
        if (lead_solid or lead_collision) and not ego_collision:
            reason='lead_crossed_solid_boundary' if lead_solid else 'lead_collision'
            if reason not in self.trial.invalid:self.trial.invalid.append(reason)
            self.trial.phase='INVALID'
        self.trial.update(now,ev*3.6,lv*3.6,decel,ordered=gap>0,collision=ego_collision,lane_violation=ego_solid)
        self.telemetry.append({'frame':frame,'sim_time':now,'phase':self.trial.phase,'ego':point(self.ego),'lead':point(self.lead),'ego_speed_kmh':ev*3.6,'lead_speed_kmh':lv*3.6,'lead_deceleration_mps2':decel,'net_gap_m':gap,'ego_lateral_m':ep['lateral'],'ego_envelope_lateral_m':ego_envelope,'lead_envelope_lateral_m':lead_envelope,'ego_light_state':str(self.ego.get_light_state()),'road_speed_limit_kmh':self.ego.get_speed_limit()})
        if self.trial.phase in ('DONE','INVALID','FAILED'):return False
        if ep['s']>ep['length']-5 or lp['s']>lp['length']-5:
            if self.trial.phase=='ACQUIRE' and self.model=='tcp':
                self.trial.failures.append('stable_following_not_established_before_route_end')
                self.trial.phase='FAILED'
            else:
                self.trial.invalid.append('route_exhausted_before_trial_completion')
                self.trial.phase='INVALID'
            return False
        lc=carla.VehicleControl(steer=steer(self.lead,self.route))
        if self.trial.phase=='BRAKE':
            if self.lead_constant_velocity:
                self.lead.disable_constant_velocity()
                self.lead_constant_velocity=False
            self.brake_command=max(0.,min(1.,self.brake_command+self.eng['brake_kp']*(self.eng['deceleration_target_mps2']-decel)*dt))
            lc.brake=1. if lv<.1 else self.brake_command
        else:
            # Only the target vehicle is kinematically speed-controlled. Ego
            # remains under the tested controller throughout acquisition.
            lc.throttle=0.;lc.brake=0.
        self.lead.apply_control(lc)
        if self.model=='tcp':
            from stf_vehicle_control import decode_vehicle_action
            import torch
            rgb=self.camera.wait(frame,now,self.eng['camera_timeout_s'])
            if self.world.get_snapshot().frame!=frame:raise RuntimeError('world_advanced_before_inference')
            with torch.inference_mode():action=self.tcp.get_action(rgb,self.ego)
            throttle,turn,brake=decode_vehicle_action(action)
            control=carla.VehicleControl(throttle=throttle,steer=turn,brake=brake)
        else:
            # Explicit reference ACC. TCP never uses this path.
            desired_gap=self.eng['initial_net_gap_m']
            target=max(0.,min(self.d['test_speed_kmh']/3.6,lv+(gap-desired_gap)*.45))
            error=target-ev
            if error<-.2:self.ego_speed_integral=0.
            else:self.ego_speed_integral=max(0.,min(.8,self.ego_speed_integral+.3*error*dt))
            control=carla.VehicleControl(throttle=max(0.,min(.8,.5*error+self.ego_speed_integral)) if error>=-.2 else 0.,steer=steer(self.ego,self.route),brake=max(0.,min(1.,-.35*error)))
        if self.world.get_snapshot().frame!=frame:raise RuntimeError('world_advanced_during_control')
        self.ego.apply_control(control);self.last_control_frame=frame
        control_sample={'world_frame':frame,'camera_frame':frame if self.model=='tcp' else None,
                        'throttle':control.throttle,'steer':control.steer,'brake':control.brake,
                        'lead_throttle':lc.throttle,'lead_steer':lc.steer,'lead_brake':lc.brake,
                        'lead_constant_velocity':self.lead_constant_velocity}
        if self.model=='tcp':
            # Snapshot the already JSON-safe branch diagnostics produced by
            # TCPAgent. This records the controller's reasoning without
            # changing its requested action.
            control_sample['tcp_debug']=dict(self.tcp.last_debug)
        self.controls.append(control_sample)
        return True
    def finish(self):
        # Stop callbacks before making the durable decision. Do not tick extra motion.
        if self.lead is not None and self.lead_constant_velocity:
            self.lead.disable_constant_velocity();self.lead_constant_velocity=False
        for actor in (self.ego,self.lead):
            if actor is not None:actor.apply_control(carla.VehicleControl(brake=1.))
        time.sleep(.2) # Bounded callback-drain allowance; engineering, not a standards threshold.
        for sensor in self.sensors:
            try:sensor.stop()
            except RuntimeError:pass
        ego_collision=any(x['actor']=='ego' for x in self.collisions)
        lead_collision=any(x['actor']=='lead' for x in self.collisions)
        if ego_collision:
            self.trial.invalid=[x for x in self.trial.invalid if x!='lead_collision']
            if 'collision' not in self.trial.failures:self.trial.failures.append('collision')
            if self.trial.phase=='INVALID' and not self.trial.invalid:self.trial.phase='FAILED'
        elif lead_collision and 'lead_collision' not in self.trial.invalid:
            self.trial.invalid.append('lead_collision');self.trial.phase='INVALID'
        if any(x['actor']=='ego' and crosses_solid(x) for x in self.lanes) and 'solid_boundary_crossed' not in self.trial.failures:self.trial.failures.append('solid_boundary_crossed')
        if any(x['actor']=='lead' and crosses_solid(x) for x in self.lanes) and 'lead_crossed_solid_boundary' not in self.trial.invalid:self.trial.invalid.append('lead_crossed_solid_boundary')
        result=self.trial.result()
        result.update(controller=self.model,controller_fallback_used=False,collision=bool(self.collisions),min_net_gap_m=self.min_gap if math.isfinite(self.min_gap) else None,telemetry_samples=self.telemetry,control_samples=self.controls,collision_events=self.collisions,lane_events=self.lanes,screening_only=True,physical_field_test_compliance_claimed=False)
        result.update(initialization=self.initialization,route_plan=self.route_plan,lead_physics=self.lead_physics)
        return result
    def destroy(self):
        if self.lead is not None and self.lead_constant_velocity:
            try:self.lead.disable_constant_velocity()
            except RuntimeError:pass
            self.lead_constant_velocity=False
        for sensor in self.sensors:
            try:sensor.stop()
            except RuntimeError:pass
        for actor in reversed(self.actors):
            try:actor.destroy()
            except RuntimeError:pass
