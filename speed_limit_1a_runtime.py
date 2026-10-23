"""CARLA adapter for 1a. The CLI owns every simulation tick."""
import gzip
import json
import math
from types import SimpleNamespace

from speed_limit_1a import Route, Trial, decode_action
from speed_limit_1a_artifacts import artifact_paths


class ModelFailure(RuntimeError):
    pass


class Scene:
    def __init__(self, world, definition, source, model, checkpoint, directory,
                 video=True, camera_timeout=2., stall_timeout=30., attempt=1):
        import carla
        self.carla=carla;self.world=world;self.d=definition;self.model=model
        self.route=Route(definition['ego_route']);self.trial=Trial(definition,stall_timeout)
        self.paths=artifact_paths(directory,source,attempt)
        self.directory=directory;self.actors=[];self.sensors=[];self.collisions=[]
        self.ego=None;self.tcp=None;self.behavior=None;self.camera=None;self.writer=None
        self.camera_timeout=camera_timeout;self.last_frame=None;self.start_time=None
        self.log=None;self.video_enabled=video;self.source=source;self.checkpoint=checkpoint
        self.raw_action=None;self.applied_action=None
        self.video_camera=None;self.visualizer=None;self.video_frames=0

    def spawn(self):
        carla=self.carla;lib=self.world.get_blueprint_library();p=self.d['ego_start']
        bp=lib.find('vehicle.tesla.model3');bp.set_attribute('role_name','stf_1a_ego')
        self.ego=self.world.try_spawn_actor(bp,carla.Transform(
            carla.Location(x=p['x'],y=p['y'],z=p['z']),carla.Rotation(yaw=p['yaw'])))
        if self.ego is None:raise RuntimeError('ego_spawn_failed')
        self.actors.append(self.ego);self.ego.set_autopilot(False)
        self.ego.apply_control(carla.VehicleControl(brake=1.))
        collision=self.world.spawn_actor(lib.find('sensor.other.collision'),carla.Transform(),attach_to=self.ego)
        self.actors.append(collision);self.sensors.append(collision)
        collision.listen(lambda e:self.collisions.append({'frame':e.frame,'time_s':e.timestamp,
                                                         'other_type':e.other_actor.type_id}))
        if self.model=='behavior':
            from stf_scene import EgoRouteFollowScene
            self.behavior=EgoRouteFollowScene(None,self.world,str(self.source),'1a','route_01','behavior')
            self.behavior.ego=self.ego
            self.behavior.load_ego_route()
        else:
            from model.stf_tcp import TCPAgent,TCPRoutePlanner
            planner=TCPRoutePlanner()
            # Use saved route coordinates directly; projecting at junctions can
            # silently switch to a neighboring road. Commands contain navigation only.
            waypoints=[(SimpleNamespace(transform=SimpleNamespace(location=carla.Location(
                x=p['x'],y=p['y'],z=p['z']))),int(p.get('road_option',4))) for p in self.d['ego_route']]
            planner.set_route(waypoints)
            self.tcp=TCPAgent(str(self.checkpoint),planner);self.tcp.load_model(strict=True)
        if self.model=='tcp':
            from emergency_brake_5d_runtime import CameraFrames
            from model.stf_tcp import (TCP_CAMERA_WIDTH,TCP_CAMERA_HEIGHT,TCP_CAMERA_FOV,
                                   TCP_CAMERA_X,TCP_CAMERA_Y,TCP_CAMERA_Z)
            self.camera=CameraFrames();bp=lib.find('sensor.camera.rgb')
            for key,value in {'image_size_x':TCP_CAMERA_WIDTH,'image_size_y':TCP_CAMERA_HEIGHT,
                              'fov':TCP_CAMERA_FOV,'sensor_tick':0}.items():
                bp.set_attribute(key,str(value))
            sensor=self.world.spawn_actor(bp,carla.Transform(carla.Location(
                x=TCP_CAMERA_X,y=TCP_CAMERA_Y,z=TCP_CAMERA_Z)),attach_to=self.ego)
            self.actors.append(sensor);self.sensors.append(sensor);sensor.listen(self.camera.callback)
        self.directory.mkdir(parents=True,exist_ok=True)
        self.paths['telemetry'].parent.mkdir(parents=True,exist_ok=True)
        self.log=gzip.open(self.paths['telemetry'],'wt',encoding='utf-8')
        if self.video_enabled:
            from emergency_brake_5d_runtime import CameraFrames
            from speed_limit_1a_video import SpeedLimitVideo
            import imageio.v2 as imageio
            # This visualization camera matches render.Visualizer exactly.
            # TCP continues to use its independent original 900x256 camera.
            self.video_camera=CameraFrames();bp=lib.find('sensor.camera.rgb')
            for key,value in {'image_size_x':256,'image_size_y':256,'fov':90,'sensor_tick':0}.items():
                bp.set_attribute(key,str(value))
            sensor=self.world.spawn_actor(bp,carla.Transform(carla.Location(x=.5,z=1.6),
                carla.Rotation(pitch=-5)),attach_to=self.ego)
            self.actors.append(sensor);self.sensors.append(sensor);sensor.listen(self.video_camera.callback)
            self.visualizer=SpeedLimitVideo(self.world,self.ego,headless=True)
            self.writer=imageio.get_writer(str(self.paths['video']),fps=20,codec='libx264',pixelformat='yuv420p')

    def rgb(self,snapshot):
        return self.camera.wait(snapshot.frame,snapshot.timestamp.elapsed_seconds,self.camera_timeout)

    def warmup(self,snapshot):
        if self.camera:
            rgb=self.rgb(snapshot)
            if self.tcp:
                import torch
                try:
                    with torch.inference_mode(): action=self.tcp.get_action(rgb,self.ego)
                    decode_action(action)
                except Exception as exc:
                    raise ModelFailure('model_warmup_error:'+str(exc)) from exc

    def handoff(self,snapshot):
        if self.collisions:raise RuntimeError('initialization_collision')
        p=self.ego.get_location();s,lateral,_=self.route.project(p.x,p.y)
        if s>5 or lateral>.75:raise RuntimeError('settled_spawn_outside_saved_start')
        self.start_time=snapshot.timestamp.elapsed_seconds
        self.last_frame=snapshot.frame
        self.ego.apply_control(self.carla.VehicleControl())

    def measure(self,snapshot):
        pose=self.ego.get_transform();p=pose.location
        s,lateral,index=self.route.project(p.x,p.y)
        a,b=self.d['ego_route'][index:index+2]
        dx,dy=b['x']-a['x'],b['y']-a['y'];n=math.hypot(dx,dy);fx,fy=dx/n,dy/n
        vertices=self.ego.bounding_box.get_world_vertices(pose)
        front_s=s+max((v.x-p.x)*fx+(v.y-p.y)*fy for v in vertices)
        gates={}
        for label in ('B','C','D'):
            gate=self.d['speed_limit_test']['test_points'][label]
            if gate is None:continue
            yaw=math.radians(gate['yaw']);gx,gy=math.cos(yaw),math.sin(yaw)
            gates[label]=max((v.x-gate['x'])*gx+(v.y-gate['y'])*gy for v in vertices)
        velocity=self.ego.get_velocity()
        v=3.6*math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
        end=self.d['ego_route'][-1]
        return {'frame':snapshot.frame,'time_s':snapshot.timestamp.elapsed_seconds-self.start_time,
                'sim_time_s':snapshot.timestamp.elapsed_seconds,'front_s_m':front_s,'center_s_m':s,
                'speed_kmh':v,'lateral_m':lateral,'gates':gates,'x':p.x,'y':p.y,'z':p.z,
                'collision':bool(self.collisions),
                'outside_route':lateral>max(.5,a['lane_width_m']/2-self.ego.bounding_box.extent.y+.25),
                'at_end':math.hypot(p.x-end['x'],p.y-end['y'])<=2. and s>=end['s_m']-2.}

    def step(self,snapshot):
        if snapshot.frame!=self.last_frame and snapshot.frame!=self.last_frame+1:
            raise RuntimeError('unexpected_world_tick')
        self.last_frame=snapshot.frame
        obs=self.measure(snapshot)
        obs['previous_raw_action']=self.raw_action;obs['previous_applied_action']=self.applied_action
        self.trial.observe(obs)
        rgb=self.rgb(snapshot) if self.camera else None
        if self.writer is not None:
            video_rgb=self.video_camera.wait(snapshot.frame,snapshot.timestamp.elapsed_seconds,self.camera_timeout)
            self.writer.append_data(self.visualizer.frame(video_rgb,self.route))
            self.video_frames+=1
        obs['phase']=self.trial.phase
        rule=self.d['speed_limit_test']
        obs['rule_limit_kmh']=(rule['initial_limit_kmh'] if obs['front_s_m']<rule['restriction_start_s_m']
            else rule['restored_limit_kmh'] if rule['release_s_m'] is not None and obs['front_s_m']>=rule['release_s_m']
            else rule['restricted_limit_kmh'])
        if not self.trial.done:
            if self.tcp:
                import torch
                try:
                    with torch.inference_mode(): action=self.tcp.get_action(rgb,self.ego)
                    channels=decode_action(action)
                except Exception as exc:
                    self.trial.fail('model_output_error:'+str(exc))
                    channels=None
                if channels is not None:
                    self.raw_action=list(channels)
                    control=self.carla.VehicleControl(throttle=channels[0],steer=channels[1],brake=channels[2])
                    self.ego.apply_control(control)
                    self.applied_action=[control.throttle,control.steer,control.brake]
            else:
                try:self.behavior.follow_route()
                except Exception as exc:self.trial.fail('behavior_control_error:'+str(exc))
                control=self.ego.get_control()
                self.raw_action=self.applied_action=[control.throttle,control.steer,control.brake]
        obs['control_applied_this_frame']=not self.trial.done
        obs['raw_action']=self.raw_action;obs['applied_action']=self.applied_action
        self.log.write(json.dumps(obs,allow_nan=False)+'\n')
        return not self.trial.done

    def destroy(self):
        errors=[]
        for sensor in self.sensors:
            try:sensor.stop()
            except Exception as exc:errors.append(str(exc))
        for actor in reversed(self.actors):
            try:actor.destroy()
            except Exception as exc:errors.append(str(exc))
        if self.log:
            try:self.log.close()
            except Exception as exc:errors.append(str(exc))
        if self.writer:
            try:self.writer.close()
            except Exception as exc:errors.append(str(exc))
        if self.visualizer:
            try:self.visualizer.close()
            except Exception as exc:errors.append(str(exc))
        self.actors.clear()
        return errors
