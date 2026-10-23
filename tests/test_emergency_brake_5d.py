import copy
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from emergency_brake_5d import BrakeTrial,load_settings,project,minimum_route_length,digest,capability_labels
from emergency_brake_5d_cli import dispatch_arguments,expand,apply_result_policy,summarize_records,condition_completed
ROOT=Path(__file__).resolve().parents[1]

class BrakeRulesTests(unittest.TestCase):
    def setUp(self):self.cfg=load_settings(ROOT/'config/emergency_brake_5d.yaml');self.eng=self.cfg['engineering']
    def trial(self):return BrakeTrial(30,self.eng)
    def trigger(self,t):
        for now in (0,1,2,3):t.update(now,30,30,0)
        self.assertEqual(t.phase,'BRAKE')
    def test_aliases_and_conflicts(self):
        self.assertEqual(dispatch_arguments(['-5d','--model','tcp']),['--model','tcp'])
        self.assertEqual(dispatch_arguments(['--scenario','5.d']),[])
        self.assertEqual(dispatch_arguments(['--scenario=5d']),[])
        self.assertIsNone(dispatch_arguments(['--scenario','2a']))
        with self.assertRaises(ValueError):dispatch_arguments(['-5d','--scenario','2a'])
    def test_no_invented_capability(self):
        cfg=load_settings(ROOT/'config/emergency_brake_5d.yaml',True)
        self.assertEqual(cfg['test_speeds_kmh'],[10,20,30,40,50,60,80])
        self.assertIsNone(cfg['ads_vmax_kmh']['tcp'])
        self.assertIsNone(capability_labels(100,None)['standard_speed_case'])
        self.assertFalse(capability_labels(100,40)['within_declared_speed_range'])
        self.assertTrue(capability_labels(30,40)['standard_speed_case'])
    def test_stable_speed_and_order_required(self):
        t=self.trial();t.update(0,30,30,0);t.update(2,27,30,0);t.update(3,30,30,0);t.update(5.99,30,30,0)
        self.assertEqual(t.phase,'ACQUIRE');t.update(6,30,30,0);self.assertEqual(t.phase,'BRAKE')
        t=self.trial();t.update(0,30,30,0,ordered=False);t.update(4,30,30,0,ordered=False);self.assertEqual(t.phase,'ACQUIRE')
    def test_wrong_target_speed_not_stable(self):
        t=self.trial();t.update(0,20,20,0);t.update(4,20,20,0);self.assertEqual(t.phase,'ACQUIRE')
    def test_braking_requires_measured_deceleration(self):
        t=self.trial();self.trigger(t);t.update(4.02,28,20,2)
        self.assertEqual(t.phase,'INVALID');self.assertIsNone(t.result()['pass'])
    def test_overshoot_is_recorded_but_not_invalid(self):
        t=self.trial();self.trigger(t);t.update(3.5,25,20,8);t.update(4.1,20,10,2)
        self.assertEqual(t.phase,'BRAKE');self.assertEqual(t.peak_decel,8)
    def test_success_when_both_stopped_without_invented_hold(self):
        t=self.trial();self.trigger(t);t.update(3.2,28,27,6);t.update(4,20,15,6);t.update(5,2,0,0)
        self.assertEqual(t.phase,'BRAKE');t.update(6,0,0,0);self.assertTrue(t.result()['pass'])
    def test_collision_does_not_pass(self):
        t=self.trial();t.update(0,30,30,0,collision=True);self.assertFalse(t.result()['pass'])
    def test_lane_violation_is_failure(self):
        t=self.trial();t.update(0,30,30,0,lane_violation=True);self.assertFalse(t.result()['pass'])
    def test_setup_timeout_invalid_not_pass(self):
        t=self.trial();t.update(0,0,30,0);t.update(91,0,30,0);self.assertEqual(t.phase,'INVALID')
    def test_tcp_setup_timeout_is_valid_failure(self):
        t=BrakeTrial(30,self.eng,precondition_failure_is_failure=True)
        t.update(0,0,30,0);t.update(91,0,30,0)
        self.assertEqual(t.phase,'FAILED');self.assertTrue(t.result()['trial_valid'])
        self.assertFalse(t.result()['pass'])
        self.assertIn('stable_following_not_established',t.result()['failure_reasons'])
    def test_trial_timeout_is_invalid_not_ads_failure(self):
        t=self.trial();self.trigger(t);t.update(3.2,30,29,6);t.update(34,10,0,0)
        self.assertEqual(t.phase,'INVALID');self.assertIn('trial_observation_timeout',t.invalid)
    def test_actual_stable_follow_result(self):
        t=self.trial();self.assertFalse(t.result()['standard_follow_duration_met'])
        self.trigger(t);self.assertTrue(t.result()['standard_follow_duration_met'])
    def test_none_lane_event_is_not_solid(self):
        from emergency_brake_5d_runtime import crosses_solid
        self.assertFalse(crosses_solid({'markings':['NONE']}))
        self.assertTrue(crosses_solid({'markings':['Solid']}))
        self.assertTrue(crosses_solid({'markings':['SolidSolid']}))
    def test_tcp_debug_snapshot_is_json_safe(self):
        debug={'desired_speed_mps':5.5,'predicted_waypoints':[[1.0,2.0]],
               'final_throttle':0.3,'final_brake':0.0}
        self.assertEqual(json.loads(json.dumps(dict(debug))),debug)
    def test_projection_and_length(self):
        p=project({'x':15,'y':2},[{'x':0,'y':0},{'x':10,'y':0},{'x':20,'y':0}])
        self.assertEqual(p,{'s':15,'lateral':2,'length':20})
        self.assertGreater(minimum_route_length(60,20,self.eng),minimum_route_length(30,20,self.eng))
    def test_840_unique_conditions_and_nonoverwrite(self):
        cfg=copy.deepcopy(self.cfg);cfg['ads_vmax_kmh']={'tcp':40,'behavior':None}
        with tempfile.TemporaryDirectory() as tmp:
            b={'route_length_m':1000,'initial_center_gap_m':20};b['baseline_fingerprint']=digest(b)
            path=Path(tmp)/'baseline.json';path.write_text(json.dumps(b))
            args=SimpleNamespace(baseline=str(path),input_dir=str(Path(tmp)/'definitions'),dry_run=False)
            expand(args,cfg);files=list(Path(args.input_dir).glob('scenario_5d_*.json'))
            self.assertEqual(len(files),840)
            ds=[json.loads(p.read_text()) for p in files]
            self.assertEqual({d['test_speed_kmh'] for d in ds},{10,20,30,40,50,60,80})
            self.assertEqual(sum(d['speed_band']=='low' for d in ds),360)
            self.assertEqual(sum(d['speed_band']=='medium' for d in ds),360)
            self.assertEqual(sum(d['speed_band']=='high' for d in ds),120)
            self.assertTrue(all('vmax_kmh' not in d and 'standard_speed_case' not in d for d in ds))
            self.assertEqual(len({d['condition_fingerprint'] for d in ds}),840)
            with self.assertRaises(ValueError):expand(args,cfg)
            cfg['ads_vmax_kmh']={'tcp':None,'behavior':100}
            args.input_dir=str(Path(tmp)/'other_declaration')
            expand(args,cfg)
            second=[json.loads(p.read_text()) for p in Path(args.input_dir).glob('scenario_5d_*.json')]
            self.assertEqual({d['condition_fingerprint'] for d in ds},
                             {d['condition_fingerprint'] for d in second})
    def test_camera_exact_frame(self):
        from emergency_brake_5d_runtime import CameraFrames
        c=CameraFrames();c.frames[1]=(0.02,'stale');c.frames[3]=(0.06,'future')
        with self.assertRaises(RuntimeError):c.wait(2,.04,.01)
        def deliver():
            time.sleep(.01)
            with c.condition:c.frames[2]=(.04,'exact');c.condition.notify_all()
        thread=threading.Thread(target=deliver);thread.start()
        self.assertEqual(c.wait(2,.04,1),'exact');thread.join()
        with self.assertRaises(RuntimeError):c.wait(3,.08,.1)

if __name__=='__main__':unittest.main()

class GeometryTests(unittest.TestCase):
    def setUp(self):
        import carla
        xml='''<?xml version="1.0"?><OpenDRIVE><header revMajor="1" revMinor="4" name="5d_test" version="1" date="" north="0" south="0" east="0" west="0"/><road name="straight" length="500" id="1" junction="-1"><link/><type s="0" type="rural"/><planView><geometry s="0" x="0" y="0" hdg="0" length="500"><line/></geometry></planView><elevationProfile/><lateralProfile/><lanes><laneSection s="0"><center><lane id="0" type="none" level="false"><roadMark sOffset="0" type="solid" weight="standard" color="white" width="0.15"/></lane></center><right><lane id="-1" type="driving" level="false"><link/><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" weight="standard" color="white" width="0.15"/></lane></right></laneSection></lanes></road></OpenDRIVE>'''
        self.map=carla.Map('5d_test',xml)
        def p(s):
            w=self.map.get_waypoint_xodr(1,-1,s);l=w.transform.location
            return {'x':l.x,'y':l.y,'z':l.z,'yaw':w.transform.rotation.yaw}
        self.baseline={'ego_start':p(10),'lead_start':p(30),'ego_route':[p(200),p(450)]}
        self.eng=load_settings(ROOT/'config/emergency_brake_5d.yaml')['engineering']
    def test_forward_single_lane(self):
        from emergency_brake_5d import validate_geometry
        route,gap=validate_geometry(self.map,self.baseline,self.eng)
        self.assertAlmostEqual(gap,20,places=2);self.assertGreater(len(route),400)
    def test_reverse_rejected(self):
        from emergency_brake_5d import validate_geometry
        b=copy.deepcopy(self.baseline);b['ego_route'].reverse()
        with self.assertRaisesRegex(ValueError,'forward'):validate_geometry(self.map,b,self.eng)
    def test_lead_behind_rejected(self):
        from emergency_brake_5d import validate_geometry
        b=copy.deepcopy(self.baseline);b['lead_start']=dict(b['ego_start'],x=b['ego_start']['x']-5)
        with self.assertRaises(ValueError):validate_geometry(self.map,b,self.eng)

class RunnerLifecycleTests(unittest.TestCase):
    def test_invalid_setup_cleans_owned_actors_and_restores_world(self):
        from unittest.mock import patch
        from emergency_brake_5d_cli import run
        import emergency_brake_5d_runtime
        class World:
            def __init__(self):self.applied=[];self.weather=[]
            def get_settings(self):return SimpleNamespace(synchronous_mode=False,fixed_delta_seconds=None)
            def get_weather(self):return 'original-weather'
            def get_actors(self):return SimpleNamespace(filter=lambda pattern:[])
            def apply_settings(self,s):self.applied.append(s)
            def set_weather(self,w):self.weather.append(w)
        class FailedScene:
            destroyed=False
            def __init__(self,*args):self.trial=SimpleNamespace(invalid=[],phase='SETUP')
            def spawn(self,**kwargs):raise RuntimeError('camera startup failed')
            def finish(self):return {'trial_valid':False,'pass':None,'failure_reasons':[],'invalid_reasons':list(self.trial.invalid)}
            def destroy(self):FailedScene.destroyed=True
        world=World();client=SimpleNamespace(set_timeout=lambda x:None,get_world=lambda:world,get_server_version=lambda:'test')
        cfg=load_settings(ROOT/'config/emergency_brake_5d.yaml')
        d={'scenario':'scenario_5d_test','condition_fingerprint':'testhash','weather':{},'test_speed_kmh':30,'speed_band':'low','weather_id':'sunny_01'}
        with tempfile.TemporaryDirectory() as tmp:
            args=SimpleNamespace(model='behavior',model_path=None,host=None,port=29999,town=None,input_dir=tmp,dry_run=False,output_dir=tmp,video_dir=None,resume=False,no_video=True)
            with patch('carla.Client',return_value=client),patch('emergency_brake_5d_cli.validate_definitions',return_value=[d]),patch.object(emergency_brake_5d_runtime,'BrakeScene',FailedScene):run(args,cfg)
            result=json.loads((Path(tmp)/'5d_result.json').read_text())
            self.assertIsNone(result['records'][0]['pass'])
            self.assertEqual(result['metadata']['status'],'interrupted')
            self.assertTrue(FailedScene.destroyed)
            self.assertEqual(world.weather[-1],'original-weather')
            self.assertFalse(world.applied[-1].synchronous_mode)

class MovingStartTests(unittest.TestCase):
    def test_short_route_has_per_speed_feasibility(self):
        from emergency_brake_5d import route_feasibility
        eng=load_settings(ROOT/'config/emergency_brake_5d.yaml')['engineering']
        low=route_feasibility(80,20,192.09,eng)
        high=route_feasibility(100,20,192.09,eng)
        self.assertTrue(low['route_length_sufficient'])
        self.assertFalse(high['route_length_sufficient'])
        self.assertAlmostEqual(high['required_route_length_m'],210.412,places=2)
    def test_velocity_set_once_after_tcp_warmup(self):
        from emergency_brake_5d_runtime import BrakeScene
        import carla
        events=[]
        class World:
            frame=10
            def get_snapshot(self):return SimpleNamespace(frame=self.frame,timestamp=SimpleNamespace(elapsed_seconds=self.frame*.02))
            def tick(self):self.frame+=1;events.append(('tick',self.frame))
        class Actor:
            def __init__(self):self.velocity=carla.Vector3D();self.sets=0
            def get_transform(self):return carla.Transform(rotation=carla.Rotation(yaw=90))
            def get_velocity(self):return self.velocity
            def apply_control(self,c):events.append(('control',c.brake))
            def set_target_angular_velocity(self,v):pass
            def enable_constant_velocity(self,v):events.append(('constant_velocity',self))
            def set_target_velocity(self,v):self.velocity=v;self.sets+=1;events.append(('velocity',v.length()))
        scene=BrakeScene.__new__(BrakeScene);scene.model='tcp';scene.world=World();scene.ego=Actor();scene.lead=Actor()
        scene.initialization={};scene.warmup_frames=[];scene.eng={'camera_timeout_s':1,'speed_tolerance_kmh':1}
        scene.d={'test_speed_kmh':36};scene.gap=lambda:19
        scene.camera=SimpleNamespace(wait=lambda *args:None)
        scene.tcp=SimpleNamespace(config=SimpleNamespace(seq_len=2),get_action=lambda *args:events.append(('warmup',scene.world.frame)))
        scene._initialize_moving_start()
        self.assertEqual(scene.ego.sets,1);self.assertEqual(scene.lead.sets,1)
        self.assertAlmostEqual(scene.ego.velocity.y,10)
        self.assertLess(max(i for i,e in enumerate(events) if e[0]=='warmup'),min(i for i,e in enumerate(events) if e[0]=='velocity'))
        self.assertEqual(scene.initialization['handoff_frame'],14)
        self.assertEqual([e[1] for e in events if e[0]=='constant_velocity'],[scene.lead])
        with self.assertRaises(RuntimeError):scene._initialize_moving_start()
    def test_short_route_conditions_not_removed(self):
        cfg=load_settings(ROOT/'config/emergency_brake_5d.yaml',True)
        cfg['test_speeds_kmh'].append(100)
        with tempfile.TemporaryDirectory() as tmp:
            b={'route_length_m':192.09,'initial_center_gap_m':20};b['baseline_fingerprint']=digest(b)
            p=Path(tmp)/'base.json';p.write_text(json.dumps(b))
            args=SimpleNamespace(baseline=str(p),input_dir=str(Path(tmp)/'definitions'),dry_run=False)
            expand(args,cfg)
            ds=[json.loads(p.read_text()) for p in Path(args.input_dir).glob('scenario_5d_*.json')]
            self.assertEqual(len(ds),960)
            blocked=[d for d in ds if not d['route_plan']['route_length_sufficient']]
            self.assertEqual(len(blocked),120);self.assertEqual({d['test_speed_kmh'] for d in blocked},{100})

class ThreeSecondTriggerTests(unittest.TestCase):
    def test_exact_three_second_boundary(self):
        eng=load_settings(ROOT/'config/emergency_brake_5d.yaml')['engineering']
        t=BrakeTrial(30,eng)
        t.update(10,30,30,0);t.update(12.99,30,30,0)
        self.assertEqual(t.phase,'ACQUIRE')
        t.update(13,30,30,0)
        self.assertEqual(t.phase,'BRAKE')
        self.assertEqual(t.brake_at,13)
        self.assertTrue(t.result()['standard_follow_duration_met'])

class FixedGapTests(unittest.TestCase):
    def test_fixed_net_gap_ignores_speed_and_drawn_gap(self):
        from emergency_brake_5d import moving_center_gap
        eng=load_settings(ROOT/'config/emergency_brake_5d.yaml')['engineering']
        for speed in (10,30,60,80,100):
            for drawn in (20,80):
                self.assertEqual(moving_center_gap(speed,drawn,eng,4.8),24.8)

class AutomaticLeadTests(GeometryTests):
    def test_lead_generated_without_manual_actor(self):
        from emergency_brake_5d import validate_geometry
        del self.baseline['lead_start']
        route,gap=validate_geometry(self.map,self.baseline,self.eng,auto_lead=True)
        self.assertAlmostEqual(gap,25,places=2)
        self.assertAlmostEqual(self.baseline['lead_start']['x']-self.baseline['ego_start']['x'],25,places=2)
    def test_short_route_cannot_save_auto_lead(self):
        from emergency_brake_5d import validate_geometry
        self.baseline['ego_route']=[dict(self.baseline['ego_start'],x=20),dict(self.baseline['ego_start'],x=30)]
        with self.assertRaises(ValueError):validate_geometry(self.map,self.baseline,self.eng,auto_lead=True)

class GeometryWarningTests(GeometryTests):
    def test_broken_markings_warn_instead_of_blocking(self):
        import carla
        from emergency_brake_5d import validate_geometry
        xml=self.map.to_opendrive().replace('type="solid"','type="broken"')
        m=carla.Map('broken_marking_variant',xml)
        b=copy.deepcopy(self.baseline)
        route,gap=validate_geometry(m,b,self.eng,auto_lead=True)
        self.assertTrue(route)
        self.assertTrue(any('实线' in warning for warning in b['geometry_warnings']))

class EndpointEnvelopeTests(unittest.TestCase):
    def test_rear_overhang_is_not_lateral_lane_departure(self):
        route=[{'x':0,'y':0},{'x':50,'y':0},{'x':100,'y':0}]
        self.assertAlmostEqual(project({'x':-2.4,'y':1},route,extend_ends=True)['lateral'],1)
        self.assertAlmostEqual(project({'x':102.4,'y':1},route,extend_ends=True)['lateral'],1)
        self.assertGreater(project({'x':-2.4,'y':2},route,extend_ends=True)['lateral'],1.75)


class LeadBrakePhysicsTests(unittest.TestCase):
    def actor(self, apply=True):
        class Physics:
            mass=1845.
            def __init__(self):
                self._wheels=[SimpleNamespace(max_brake_torque=700.,radius=37.,tire_friction=3.5) for _ in range(4)]
            @property
            def wheels(self):return copy.deepcopy(self._wheels)
            @wheels.setter
            def wheels(self,value):self._wheels=copy.deepcopy(value)
        class Actor:
            type_id='vehicle.tesla.model3'
            def __init__(self):self.physics=Physics();self.applied=0;self.pending=None
            def get_physics_control(self):return copy.deepcopy(self.physics)
            def apply_physics_control(self,physics):
                self.applied+=1
                self.pending=copy.deepcopy(physics)
            def flush(self):
                if apply and self.pending is not None:self.physics=self.pending
                self.pending=None
        return Actor()
    def scene(self, lead):
        from emergency_brake_5d_runtime import BrakeScene
        scene=BrakeScene.__new__(BrakeScene)
        scene.eng=load_settings(ROOT/'config/emergency_brake_5d.yaml')['engineering']
        scene.ego=self.actor();scene.lead=lead
        scene.world=SimpleNamespace(tick=lead.flush)
        return scene
    def test_only_lead_brake_torque_changes_and_is_recorded(self):
        scene=self.scene(self.actor())
        scene._configure_lead_brakes()
        self.assertEqual(scene.ego.applied,0)
        self.assertEqual([w.max_brake_torque for w in scene.ego.get_physics_control().wheels],[700.]*4)
        self.assertEqual(scene.lead_physics['original_max_brake_torque_nm'],[700.]*4)
        self.assertEqual(scene.lead_physics['applied_max_brake_torque_nm'],[1400.]*4)
        self.assertEqual(scene.lead_physics['mass_kg'],1845.)
        self.assertEqual(scene.lead_physics['wheel_radius_cm'],[37.]*4)
        self.assertEqual(scene.lead_physics['tire_friction'],[3.5]*4)
    def test_rejected_physics_change_cannot_be_silently_used(self):
        scene=self.scene(self.actor(apply=False))
        with self.assertRaisesRegex(RuntimeError,'lead_brake_physics_not_applied'):
            scene._configure_lead_brakes()
    def test_nonpositive_torque_configuration_rejected(self):
        import yaml
        cfg=load_settings(ROOT/'config/emergency_brake_5d.yaml')
        cfg['engineering']['lead_max_brake_torque_nm']=0
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'config.yaml';path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError,'lead_max_brake_torque_nm'):
                load_settings(path)


class IncompleteFailurePolicyTests(unittest.TestCase):
    def record(self,reason):
        return {'scenario':reason,'trial_valid':False,'pass':None,'phase':'INVALID',
                'failure_reasons':[],'invalid_reasons':[reason]}
    def test_all_incomplete_causes_score_failed_with_original_evidence(self):
        reasons=['trial_observation_timeout','route_exhausted_before_trial_completion',
                 'insufficient_route_length','lead_deceleration_target_not_reached',
                 'RuntimeError: camera_frame_timeout']
        records=[]
        for reason in reasons:
            r=apply_result_policy(self.record(reason),True);records.append(r)
            self.assertEqual(r['result_status'],'FAILED');self.assertFalse(r['pass'])
            self.assertFalse(r['trial_valid']);self.assertIsNone(r['raw_trial_result']['pass'])
            self.assertEqual(r['invalid_reasons'],[reason]);self.assertTrue(condition_completed(r))
        summary=summarize_records(records,len(records)+1)
        self.assertEqual(summary['fail'],5);self.assertEqual(summary['failed_incomplete'],5)
        self.assertEqual(summary['completed_valid'],0);self.assertEqual(summary['invalid_trials'],5)
        self.assertEqual(summary['invalid'],0);self.assertEqual(summary['remaining'],1)
        self.assertEqual(summary['pending_unattempted'],1)
    def test_default_policy_keeps_invalid_unresolved(self):
        r=apply_result_policy(self.record('trial_observation_timeout'))
        self.assertIsNone(r['pass']);self.assertEqual(r['result_status'],'INVALID')
        self.assertFalse(condition_completed(r))
        summary=summarize_records([r],1)
        self.assertEqual(summary['fail'],0);self.assertEqual(summary['invalid'],1)
        self.assertEqual(summary['remaining'],1)
    def test_valid_pass_and_fail_remain_unchanged(self):
        for passed in (True,False):
            r={'scenario':str(passed),'trial_valid':True,'pass':passed,'phase':'DONE' if passed else 'FAILED',
               'failure_reasons':[] if passed else ['collision'],'invalid_reasons':[]}
            apply_result_policy(r,True)
            self.assertEqual(r['pass'],passed);self.assertTrue(r['trial_valid'])
            self.assertEqual(r['result_status'],'PASS' if passed else 'FAILED')
            self.assertNotIn('raw_trial_result',r)
    def test_runner_continues_and_resume_does_not_retry_scored_failures(self):
        from unittest.mock import patch
        from emergency_brake_5d_cli import run
        import emergency_brake_5d_runtime
        class World:
            def get_settings(self):return SimpleNamespace(synchronous_mode=False,fixed_delta_seconds=None)
            def get_weather(self):return 'original'
            def get_actors(self):return SimpleNamespace(filter=lambda pattern:[])
            def apply_settings(self,s):pass
            def set_weather(self,w):pass
        class FailedScene:
            spawned=0;destroyed=0
            def __init__(self,*args):self.trial=SimpleNamespace(invalid=[],phase='SETUP')
            def spawn(self,**kwargs):
                FailedScene.spawned+=1
                raise RuntimeError('camera startup failed')
            def finish(self):return {'trial_valid':False,'pass':None,'phase':'INVALID','failure_reasons':[],'invalid_reasons':list(self.trial.invalid)}
            def destroy(self):FailedScene.destroyed+=1
        client=SimpleNamespace(set_timeout=lambda x:None,get_world=lambda:World(),get_server_version=lambda:'test')
        cfg=load_settings(ROOT/'config/emergency_brake_5d.yaml')
        definitions=[{'scenario':'scenario_5d_test_'+str(i),'condition_fingerprint':str(i),'weather':{},
                      'test_speed_kmh':30,'speed_band':'low','weather_id':'sunny_01'} for i in range(2)]
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint=Path(tmp)/'test.ckpt';checkpoint.write_bytes(b'test checkpoint')
            args=SimpleNamespace(model='tcp',model_path=str(checkpoint),host=None,port=29998,town=None,
                input_dir=tmp,dry_run=False,output_dir=tmp,video_dir=None,resume=False,no_video=True,incomplete_as_failed=True)
            with patch('carla.Client',return_value=client),patch('emergency_brake_5d_cli.validate_definitions',return_value=definitions),patch.object(emergency_brake_5d_runtime,'BrakeScene',FailedScene):
                run(args,cfg)
                result=json.loads((Path(tmp)/'5d_result.json').read_text())
                self.assertEqual(result['metadata']['status'],'completed')
                self.assertEqual(result['summary']['fail'],2);self.assertEqual(result['summary']['remaining'],0)
                self.assertEqual(result['summary']['failed_incomplete'],2)
                self.assertEqual(FailedScene.destroyed,2)
                args.resume=True;run(args,cfg)
                self.assertEqual(FailedScene.spawned,2)
                args.incomplete_as_failed=False
                with self.assertRaisesRegex(ValueError,'resume identity mismatch'):run(args,cfg)
