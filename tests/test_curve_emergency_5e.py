import copy
from contextlib import ExitStack, nullcontext
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from curve_emergency_5e import (CurveTrial, RISK_IDS, digest, geometry_metrics, load_settings,
                                 make_baseline, make_definitions, validate_definitions, crossing_delay,
                                 ensure_world, map_identifier, map_matches)
from curve_emergency_5e_cli import dispatch_arguments, expand, select_definitions

ROOT = Path(__file__).resolve().parents[1]


def fixture():
    cfg = load_settings(ROOT/'config/curve_emergency_5e.yaml')
    route = [dict(x=50*math.sin(i/50),y=50*(1-math.cos(i/50)),z=0.,
                  yaw=math.degrees(i/50),width=4.) for i in range(111)]
    p = route[55]
    normal = (-math.sin(55/50),math.cos(55/50))
    b = dict(schema_version=1,scenario_code='5e',map_name=cfg['town'],opendrive_sha256='offline',
             ego_start=route[0],ego_route=route,route_anchors=[route[40],route[80],route[-1]],
             risk_position=p,trigger_position=route[10],
             crossing_start=dict(x=p['x']-normal[0]*10,y=p['y']-normal[1]*10,z=0.),
             crossing_end=dict(x=p['x']+normal[0]*10,y=p['y']+normal[1]*10,z=0.))
    b['geometry'] = geometry_metrics(b,cfg['engineering'])
    b['baseline_fingerprint'] = digest(b)
    return cfg,b


class MapSelectionTests(unittest.TestCase):
    target = '/Game/stf_test_map_package/Maps/t_shaped_intersection_scene/t_shaped_intersection_scene'

    def client(self, current='Town01', maps=None, busy=False):
        state = {'loaded': [], 'timeouts': [], 'listed': 0}
        def world(name):
            return SimpleNamespace(get_map=lambda:SimpleNamespace(name=name),
                                   get_actors=lambda:SimpleNamespace(filter=lambda _:[object()] if busy else []))
        def available():
            state['listed']+=1
            return [self.target] if maps is None else maps
        def load(name):
            state['loaded'].append(name)
            return world(name)
        client = SimpleNamespace(get_world=lambda:world(current), get_available_maps=available,
                                 load_world=load,set_timeout=state['timeouts'].append)
        return client,state

    def test_default_map_and_umap_name_resolution(self):
        cfg=load_settings(ROOT/'config/curve_emergency_5e.yaml')
        self.assertEqual(cfg['town'],'t_shaped_intersection_scene')
        self.assertEqual(map_identifier('t_shaped_intersection_scene.umap'),cfg['town'])
        self.assertTrue(map_matches(self.target,cfg['town']))
        client,state=self.client()
        world=ensure_world(client,'t_shaped_intersection_scene.umap')
        self.assertEqual(world.get_map().name,self.target)
        self.assertEqual(state['loaded'],[self.target])
        self.assertEqual(state['timeouts'],[120,20])

    def test_matching_world_is_not_reloaded(self):
        client,state=self.client(current=self.target,busy=True)
        ensure_world(client,'t_shaped_intersection_scene')
        self.assertEqual(state['loaded'],[])
        self.assertEqual(state['listed'],0)

    def test_missing_ambiguous_and_busy_maps_do_not_reload(self):
        for maps,busy in [([],False),([self.target,'/Game/other/t_shaped_intersection_scene'],False),
                          ([self.target],True)]:
            client,state=self.client(maps=maps,busy=busy)
            with self.assertRaises((ValueError,RuntimeError)):
                ensure_world(client,'t_shaped_intersection_scene')
            self.assertEqual(state['loaded'],[])

    def test_expansion_rejects_a_baseline_from_another_map(self):
        cfg,b=fixture();b['map_name']='Town01'
        b.pop('baseline_fingerprint');b['baseline_fingerprint']=digest(b)
        with self.assertRaisesRegex(ValueError,'map mismatch'):make_definitions(b,cfg)


class DefinitionTests(unittest.TestCase):
    def setUp(self):
        self.cfg,self.b = fixture()

    def test_dispatch_aliases_and_conflicting_scenarios(self):
        self.assertEqual(dispatch_arguments(['-5e','--model','tcp']),['--model','tcp'])
        self.assertEqual(dispatch_arguments(['--scenario','5.e']),[])
        self.assertEqual(dispatch_arguments(['--scenario=5E']),[])
        self.assertIsNone(dispatch_arguments(['--scenario','5d']))
        for argv in (['-5e','-5d'],['-5e','--scenario','2a'],['--scenario','5e','--scenario','5d']):
            with self.assertRaises(ValueError):dispatch_arguments(argv)

    def test_entrypoint_help_does_not_load_legacy_runtime(self):
        for script,flag in [('waypoints_stf.py','-5e'),('run_stf.py','--scenario=5e'),
                            ('waypoints_stf.py','-5d'),('run_stf.py','-5d')]:
            p = subprocess.run([sys.executable,'-B',str(ROOT/script),flag,'--help'],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr)
            self.assertIn('usage:',p.stdout)

    def test_matrix_has_960_unique_fixed_geometry_conditions(self):
        ds = make_definitions(self.b,self.cfg)
        self.assertEqual(len(ds),960)
        self.assertEqual(len({d['condition_fingerprint'] for d in ds}),960)
        self.assertEqual({d['risk']['id'] for d in ds},set(RISK_IDS))
        self.assertEqual(len({d['weather_id'] for d in ds}),120)
        self.assertEqual({digest(d['baseline']) for d in ds},{digest(self.b)})
        for rid in RISK_IDS:
            self.assertEqual(len({digest(d['risk']) for d in ds if d['risk']['id']==rid}),1)
        self.assertTrue(all(not d['collision_enhancement_enabled'] for d in ds))

    def test_derived_delay_aligns_reference_arrivals_without_new_dimensions(self):
        g = self.b['geometry']
        arrival = (g['risk_s']-g['trigger_s'])/(self.cfg['ego_speed_kmh']/3.6)
        for r in self.cfg['risks'][3:]:
            delay = crossing_delay(g,r,self.cfg['ego_speed_kmh'])
            self.assertAlmostEqual(delay+g['crossing_conflict_s']/r['speed_mps'],arrival)
        self.assertEqual(crossing_delay(dict(g,trigger_s=g['risk_s']-5),self.cfg['risks'][3],20),0.)

    def test_trigger_has_no_distance_or_order_restriction(self):
        for index in (0,1,54,55,65):
            b=copy.deepcopy(self.b);b['trigger_position']=b['ego_route'][index]
            g=geometry_metrics(b,self.cfg['engineering'])
            self.assertGreaterEqual(g['trigger_s'],0.)
            for risk in self.cfg['risks'][3:]:
                delay=crossing_delay(g,risk,self.cfg['ego_speed_kmh'])
                self.assertGreaterEqual(delay,0.)
                if index >= 54:self.assertEqual(delay,0.)

    def test_risk_must_be_in_bend_and_crossing_must_pass_through_it(self):
        b = copy.deepcopy(self.b)
        for i,p in enumerate(b['ego_route']):p.update(x=float(i),y=0.,yaw=0.)
        with self.assertRaisesRegex(ValueError,'bend'):geometry_metrics(b,self.cfg['engineering'])
        b = copy.deepcopy(self.b);b['crossing_end']['x']+=8
        with self.assertRaisesRegex(ValueError,'pass through'):geometry_metrics(b,self.cfg['engineering'])

    def test_expansion_disk_round_trip_tampering_and_no_overwrite(self):
        # A near-P trigger must survive expansion and run preflight, not only the editor.
        self.b['trigger_position']=self.b['ego_route'][54]
        self.b['geometry']=geometry_metrics(self.b,self.cfg['engineering'])
        self.b.pop('baseline_fingerprint');self.b['baseline_fingerprint']=digest(self.b)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);baseline=root/'b.json';baseline.write_text(json.dumps(self.b))
            args = SimpleNamespace(baseline=str(baseline),input_dir=str(root/'definitions'),dry_run=False)
            expand(args,self.cfg)
            paths = sorted(Path(args.input_dir).glob('scenario_5e_*.json'))
            ds = validate_definitions(paths,self.cfg)
            selected = select_definitions(ds,SimpleNamespace(risks=['truck'],weather='sunny_07'))
            self.assertEqual(len(selected),1)
            self.assertEqual(selected[0]['risk']['blueprint'],'vehicle.carlamotors.carlacola')
            self.assertEqual(selected[0]['risk']['start_delay_s'],0.)
            with self.assertRaisesRegex(ValueError,'not empty'):expand(args,self.cfg)
            with self.assertRaisesRegex(ValueError,'complete'):validate_definitions(paths[:-1],self.cfg)
            changed = json.loads(paths[0].read_text());changed['risk']['start_delay_s']+=1
            changed.pop('condition_fingerprint');changed['condition_fingerprint']=digest(changed)
            paths[0].write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError,'differs'):validate_definitions(paths,self.cfg)


class ScoringTests(unittest.TestCase):
    def setUp(self):
        cfg,b=fixture();self.ds=make_definitions(b,cfg)

    def trial(self,risk,**kwargs):
        return CurveTrial(next(d for d in self.ds if d['risk']['id']==risk),**kwargs)

    def update(self,t,now,**kwargs):
        values=dict(ego_s=45.,ego_speed=0.,gap=5.)
        values.update(kwargs);t.update(now,**values)

    def test_tcp_stationary_timeout_uses_continuous_simulation_seconds(self):
        t=self.trial('bicycle',stationary_timeout_s=30.)
        self.update(t,100,ego_s=0,gap=50)
        self.update(t,130,ego_s=0,gap=50)
        self.assertEqual(t.status,'RUNNING')
        self.update(t,130.05,ego_s=0,gap=50)
        self.assertEqual(t.status,'FAILED')
        self.assertEqual(t.reasons,['ego_stationary_timeout'])
        self.assertAlmostEqual(t.result()['ego_stationary_duration_s'],30.05)
        self.assertEqual(t.result()['ego_stationary_since_sim_time'],100)

    def test_tcp_movement_resets_stationary_timer(self):
        t=self.trial('bicycle',stationary_timeout_s=30.)
        self.update(t,10,ego_s=0,gap=50,ego_speed=.15)
        self.update(t,39,ego_s=0,gap=50,ego_speed=.15)
        self.update(t,40,ego_s=0,gap=50,ego_speed=.16)
        self.assertIsNone(t.stationary_since)
        self.update(t,41,ego_s=0,gap=50,ego_speed=0)
        self.update(t,71,ego_s=0,gap=50,ego_speed=0)
        self.assertEqual(t.status,'RUNNING')
        self.update(t,71.05,ego_s=0,gap=50,ego_speed=0)
        self.assertEqual(t.reasons,['ego_stationary_timeout'])

    def test_tcp_safe_stop_keeps_pass_and_far_stop_times_out(self):
        t=self.trial('box',stationary_timeout_s=30.)
        self.update(t,0);self.update(t,2)
        self.assertEqual(t.result()['completion_reason'],'safe_stop_before_obstacle')
        self.update(t,31)
        self.assertEqual(t.status,'PASS')
        t=self.trial('box',stationary_timeout_s=30.)
        self.update(t,0,gap=21);self.update(t,30.05,gap=21)
        self.assertEqual(t.reasons,['ego_stationary_timeout'])

    def test_tcp_completion_and_collision_precede_stationary_timeout(self):
        for key in ('at_end','target_finished'):
            with self.subTest(event=key):
                t=self.trial('truck',stationary_timeout_s=30.)
                self.update(t,0,ego_s=0,gap=50)
                self.update(t,30.05,ego_s=0,gap=50,**{key:True})
                self.assertEqual(t.status,'PASS')
        t=self.trial('truck',stationary_timeout_s=30.)
        self.update(t,0,ego_s=0,gap=50)
        self.update(t,30.05,ego_s=0,gap=50,collision=True,target_finished=True)
        self.assertEqual(t.reasons,['collision'])

    def test_behavior_retains_overall_timeout_without_stationary_limit(self):
        t=self.trial('truck')
        self.update(t,0,ego_s=0,gap=50);self.update(t,31,ego_s=0,gap=50)
        self.assertEqual(t.status,'RUNNING')
        self.assertIsNone(t.result()['ego_stationary_timeout_s'])
        self.update(t,120,ego_s=0,gap=50)
        self.assertEqual(t.reasons,['ego_task_timeout'])

    def test_static_safe_stop_requires_hold_and_reachable_obstacle(self):
        t=self.trial('box')
        self.update(t,0);self.update(t,1.99);self.assertEqual(t.status,'RUNNING')
        self.update(t,2);self.assertEqual(t.status,'PASS')
        t=self.trial('box')
        self.update(t,0,ego_s=0,gap=50);self.update(t,t.eng['timeout_s'],ego_s=0,gap=50)
        self.assertEqual(t.status,'FAILED')

    def test_dynamic_waiting_forever_is_not_success(self):
        t=self.trial('pedestrian')
        self.update(t,0);self.update(t,5,target_clear=True);self.update(t,t.eng['timeout_s'],target_clear=True)
        self.assertEqual(t.status,'FAILED')
        self.assertIn('ego_task_timeout',t.reasons)

    def test_ego_endpoint_completes_without_waiting_for_npc(self):
        t=self.trial('truck')
        self.update(t,0);trigger=t.triggered
        self.update(t,2,at_end=True,target_clear=False)
        self.assertEqual(t.status,'PASS');self.assertEqual(t.triggered,trigger)
        self.assertEqual(t.result()['completion_reason'],'ego_reached_route_end')

    def test_npc_endpoint_completes_without_waiting_for_ego_route(self):
        t=self.trial('truck');self.update(t,0)
        self.update(t,5,target_clear=True,target_finished=False)
        self.assertEqual(t.status,'RUNNING')
        self.update(t,6,target_clear=True,target_finished=True,at_end=False)
        self.assertEqual(t.status,'PASS')
        self.assertEqual(t.result()['completion_reason'],'target_reached_crossing_end')

    def test_collision_terminates_as_failed_even_at_endpoint(self):
        t=self.trial('truck');self.update(t,0)
        self.update(t,6,target_finished=True,at_end=True,collision=True)
        self.assertEqual(t.status,'FAILED');self.assertEqual(t.reasons,['collision'])

    def test_collisions_and_lane_violations_cannot_pass(self):
        for key in ('collision','outside'):
            t=self.trial('box');self.update(t,0);self.update(t,2,**{key:True})
            self.assertEqual(t.status,'FAILED')

    def test_late_collision_replaces_provisional_pass_reason(self):
        t=self.trial('box');self.update(t,0);self.update(t,2)
        self.assertEqual(t.status,'PASS')
        t.end('FAILED','collision')
        self.assertEqual(t.result()['failure_reasons'],['collision'])

    def test_target_failure_invalid_but_ego_failure_is_failed(self):
        t=self.trial('bicycle');self.update(t,0,target_error='target_left_fixed_crossing_path')
        self.assertEqual(t.result()['result_status'],'INVALID');self.assertIsNone(t.result()['pass'])
        t=self.trial('truck');self.update(t,0);self.update(t,40)
        self.assertEqual(t.status,'INVALID')
        t=self.trial('truck');self.update(t,0,collision=True,target_error='target_speed_not_established')
        self.assertEqual(t.status,'FAILED')


class CarlaApiTests(unittest.TestCase):
    def setUp(self):
        try:
            import carla
        except ImportError:
            self.skipTest('requires installed CARLA Python API, no server needed')
        self.carla=carla
        self.cfg,self.b=fixture()

    def test_native_opendrive_curve_trace_and_revalidation(self):
        xml='''<?xml version="1.0"?><OpenDRIVE><header revMajor="1" revMinor="4" name="5e" version="1" date="" north="0" south="0" east="0" west="0"/><road name="curve" length="140" id="1" junction="-1"><link/><type s="0" type="rural"/><planView><geometry s="0" x="0" y="0" hdg="0" length="140"><arc curvature="0.02"/></geometry></planView><elevationProfile/><lateralProfile/><lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center><right><lane id="-1" type="driving" level="false"><link/><width sOffset="0" a="4" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road></OpenDRIVE>'''
        m=self.carla.Map('5e_test',xml)
        self.cfg['town']=m.name
        def point(s):
            tf=m.get_waypoint_xodr(1,-1,s).transform
            return dict(x=tf.location.x,y=tf.location.y,z=tf.location.z,yaw=tf.rotation.yaw)
        p=point(70);yaw=math.radians(p['yaw']);normal=(-math.sin(yaw),math.cos(yaw))
        draft=dict(ego_start=point(20),route_anchors=[point(40),point(60),point(90),point(120)],
                   risk_position=p,trigger_position=point(25),
                   crossing_start=dict(p,x=p['x']-normal[0]*10,y=p['y']-normal[1]*10),
                   crossing_end=dict(p,x=p['x']+normal[0]*10,y=p['y']+normal[1]*10))
        b=make_baseline(m,draft,self.cfg)
        self.assertGreater(len(b['ego_route']),100)
        self.assertGreater(b['geometry']['curve_angle_deg'],8)
        from curve_emergency_5e_cli import validate_world
        ds=make_definitions(b,self.cfg)
        world=SimpleNamespace(get_map=lambda:m,get_blueprint_library=lambda:[SimpleNamespace(id=r['blueprint']) for r in self.cfg['risks']])
        validate_world(world,ds)
        for trigger_s in (20,69.5,70,72):
            draft['trigger_position']=point(trigger_s)
            relaxed=make_baseline(m,draft,self.cfg)
            self.assertIn('baseline_fingerprint',relaxed)
        draft['route_anchors']=[point(10),point(40)]
        with self.assertRaises(ValueError):make_baseline(m,draft,self.cfg)

    def test_truck_tail_must_clear_not_only_center(self):
        from curve_emergency_5e_runtime import CurveScene, transform
        d=next(d for d in make_definitions(self.b,self.cfg) if d['risk']['id']=='truck')
        scene=CurveScene(None,d,'behavior','')
        scene.target_started=True
        start,end=scene.crossing;dx=(end['x']-start['x'])/20;dy=(end['y']-start['y'])/20
        box=self.carla.BoundingBox(self.carla.Location(),self.carla.Vector3D(x=4,y=1,z=2))
        def actor(s):
            pose=transform(dict(x=start['x']+s*dx,y=start['y']+s*dy,z=0),scene.geometry['crossing_yaw'])
            return SimpleNamespace(bounding_box=box,get_transform=lambda:pose)
        scene.target=actor(14)
        self.assertFalse(scene.target_is_clear())
        scene.target=actor(18)
        self.assertTrue(scene.target_is_clear())

    def test_crossing_space_uses_requested_pose_before_snapshot_arrives(self):
        from curve_emergency_5e_runtime import CurveScene, transform
        d=next(d for d in make_definitions(self.b,self.cfg) if d['risk']['id']=='truck')
        s=CurveScene(None,d,'behavior','')
        box=self.carla.BoundingBox(self.carla.Location(),self.carla.Vector3D(x=2.6,y=1.3,z=1.2))
        s.target=SimpleNamespace(bounding_box=box,get_transform=lambda:self.carla.Transform())
        s.check_crossing_space(transform(s.crossing[0],s.geometry['crossing_yaw']),
                               transform(s.crossing[1],s.geometry['crossing_yaw']))

    def test_stationary_timeout_is_enabled_only_for_tcp(self):
        from curve_emergency_5e_runtime import CurveScene
        d=make_definitions(self.b,self.cfg)[0]
        self.assertEqual(CurveScene(None,d,'tcp','').trial.stationary_timeout_s,30.)
        self.assertIsNone(CurveScene(None,d,'behavior','').trial.stationary_timeout_s)

    def test_tcp_brake_is_preserved_without_reference_override(self):
        from curve_emergency_5e_runtime import CurveScene
        d=make_definitions(self.b,self.cfg)[0]
        s=CurveScene(None,d,'tcp','');s.ego=object()
        s.camera=SimpleNamespace(wait=lambda *args:object())
        s.tcp=SimpleNamespace(get_action=lambda *args:[[0.,.1,.8]])
        with patch.dict(sys.modules,torch=SimpleNamespace(inference_mode=nullcontext)):
            control=s.ego_control(1,.05,{})
        self.assertAlmostEqual(control.brake,.8,places=6)
        self.assertEqual(control.throttle,0.)

    def test_tcp_mixed_throttle_and_brake_reaches_vehicle_unchanged(self):
        from curve_emergency_5e_runtime import CurveScene
        d=make_definitions(self.b,self.cfg)[0]
        s=CurveScene(None,d,'tcp','');s.ego=object()
        s.camera=SimpleNamespace(wait=lambda *args:object())
        s.tcp=SimpleNamespace(get_action=lambda *args:[[.525,.1,.3]])
        with patch.dict(sys.modules,torch=SimpleNamespace(inference_mode=nullcontext)):
            control=s.ego_control(1,.05,{})
        self.assertAlmostEqual(control.throttle,.525,places=6)
        self.assertAlmostEqual(control.brake,.3,places=6)

    def test_video_lookahead_clamps_to_route_end(self):
        import numpy as np
        from curve_emergency_5e_video import CurveVideo
        video=CurveVideo.__new__(CurveVideo)
        drawn=[];p=self.b['ego_route'][-2]
        video.viz=SimpleNamespace(ego=SimpleNamespace(get_location=lambda:SimpleNamespace(**p)),
                                  render=lambda points:drawn.extend(points),screen=object())
        video.pygame=SimpleNamespace(event=SimpleNamespace(pump=lambda:None),
            surfarray=SimpleNamespace(array3d=lambda _:np.zeros((512,256,3),dtype=np.uint8)))
        frame=video.frame(None,self.b['ego_route'])
        self.assertEqual(frame.shape,(256,512,3))
        self.assertEqual(drawn[-1],(self.b['ego_route'][-1]['x'],self.b['ego_route'][-1]['y']))


class ContinuationSelectionTests(unittest.TestCase):
    def setUp(self):
        cfg,b=fixture();self.definitions=make_definitions(b,cfg)[:4]
        self.checkpoint='same-tcp-checkpoint'

    def record(self,index,status):
        d=self.definitions[index]
        return dict(scenario=d['scenario'],condition_fingerprint=d['condition_fingerprint'],
                    controller='tcp',checkpoint_sha256=self.checkpoint,result_status=status)

    def select(self,records,**metadata):
        from curve_emergency_5e_cli import exclude_completed_from_source
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'5e_result.json'
            meta=dict(status='interrupted',controller='tcp',runtime_sha256='historical-runtime')
            meta.update(metadata)
            path.write_text(json.dumps(dict(metadata=meta,records=records)))
            return exclude_completed_from_source(self.definitions,path,'tcp',self.checkpoint)

    def test_preserves_valid_old_results_but_selects_latest_invalid_and_unattempted(self):
        records=[self.record(0,'PASS'),self.record(1,'FAILED'),self.record(2,'PASS'),self.record(2,'INVALID')]
        selected,provenance=self.select(records)
        self.assertEqual([d['scenario'] for d in selected],[d['scenario'] for d in self.definitions[2:]])
        self.assertEqual(provenance['retained_valid_conditions'],2)
        self.assertEqual(provenance['source_invalid_conditions'],1)
        self.assertEqual(provenance['source_runtime_sha256'],'historical-runtime')
        self.assertEqual(len(provenance['source_sha256']),64)

    def test_rejects_active_or_incompatible_source(self):
        record=self.record(0,'PASS')
        with self.assertRaisesRegex(ValueError,'still running'):
            self.select([record],status='running')
        with self.assertRaisesRegex(ValueError,'controller'):
            self.select([record],controller='behavior')
        for key in ('condition_fingerprint','checkpoint_sha256','controller'):
            with self.subTest(field=key),self.assertRaises(ValueError):
                self.select([dict(record,**{key:'different'})])


class RunnerLifecycleTests(unittest.TestCase):
    def test_matching_source_build_is_accepted_but_mismatch_is_rejected(self):
        from curve_emergency_5e_cli import validate_carla_version
        client=SimpleNamespace(get_client_version=lambda:'294096eb1-dirty',get_server_version=lambda:'294096eb1-dirty')
        with patch('importlib.metadata.version',return_value='0.9.16'):
            validate_carla_version(client)
            client.get_server_version=lambda:'another-build'
            with self.assertRaisesRegex(ValueError,'matching'):validate_carla_version(client)

    def test_invalid_attempt_restores_world_and_resume_retries(self):
        try:
            import curve_emergency_5e_runtime as runtime
        except ImportError:
            self.skipTest('runtime dependencies unavailable')
        from curve_emergency_5e_cli import run_connected
        cfg,b=fixture();d=make_definitions(b,cfg)[0]
        class World:
            def __init__(self):self.settings=[];self.weather=[]
            def get_map(self):return SimpleNamespace(name=cfg['town'])
            def get_settings(self):return SimpleNamespace(synchronous_mode=False,fixed_delta_seconds=None)
            def get_weather(self):return 'original-weather'
            def apply_settings(self,s):self.settings.append(s)
            def set_weather(self,w):self.weather.append(w)
            def get_actors(self):return SimpleNamespace(filter=lambda _:[])
            def get_snapshot(self):return SimpleNamespace(frame=1,timestamp=SimpleNamespace(elapsed_seconds=.05))
        world=World();created=[]
        class Scene:
            def __init__(self,*args):self.trial=CurveTrial(d);created.append(self)
            def spawn(self,**kwargs):raise RuntimeError('missing asset')
            def finish(self):return self.trial.result()
            def destroy(self):self.destroyed=True
        client=SimpleNamespace(set_timeout=lambda _:None,get_world=lambda:world,
                               get_server_version=lambda:'0.9.16',get_client_version=lambda:'0.9.16')
        fake_carla=SimpleNamespace(Client=lambda *args:client,WeatherParameters=lambda **kwargs:kwargs)
        with tempfile.TemporaryDirectory() as tmp:
            args=SimpleNamespace(host='unit-test-'+Path(tmp).name,port=None,town=None,model='behavior',model_path=None,
                                 resume=False,output_dir=tmp,video_dir=None,no_video=True,headless=True,continue_invalid=False)
            with patch.dict(sys.modules,carla=fake_carla),patch.object(runtime,'CurveScene',Scene),patch('curve_emergency_5e_cli.validate_world'):
                with ExitStack() as stack:run_connected(args,cfg,[d],stack)
                self.assertTrue(created[0].destroyed)
                self.assertEqual(world.weather[-1],'original-weather')
                self.assertFalse(world.settings[-1].synchronous_mode)
                ledger=json.loads((Path(tmp)/'5e_result.json').read_text())
                self.assertEqual(ledger['summary']['INVALID'],1)
                self.assertEqual(ledger['summary']['remaining'],1)
                self.assertTrue(Path(ledger['records'][0]['detail_path']).is_file())
                args.resume=True
                with ExitStack() as stack:run_connected(args,cfg,[d],stack)
                self.assertEqual(len(created),2)


if __name__=='__main__':
    unittest.main()
