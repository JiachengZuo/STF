"""Moving-state preparation is shared, measured and outside the trial clock."""
import copy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from cross_right_2d import at
from test_cross_right_2d import definition, fake_runtime, observation
from test_cross_right_2d_routes import Location


class Waypoint:
    def __init__(self,x):
        self.s=x+1000.;self.road_id=1;self.section_id=0;self.lane_id=-1
        self.lane_width=3.5;self.is_junction=False
        self.transform=SimpleNamespace(location=Location(x=x),rotation=SimpleNamespace(yaw=0.))
    def previous(self,distance):return [Waypoint(self.transform.location.x-distance)]


class InitializationTests(unittest.TestCase):
    def scene(self,model='tcp',offset=0.,gap=False):
        runtime=fake_runtime()
        state=dict(frame=0,enabled=False)
        ego=SimpleNamespace(id=1,speed=0.,position=dict(x=-100.,y=0.,z=0.),apply_control=Mock(),set_target_velocity=Mock())
        ego.get_control=lambda:SimpleNamespace(gear=1,manual_gear_shift=False)
        ego.get_transform=lambda:SimpleNamespace(get_forward_vector=lambda:SimpleNamespace(x=1.,y=0.,z=0.))
        def enable(v):state.update(enabled=True,command=v.x)
        def disable():state['enabled']=False
        ego.enable_constant_velocity=Mock(side_effect=enable)
        ego.disable_constant_velocity=Mock(side_effect=disable)
        def tick():
            state['frame']+=2 if gap and state['frame']==2 else 1
            if state['enabled']:ego.speed=state['command']+offset/3.6
            ego.position['x']+=ego.speed*.05
        world=SimpleNamespace(tick=Mock(side_effect=tick),get_snapshot=lambda:SimpleNamespace(frame=state['frame'],timestamp=SimpleNamespace(elapsed_seconds=state['frame']*.05)))
        scene=runtime.CrossRightScene(world,definition(),model=model)
        scene.ego=ego
        scene.target=SimpleNamespace(id=2,apply_control=Mock(),enable_constant_velocity=Mock(),disable_constant_velocity=Mock())
        scene.initialization_plan=dict(handoff_gate_s=20.)
        return runtime,scene,state

    def test_both_models_receive_verified_same_speed_without_post_handoff_velocity(self):
        records=[]
        for model in ('behavior','tcp'):
            runtime,scene,state=self.scene(model)
            scene.handoff()
            self.assertTrue(scene.initialization['verified'])
            self.assertGreaterEqual(scene.initialization['held_s'],1.)
            self.assertLessEqual(abs(scene.initialization['actual_speed_kmh']-40),1.)
            self.assertFalse(state['enabled'])
            self.assertEqual(scene.ego.apply_control.call_args_list[0].args[0].gear,1)
            self.assertTrue(scene.ego.apply_control.call_args_list[0].args[0].manual_gear_shift)
            self.assertFalse(scene.ego.apply_control.call_args_list[-1].args[0].manual_gear_shift)
            self.assertFalse(scene.ego_velocity_assist)
            scene.ego.enable_constant_velocity.assert_called_once()
            scene.ego.disable_constant_velocity.assert_called_once()
            scene.ego.set_target_velocity.assert_not_called()
            self.assertIsNone(scene.trial.start_time)
            self.assertEqual(scene.telemetry,[])
            frame=state['frame'];time=frame*.05
            self.assertEqual(scene.initialization['frame'],frame)
            scene.observation=lambda f:observation(x=scene.ego.position['x'],v=scene.ego.speed,frame=f)
            scene.ego_control=Mock(return_value=runtime.carla.VehicleControl())
            scene.tcp=SimpleNamespace(last_debug={})
            scene.behavior=SimpleNamespace(last_debug={})
            scene.step()
            self.assertEqual(state['frame'],frame)
            self.assertEqual(scene.trial.start_time,time)
            self.assertEqual(scene.telemetry[0]['frame'],frame)
            self.assertEqual(scene.trial.result()['elapsed_sim_time_s'],0.)
            self.assertFalse(state['enabled'])
            scene.ego.enable_constant_velocity.assert_called_once()
            records.append((scene.initialization['actual_speed_kmh'],scene.initialization['pose']))
        self.assertEqual(records[0],records[1])

    def test_unmet_initial_speed_is_setup_invalid_and_assistance_is_disabled(self):
        _,scene,state=self.scene(offset=3.)
        with self.assertRaisesRegex(RuntimeError,'initialization_speed_not_reached_before_window'):
            scene.handoff()
        self.assertFalse(state['enabled'])
        self.assertFalse(scene.initialization['verified'])
        scene.trial.latch('initialization_speed_not_reached_before_window',immediate=True)
        result=scene.trial.result()
        self.assertEqual(result['result_status'],'INVALID')
        self.assertIsNone(result['speed_requirement_met'])
        self.assertEqual(result['failure_reasons'],[])

    def test_preparation_frame_gap_is_not_scored_as_model_failure(self):
        _,scene,state=self.scene(gap=True)
        with self.assertRaisesRegex(RuntimeError,'initialization_frame_discontinuity'):
            scene.handoff()
        self.assertFalse(state['enabled'])
        self.assertIsNone(scene.trial.start_time)

    def test_feedforward_balances_motor_drag_and_grade(self):
        _,scene,_=self.scene('behavior')
        p=SimpleNamespace(mass=1845., final_ratio=9.,
            wheels=[SimpleNamespace(radius=37.,max_brake_torque=700.) for _ in range(4)],
            forward_gears=[SimpleNamespace(ratio=1.)],
            torque_curve=[SimpleNamespace(x=0.,y=743.),SimpleNamespace(x=9000.,y=743.)],
            damping_rate_zero_throttle_clutch_engaged=2.,damping_rate_full_throttle=.15)
        scene.drive_physics=p
        scene.reference_brake_accel=4*700/(.37*1845)
        for v in (5/3.6,15/3.6,40/3.6):
            for z in (-.08,0.,.08):
                scene.ego.get_transform=lambda:SimpleNamespace(get_forward_vector=lambda:SimpleNamespace(z=z))
                throttle=scene.cruise_feedforward(v)
                omega=v/.37*9
                damping=2*(1-throttle)+.15*throttle
                acceleration=(743*throttle-damping*omega)*9/(.37*1845)-9.81*z
                self.assertAlmostEqual(acceleration,0.,places=8)
                self.assertGreaterEqual(throttle,0.)
                self.assertLess(throttle,.6)

    def test_preparation_lane_events_do_not_enter_formal_failure_list(self):
        _,scene,_=self.scene()
        scene.trial_start_frame=100
        scene.lane_events=[dict(frame=99,markings=['Solid'])]
        self.assertNotIn('solid_line',scene.event_failures(100)[1])
        scene.lane_events.append(dict(frame=100,markings=['Solid']))
        self.assertIn('solid_line',scene.event_failures(100)[1])

    def test_approach_geometry_is_common_to_models_and_speeds_without_editing_json(self):
        runtime=fake_runtime();runtime.carla.Location=Location
        world=SimpleNamespace(get_map=lambda:SimpleNamespace(get_waypoint=lambda p:Waypoint(p.x)))
        routes=[]
        for model,speed in (('behavior',5),('tcp',5),('behavior',40),('tcp',40)):
            d=definition(speed);original=copy.deepcopy(d)
            scene=runtime.CrossRightScene(world,d,model=model)
            scene.prepare_approach()
            self.assertEqual(d,original)
            self.assertGreater(scene.initialization_plan['preparation_added_m'],0.)
            self.assertEqual(scene.b['geometry']['conflict_point'],d['baseline']['geometry']['conflict_point'])
            p=at(scene.route,scene.b['geometry']['ego_conflict_s'])
            self.assertAlmostEqual(p['x'],0.,places=5)
            self.assertAlmostEqual(p['y'],0.,places=5)
            self.assertEqual(scene.route[-len(d['baseline']['ego_route']):],d['baseline']['ego_route'])
            self.assertEqual(scene.trial.b,scene.b)
            routes.append((scene.route,scene.initialization_plan))
        self.assertTrue(all(r==routes[0] for r in routes))


if __name__=='__main__':unittest.main()
