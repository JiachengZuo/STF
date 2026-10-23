"""The evaluated policy must not consume the experiment's hidden answers."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from cross_right_2d import CrossRightTrial
from cross_right_2d_behavior import VehicleWithSectionLimit
from test_cross_right_2d import definition, fake_runtime, observation


class ForbiddenState:
    def __getattr__(self, key):
        raise AssertionError('controller read hidden scenario state: '+key)


class BehaviorIsolationTests(unittest.TestCase):
    def test_formal_action_is_agent_output_without_scenario_or_target_access(self):
        runtime = fake_runtime()
        scene = runtime.CrossRightScene(None, definition())
        action = runtime.carla.VehicleControl(throttle=.31, brake=.07, steer=-.23)
        scene.behavior = SimpleNamespace(run_step=Mock(return_value=action))
        scene.trial = scene.target = scene.d = scene.eng = ForbiddenState()
        scene.cruise_feedforward = Mock(side_effect=AssertionError('runtime changed algorithm control'))
        self.assertIs(scene.ego_control(100, 5., ForbiddenState()), action)
        scene.behavior.run_step.assert_called_once_with()
        scene.cruise_feedforward.assert_not_called()

    def test_release_and_clearance_flags_do_not_override_agent(self):
        runtime = fake_runtime()
        scene = runtime.CrossRightScene(None, definition())
        action = runtime.carla.VehicleControl(throttle=.2)
        scene.behavior = SimpleNamespace(run_step=lambda: action)
        for released, clear, gate in ((False, None, False), (True, None, False),
                                      (True, 8., False), (True, 8., True)):
            scene.trial = SimpleNamespace(released=released, t_clear=clear, gate_crossed=gate)
            self.assertIs(scene.ego_control(100, 5., {}), action)

    def test_section_limit_view_preserves_live_vehicle_state(self):
        ego = SimpleNamespace(id=7, get_speed_limit=lambda: 30., get_velocity=Mock(return_value='live'))
        view = VehicleWithSectionLimit(ego, 40.)
        self.assertEqual(view.get_speed_limit(), 40.)
        self.assertEqual(view.id, 7)
        self.assertEqual(view.get_velocity(), 'live')

    def test_old_script_pass_cannot_skip_a_new_algorithm_trial(self):
        from cross_right_2d_cli import exclude_completed_from_source
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'old_result.json'
            path.write_text(json.dumps(dict(metadata=dict(status='completed', controller='behavior'), records=[])))
            with self.assertRaisesRegex(ValueError, 'legacy scripted behavior'):
                exclude_completed_from_source([definition()], path, 'behavior', None)

    def test_collision_is_failed_even_before_release_without_synthetic_exit_failure(self):
        for model in ('behavior', 'tcp'):
            t = CrossRightTrial(definition(20), model)
            obs = observation(x=-80., v=20/3.6, frame=10)
            obs.update(collision_target=True, collision_target_frame=10)
            t.update(0., obs)
            result = t.result()
            self.assertTrue(t.done)
            self.assertEqual(result['primary_reason'], 'collision_with_target')
            self.assertEqual(result['result_status'], 'FAILED')
            self.assertNotIn('corresponding_lane_not_entered', result['failure_reasons'])

    def test_collision_induced_target_motion_does_not_invalidate_trial(self):
        for model in ('behavior', 'tcp'):
            t = CrossRightTrial(definition(20), model)
            t.update(0., observation(x=-30., v=20/3.6, frame=1))
            obs = observation(x=-5., v=20/3.6, target_y=2., target_x=2., target_v=0., frame=50)
            obs.update(collision_target=True, collision_target_frame=50)
            t.update(2.5, obs)
            self.assertTrue(t.done)
            self.assertEqual(t.result()['result_status'], 'FAILED')
            self.assertEqual(t.latches, [])
            self.assertEqual(t.collision_frame, 50)
            self.assertIn('target_speed_not_held', t.target_motion_fault_frames)

    def test_late_collision_callback_only_removes_faults_at_or_after_contact(self):
        for motion_frame, expected in ((49, 'INVALID'), (50, 'FAILED'), (51, 'FAILED')):
            t = CrossRightTrial(definition(20), 'behavior')
            t.update(0., observation(x=-30., v=20/3.6, frame=1))
            t.update(2.5, observation(x=-10., v=20/3.6, target_y=2., target_v=0., frame=motion_frame))
            self.assertIn('target_speed_not_held', t.latches)
            t.add_events(True, collision_frame=50)
            self.assertEqual(t.result()['result_status'], expected)

    def test_two_kmh_window_boundaries_are_identical_for_both_algorithms(self):
        for model in ('behavior', 'tcp'):
            for actual, accepted in ((18., True), (22., True), (17.99, False), (22.01, False)):
                with self.subTest(model=model, actual=actual):
                    t = CrossRightTrial(definition(20), model)
                    t.update(0., observation(x=-5.4*actual/3.6, v=actual/3.6))
                    result = t.result()
                    self.assertEqual(result['speed_requirement_met'], accepted)
                    self.assertEqual(result['speed_requirement_evidence']['tolerance_kmh'], 2.)
                    self.assertEqual('required_speed_not_reached' in result['failure_reasons'], not accepted)

    def test_same_failed_speed_and_unsafe_crossing_score_for_both_algorithms(self):
        for model in ('behavior', 'tcp'):
            with self.subTest(model=model):
                trial = CrossRightTrial(definition(20), model)
                trial.update(0., observation(x=-25., v=17/3.6))
                result = trial.result()
                self.assertEqual((result['result_status'], result['primary_reason']),
                                 ('FAILED', 'required_speed_not_reached'))
                self.assertTrue(trial.done)
                self.assertEqual(result['controller_role'], 'system_under_test')
                trial = CrossRightTrial(definition(20), model)
                trial.update(0., observation(x=-30., v=20/3.6))
                trial.update(5., observation(x=-2., v=20/3.6, target_y=1.))
                trial.update(9., observation(x=18., v=20/3.6, target_y=-10., target_v=20/3.6))
                self.assertEqual(trial.result()['result_status'], 'FAILED')
                self.assertIn('yield_not_established', trial.result()['failure_reasons'])


if __name__ == '__main__':
    unittest.main()
