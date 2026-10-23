import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

from tests.test_tunnel_runtime import runtime_scene, _Location, _ParentScene


def tcp_scene():
    scene = runtime_scene()
    scene.model = "tcp"
    scene.tcp_flag = True
    scene.tcp = SimpleNamespace(get_action=mock.Mock(return_value=[[0.7, 0.2, 0.9]]), load_model=mock.Mock())
    scene.control = SimpleNamespace()
    scene.ego.apply_control = mock.Mock()
    scene.follow_route = mock.Mock(side_effect=AssertionError("Reference fallback is forbidden"))
    scene._tcp_frame_condition = threading.Condition()
    scene._tcp_frames = {}
    scene._tcp_last_frame = None
    scene._tcp_error = None
    scene._tcp_control_samples = []
    scene.tcp_frame_timeout_s = 0.02
    scene.route_points = [_Location(-10), _Location(0), _Location(100), _Location(115)]
    scene.current_target_idx = 0
    scene.stop_distance = 2
    snapshot = SimpleNamespace(frame=10, timestamp=SimpleNamespace(elapsed_seconds=1.0))
    scene.world = SimpleNamespace(get_snapshot=lambda: snapshot)
    return scene, snapshot


class TunnelTCPTest(unittest.TestCase):
    def test_exact_frame_waits_for_delayed_arrival_and_applies_brake(self):
        scene, snapshot = tcp_scene()
        scene.tcp_frame_timeout_s = 1.0
        old, future, correct = object(), object(), object()
        scene._tcp_frames = {9: (0.95, old), 11: (1.05, future)}

        def arrive():
            time.sleep(0.02)
            with scene._tcp_frame_condition:
                scene._tcp_frames[10] = (1.0, correct)
                scene._tcp_frame_condition.notify_all()

        thread = threading.Thread(target=arrive)
        thread.start()
        with mock.patch.object(_ParentScene, "tick", side_effect=AssertionError("Generic TCP tick must not run")):
            self.assertTrue(scene.tick())
        thread.join()
        self.assertIs(scene.tcp.get_action.call_args.args[0], correct)
        self.assertEqual(scene.control.brake, 0.9)
        self.assertEqual(scene.control.throttle, 0)
        self.assertEqual(scene.control.steer, 0.2)
        self.assertEqual(scene._tcp_control_samples[0]["world_frame"], 10)
        self.assertEqual(scene._tcp_control_samples[0]["camera_frame"], 10)
        scene.follow_route.assert_not_called()

    def test_missing_stale_and_future_only_frames_are_invalid_without_fallback(self):
        for frames in ({}, {9: (0.95, object())}, {11: (1.05, object())}):
            with self.subTest(frames=list(frames)):
                scene, _ = tcp_scene()
                scene._tcp_frames = frames
                self.assertFalse(scene.tick())
                result = scene.get_result()
                self.assertFalse(result["trial_valid"])
                self.assertIsNone(result["pass"])
                self.assertIn("camera_frame_timeout", result["controller_error"])
                self.assertEqual(scene.ego.apply_control.call_args.args[0].brake, 1)
                scene.tcp.get_action.assert_not_called()
                scene.follow_route.assert_not_called()

    def test_inference_failure_and_invalid_actions_stop_with_tcp_identity(self):
        for action in (None, [[float("nan"), 0, 0]], [[0.1, 0.2]]):
            scene, _ = tcp_scene()
            scene._tcp_frames[10] = (1, object())
            if action is None:
                scene.tcp.get_action.side_effect = RuntimeError("injected model failure")
            else:
                scene.tcp.get_action.return_value = action
            self.assertFalse(scene.tick())
            result = scene.get_result()
            self.assertEqual(result["controller"], "tcp")
            self.assertFalse(result["trial_valid"])
            self.assertFalse(result["controller_fallback_used"])
            self.assertIsNone(result["pass"])
            scene.follow_route.assert_not_called()

    def test_duplicate_frame_cannot_be_inferred_twice(self):
        scene, _ = tcp_scene()
        scene._tcp_frames[10] = (1, object())
        self.assertTrue(scene.tick())
        self.assertFalse(scene.tick())
        self.assertEqual(scene.tcp.get_action.call_count, 1)
        self.assertIn("duplicate_or_reversed", scene._tcp_error)

    def test_matching_frame_with_wrong_timestamp_is_rejected(self):
        scene, _ = tcp_scene()
        scene._tcp_frames[10] = (0.9, object())
        self.assertFalse(scene.tick())
        self.assertIn("timestamp_mismatch", scene._tcp_error)
        scene.tcp.get_action.assert_not_called()

    def test_world_must_not_advance_during_inference(self):
        scene, snapshot = tcp_scene()
        scene._tcp_frames[10] = (1, object())

        def advance(*args):
            snapshot.frame = 11
            return [[0.8, 0, 0]]

        scene.tcp.get_action.side_effect = advance
        self.assertFalse(scene.tick())
        self.assertIn("world_advanced_during", scene._tcp_error)
        self.assertEqual(scene.ego.apply_control.call_args.args[0].brake, 1)
        self.assertEqual(scene._tcp_control_samples, [])

    def test_tcp_completes_at_final_point_after_gates_without_reference_control(self):
        scene, snapshot = tcp_scene()
        scene.ego.location = _Location(115)
        scene.tunnel_entry_sim_time, scene.tunnel_exit_sim_time = 0.2, 0.9
        self.assertFalse(scene._tick_tcp(snapshot))
        self.assertTrue(scene.route_complete)
        self.assertTrue(scene.finished)
        self.assertTrue(scene.get_result()["pass"])
        scene.tcp.get_action.assert_not_called()
        scene.follow_route.assert_not_called()

    def test_near_finish_without_exit_gate_is_not_complete(self):
        scene, snapshot = tcp_scene()
        scene.ego.location = _Location(115)
        scene._tcp_frames[10] = (1, object())
        self.assertTrue(scene._tick_tcp(snapshot))
        self.assertFalse(scene.route_complete)

    def test_startup_loads_strict_weights_and_holds_before_starting_trial(self):
        scene, _ = tcp_scene()
        scene.world.get_settings = lambda: SimpleNamespace(synchronous_mode=True)
        scene.world.tick = mock.Mock()
        scene._validate_map_identity = mock.Mock()
        scene._prepare_tunnel_geometry = mock.Mock()
        scene._spawn_lane_invasion_sensor = mock.Mock()
        scene._wait_tcp_image = mock.Mock(return_value=object())
        scene.spawn()
        scene.tcp.load_model.assert_called_once_with(strict=True)
        scene.tcp.get_action.assert_not_called()
        self.assertEqual(scene.tunnel_start_sim_time, 1)
        self.assertEqual(scene.tunnel_phase, "APPROACH")
        self.assertGreaterEqual(scene.ego.apply_control.call_count, 2)


if __name__ == "__main__":
    unittest.main()
