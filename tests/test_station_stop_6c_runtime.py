"""Runtime contracts using real CARLA value types but no simulator connection."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import carla

from test_station_stop_6c import definition
from station_stop_6c import geometry_metrics
from station_stop_6c_runtime import (StationStopScene, SUTFailure, read_vehicle_geometry,
                                     vehicle_camera_transform)


def scene_fixture(model='behavior', mode='hybrid'):
    d = definition()
    snap = SimpleNamespace(frame=101, timestamp=SimpleNamespace(elapsed_seconds=.05))
    world = SimpleNamespace(get_snapshot=lambda: snap)
    scene = StationStopScene(world, d, model, 'unused', mode)
    p = d['ego_start']
    actor = Mock()
    actor.get_transform.return_value = carla.Transform(carla.Location(x=p['x'], y=p['y'], z=p['z']))
    actor.get_velocity.return_value = carla.Vector3D(x=2.)
    actor.bounding_box = carla.BoundingBox(carla.Location(z=.8), carla.Vector3D(x=2.4, y=.9, z=.8))
    scene.ego = actor
    scene.handoff_frame = 100
    scene.initialization = {'handoff_frame': 100}
    scene.trial.start(0, 100)
    scene.camera = Mock()
    scene.tcp = Mock()
    scene.tcp.get_action.return_value = [[.4, .2, .6]]
    return scene, snap


class VehicleGeometryTests(unittest.TestCase):
    def test_sprinter_uses_measured_axles_steering_and_clear_camera_mount(self):
        actor = Mock()
        actor.type_id = 'vehicle.mercedes.sprinter'
        actor.bounding_box = carla.BoundingBox(carla.Location(z=1.3),
                                               carla.Vector3D(x=3., y=1., z=1.3))
        actor.get_physics_control.return_value.wheels = [
            SimpleNamespace(position=carla.Vector3D(x=x, y=y), max_steer_angle=angle)
            for x, y, angle in ((1000., 2000., 70.), (1000., 2170., 70.),
                                (640., 2000., 0.), (640., 2170., 0.))]
        geometry = read_vehicle_geometry(actor)
        self.assertAlmostEqual(geometry['wheelbase_m'], 3.6)
        self.assertAlmostEqual(geometry['max_steer_angle_deg'], 70.)
        for camera in ('tcp', 'video'):
            t = vehicle_camera_transform(dict(vehicle_geometry=geometry), camera, carla.Transform())
            self.assertGreater(t.location.x, 3.)
            self.assertAlmostEqual(t.location.z, 2.015, places=5)
        self.assertEqual(vehicle_camera_transform(dict(vehicle_geometry={}), 'video',
                                                  carla.Transform()).location.x, 0.)


class RuntimeContracts(unittest.TestCase):
    def test_station_reference_line_is_not_runtime_boundary_after_pn(self):
        scene, _ = scene_fixture()
        scene.b['station_edge'] = [dict(x=98.4, y=2., z=0.), dict(x=101.6, y=2., z=0.)]
        scene.b['geometry'] = geometry_metrics(scene.b, scene.eng)
        for y in (.8, 1.5, 2.):
            scene.ego.get_transform.return_value = carla.Transform(carla.Location(x=100., y=y, z=0.))
            self.assertFalse(scene.observation()[3])

    def test_approach_corridor_still_applies_before_pn(self):
        scene, _ = scene_fixture()
        scene.ego.get_transform.return_value = carla.Transform(carla.Location(x=50., y=5., z=0.))
        self.assertTrue(scene.observation()[3])

    def test_raw_tcp_brake_and_throttle_preserved(self):
        scene, snap = scene_fixture('tcp', 'raw_goal')
        self.assertTrue(scene.step(snap))
        control = scene.ego.apply_control.call_args.args[0]
        self.assertAlmostEqual(control.throttle, .4, places=6)
        self.assertAlmostEqual(control.brake, .6, places=6)
        self.assertEqual(scene.controls[0]['source'], 'tcp')
        self.assertEqual(scene.controls[0]['applies_to_frame'], 102)

    def test_tcp_exception_is_sut_failure_without_reference_fallback(self):
        scene, snap = scene_fixture('tcp', 'raw_goal')
        scene.tcp.get_action.side_effect = RuntimeError('inference crashed')
        with patch('station_stop_6c_runtime.reference_control') as reference:
            with self.assertRaises(SUTFailure): scene.step(snap)
            reference.assert_not_called()
        scene.ego.apply_control.assert_not_called()

    def test_missing_camera_is_platform_error_before_policy_call(self):
        scene, snap = scene_fixture('tcp', 'raw_goal')
        scene.camera.wait.side_effect = RuntimeError('camera timeout')
        with self.assertRaises(RuntimeError) as caught: scene.step(snap)
        self.assertNotIsInstance(caught.exception, SUTFailure)
        scene.tcp.get_action.assert_not_called()

    def test_hybrid_handoff_is_latched_and_logged(self):
        scene, snap = scene_fixture('tcp', 'hybrid')
        scene.ego.get_transform.return_value = carla.Transform(carla.Location(x=80.1, y=0, z=0))
        scene.step(snap)
        self.assertTrue(scene.docking)
        scene.tcp.get_action.assert_not_called()
        self.assertEqual(scene.controls[-1]['source'], 'reference_parking')
        self.assertTrue(any(e['event'] == 'TCP_TO_REFERENCE_PARKING' for e in scene.trial.events))

    def test_observer_failure_does_not_apply_emergency_control_as_model_output(self):
        scene, snap = scene_fixture()
        scene.collisions.append(dict(frame=101, sim_time=.05))
        self.assertFalse(scene.step(snap))
        self.assertEqual(scene.trial.status, 'FAILED')
        scene.ego.apply_control.assert_not_called()

    def test_late_collision_after_provisional_pass_is_recorded(self):
        scene, snap = scene_fixture()
        scene.trial.end('PASS', 'first_stop_satisfied')
        scene.terminal_frame = 105
        scene.collisions.append(dict(frame=104, sim_time=.2))
        with patch('station_stop_6c_runtime.time.sleep'):
            result = scene.finish()
        self.assertEqual(result['result_status'], 'FAILED')
        self.assertEqual(result['reasons'], ['collision'])

    def test_late_solid_line_event_is_bounded_by_terminal_frame(self):
        for frame, expected in ((105, 'FAILED'), (106, 'PASS')):
            with self.subTest(frame=frame):
                scene, _ = scene_fixture()
                scene.trial.end('PASS', 'first_stop_satisfied')
                scene.terminal_frame = 105
                scene.lanes.append(dict(frame=frame, sim_time=.25, markings=['Solid']))
                with patch('station_stop_6c_runtime.time.sleep'):
                    result = scene.finish()
                self.assertEqual(result['result_status'], expected)
                if expected == 'FAILED':
                    self.assertEqual(result['reasons'], ['crossed_solid_line'])

    def test_missing_final_evidence_can_revoke_provisional_pass(self):
        scene, _ = scene_fixture()
        scene.trial.end('PASS', 'first_stop_satisfied')
        scene.trial.end('INVALID', 'missing_terminal_sensor_frame')
        self.assertEqual(scene.trial.status, 'INVALID')

    def test_simulation_time_jump_is_invalid_even_with_consecutive_frames(self):
        scene, snap = scene_fixture()
        scene.step(snap)
        snap.frame += 1; snap.timestamp.elapsed_seconds += 1.
        scene.step(snap)
        self.assertEqual(scene.trial.status, 'INVALID')

    def test_late_initialization_collision_invalidates_setup(self):
        scene, _ = scene_fixture()
        scene.trial.end('PASS', 'first_stop_satisfied')
        scene.terminal_frame = 105
        scene.collisions.append(dict(frame=100, sim_time=0.))
        with patch('station_stop_6c_runtime.time.sleep'):
            result = scene.finish()
        self.assertEqual(result['result_status'], 'INVALID')
        self.assertEqual(result['reasons'], ['late_initialization_collision'])

    def test_invalid_initialization_cannot_be_counted_as_valid_failure(self):
        scene, _ = scene_fixture()
        scene.trial.end('FAILED', 'outside_allowed_area')
        scene.terminal_frame = 105
        scene.collisions.extend([dict(frame=100, sim_time=0.), dict(frame=104, sim_time=.2)])
        with patch('station_stop_6c_runtime.time.sleep'):
            result = scene.finish()
        self.assertEqual(result['result_status'], 'INVALID')
        self.assertEqual(result['reasons'], ['late_initialization_collision'])
        self.assertFalse(result['trial_valid'])

    def test_no_terminal_observation_cannot_score_unbounded_late_events(self):
        scene, _ = scene_fixture()
        scene.trial.end('INVALID', 'observer_failed')
        scene.collisions.append(dict(frame=106, sim_time=.3))
        with patch('station_stop_6c_runtime.time.sleep'):
            result = scene.finish()
        self.assertEqual(result['result_status'], 'INVALID')

    def test_collision_beyond_terminal_frame_is_not_retroactive(self):
        scene, _ = scene_fixture()
        scene.trial.end('PASS', 'first_stop_satisfied')
        scene.terminal_frame = 105
        scene.collisions.append(dict(frame=106, sim_time=.3))
        with patch('station_stop_6c_runtime.time.sleep'):
            result = scene.finish()
        self.assertEqual(result['result_status'], 'PASS')

    def test_goal_coordinate_is_not_projected_during_docking(self):
        scene, _ = scene_fixture()
        self.assertEqual(scene.path[-1]['y'], .8)
        self.assertEqual(scene.b['stop_pose']['y'], .8)


if __name__ == '__main__':
    unittest.main()
