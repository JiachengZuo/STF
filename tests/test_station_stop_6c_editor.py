"""Slow native RPCs must not freeze 6c input, rendering, cancellation or quit."""
import hashlib
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import carla
import pygame

from station_stop_6c_editor import StationStopEditor
from station_stop_6c_ground import GroundQuery, query_ground
from test_station_stop_6c import StraightApproachMap


WORKER = '''import ctypes, json, sys, time
r = json.loads(sys.argv[1])
if r.get('block'):
    ctypes.PyDLL(None).sleep(20)
time.sleep(r.get('delay', .08))
print(json.dumps(dict(ok=True, point=dict(x=r['x'], y=r['y'], z=r['reference_z']+.1),
                     evidence=dict(source='test-ray'))), flush=True)
'''


class GroundProcessFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.worker = Path(self.temp.name) / 'worker.py'
        self.worker.write_text(WORKER)
        self.query = GroundQuery(.8, worker_path=self.worker)
        self.request = dict(mode='a', x=30., y=2., reference_z=.5, geometry_revision=0)

    def tearDown(self):
        self.query.close()
        self.temp.cleanup()

    def await_result(self):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            result = self.query.poll()
            if result is not None:
                return result
            time.sleep(.005)
        self.fail('ground process did not complete or time out')


class GroundProcessTests(GroundProcessFixture):
    def test_slow_success_is_nonblocking_and_preserves_click(self):
        self.query.start(dict(self.request, delay=.15))
        before = time.monotonic()
        self.assertIsNone(self.query.poll())
        self.assertLess(time.monotonic() - before, .2)
        result = self.await_result()
        self.assertTrue(result['ok'])
        self.assertEqual(result['point'], dict(x=30., y=2., z=.6))
        self.assertEqual(result['request']['mode'], 'a')

    def test_native_gil_holding_worker_has_a_hard_deadline(self):
        self.query.timeout_s = .12
        self.query.start(dict(self.request, block=True))
        started = time.monotonic()
        result = self.await_result()
        self.assertFalse(result['ok'])
        self.assertIn('超过', result['error'])
        self.assertLess(time.monotonic() - started, 1.)
        self.assertIsNone(self.query.pending)

    def test_cancel_discards_reply_and_reaps_worker(self):
        self.query.start(dict(self.request, block=True))
        process = self.query.pending['process']
        self.assertEqual(self.query.cancel()['mode'], 'a')
        self.query.close()
        self.assertIsNotNone(process.poll())
        self.assertIsNone(self.query.poll())

    def test_only_one_query_can_be_active(self):
        self.query.start(dict(self.request, block=True))
        with self.assertRaises(ValueError):
            self.query.start(dict(self.request, mode='b'))
        self.assertEqual(self.query.pending['request']['mode'], 'a')

    def test_invalid_worker_output_is_visible_failure(self):
        self.worker.write_text("print('not a ground response')\n")
        self.query.start(self.request)
        result = self.await_result()
        self.assertFalse(result['ok'])
        self.assertIn('有效结果', result['error'])


class EditorFeedbackTests(GroundProcessFixture):
    def setUp(self):
        super().setUp()
        os.environ['SDL_VIDEODRIVER'] = 'dummy'
        os.environ['SDL_AUDIODRIVER'] = 'dummy'
        pygame.init()
        e = self.editor = StationStopEditor.__new__(StationStopEditor)
        e.settings = dict(matrix=dict(initial_distances_m=[40, 50, 60, 70]))
        e.screen = pygame.display.set_mode((1200, 800))
        e.font, _ = e._load_editor_font(22)
        e.panel_font, _ = e._load_editor_font(16)
        e.clock = pygame.time.Clock()
        e.zoom, e.offset_x, e.offset_y = 5., 50., 400.
        e.dragging = False
        e.ego_point = dict(x=0., y=0., z=.5, yaw=0.)
        e.ego_route_points = []
        e.marks = dict(a=None, b=None, goal=None)
        e.ground_evidence = {}
        e.vehicle_geometry = e.preview_baseline = e.ground_feedback = None
        e.history = []
        e.geometry_revision = 0
        e.mode = 'a'
        e.message = ''
        e.ground_connection = {}
        e.ground_query = self.query
        e.world = Mock()
        e.world.cast_ray.side_effect = AssertionError('native RPC reached UI thread')
        e.draw_waypoints = Mock()

    def tearDown(self):
        super().tearDown()
        pygame.quit()

    def wait_editor(self):
        deadline = time.monotonic() + 3
        while self.query.pending is not None and time.monotonic() < deadline:
            self.editor.poll_ground_query()
            time.sleep(.005)
        self.assertIsNone(self.query.pending)

    def test_press_a_then_click_gives_immediate_placeholder_and_feedback(self):
        e = self.editor
        e.select_mode('a')
        self.assertIn('已选择 A', e.message)
        e.place((200, 390))
        self.assertIsNone(e.marks['a'])
        self.assertEqual(e.ground_feedback['state'], 'pending')
        self.assertIn('已接收 A', e.message)
        e.draw_6c()  # This must render while the subprocess is still waiting.
        e.world.cast_ray.assert_not_called()
        self.wait_editor()
        self.assertEqual(e.marks['a'], dict(x=30., y=2., z=.6))
        self.assertIn('已放置', e.message)

    def test_mode_change_during_query_does_not_redirect_a_to_b(self):
        e = self.editor
        e.place((200, 390)); e.select_mode('b')
        self.wait_editor()
        self.assertIsNotNone(e.marks['a'])
        self.assertIsNone(e.marks['b'])

    def test_timeout_keeps_original_points_and_draws_failed_marker(self):
        e = self.editor
        e.marks['a'] = dict(x=1., y=2., z=.5)
        e.ground_connection['block'] = True
        self.query.timeout_s = .08
        e.place((200, 390))
        self.wait_editor()
        self.assertEqual(e.marks['a']['x'], 1.)
        self.assertEqual(e.ground_feedback['state'], 'failed')
        self.assertIn('未放置', e.message)
        self.assertEqual(e.history, [])
        e.draw_6c()

    def test_undo_during_pending_query_cancels_without_removing_previous_work(self):
        e = self.editor
        e.ground_connection['block'] = True
        e.place((200, 390))
        e.undo()
        self.assertIsNone(self.query.pending)
        self.assertIsNone(e.marks['a'])
        self.assertIsNotNone(e.ego_point)
        e.poll_ground_query()
        self.assertIsNone(e.marks['a'])

    def test_stale_geometry_reply_cannot_commit(self):
        e = self.editor
        e.place((200, 390))
        e.geometry_revision += 1
        self.wait_editor()
        self.assertIsNone(e.marks['a'])
        self.assertIn('丢弃旧结果', e.message)

    def test_preview_and_save_reject_pending_query_before_any_rpc(self):
        e = self.editor
        e.ground_connection['block'] = True
        e.place((200, 390))
        with self.assertRaisesRegex(ValueError, '正在查询'):
            e.preview()
        with self.assertRaisesRegex(ValueError, '正在查询'):
            e.save()
        e.world.try_spawn_actor.assert_not_called()

    def test_bad_p_segment_is_rejected_before_it_enters_the_route(self):
        e = self.editor
        e.map = StraightApproachMap(gap=(25, 26))
        e.get_nearest_waypoint = lambda x, y: e.map.get_waypoint(
            carla.Location(x=x, y=y, z=0.), project_to_road=True,
            lane_type=carla.LaneType.Driving)
        e.ego_point = dict(x=0., y=0., z=0., yaw=0.)
        e.mode = 'route'
        with self.assertRaisesRegex(ValueError, r'E→P1.*25\.'):
            e.place((250, 400))  # x=40 m with this editor's screen transform
        self.assertEqual(e.ego_route_points, [])
        self.assertEqual(e.history, [])
        self.assertEqual(e.route_error['goal']['x'], 40.)
        e.draw_6c()

    def test_error_uses_the_actual_p2_to_p3_label(self):
        e = self.editor
        e.map = StraightApproachMap(gap=(55, 56))
        e.get_nearest_waypoint = lambda x, y: e.map.get_waypoint(
            carla.Location(x=x, y=y, z=0.), project_to_road=True,
            lane_type=carla.LaneType.Driving)
        e.ego_point = dict(x=0., y=0., z=0., yaw=0.)
        e.ego_route_points = [dict(x=30., y=0., z=0., yaw=0.),
                              dict(x=50., y=0., z=0., yaw=0.)]
        e.mode = 'route'
        with self.assertRaisesRegex(ValueError, r'P2→P3.*55\.'):
            e.place((350, 400))  # x=60 m
        self.assertEqual(len(e.ego_route_points), 2)

    def test_event_loop_handles_zoom_and_quit_while_rpc_is_stuck(self):
        e = self.editor
        e.ground_connection['block'] = True
        batches = [
            [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_a, mod=0),
             pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(200, 390))],
            [pygame.event.Event(pygame.MOUSEWHEEL, y=1)],
            [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_q, mod=0)],
        ]
        e._zoom_at = Mock()
        with patch('pygame.event.get', side_effect=batches), patch('pygame.quit'):
            e.run()
        e._zoom_at.assert_called_once()
        self.assertIsNone(self.query.pending)
        e.world.cast_ray.assert_not_called()


class RayWorkerTests(unittest.TestCase):
    def test_worker_is_read_only_and_checks_map_before_query(self):
        world = Mock()
        map_obj = Mock(); map_obj.name = 'station'; map_obj.to_opendrive.return_value = 'xodr'
        world.get_map.return_value = map_obj
        world.cast_ray.return_value = [SimpleNamespace(location=carla.Location(x=12, y=3, z=.2),
                                                       label=carla.CityObjectLabel.Roads)]
        client = Mock(); client.get_world.return_value = world
        request = dict(host='127.0.0.1', port=2000, map_name='station',
                       map_sha256=hashlib.sha256(b'xodr').hexdigest(),
                       x=12., y=3., reference_z=0., mode='a')
        with patch('carla.Client', return_value=client):
            result = query_ground(request)
            self.assertTrue(result['ok'])
            self.assertAlmostEqual(result['point']['z'], .2)
            request['map_name'] = 'different'
            with self.assertRaisesRegex(ValueError, '地图已变化'):
                query_ground(request)
        world.tick.assert_not_called()
        world.apply_settings.assert_not_called()
        self.assertEqual(world.cast_ray.call_count, 1)


if __name__ == '__main__':
    unittest.main()
