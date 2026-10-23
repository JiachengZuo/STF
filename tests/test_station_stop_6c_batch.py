"""Fault-injection tests for 6c durable evidence and full-batch lifecycle."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from station_stop_6c import StationStopEvaluator, seal
from station_stop_6c_cli import (artifact_paths, batch_outcome, file_hash, latest_records,
                                recover_records, write_json)
from station_stop_6c_runtime import finalize_video_evidence, run_connected
from test_station_stop_6c import definition, fixture
from test_station_stop_6c_runtime import scene_fixture


class EvidenceRecoveryTests(unittest.TestCase):
    def evidence(self, out, video=False):
        d = definition()
        paths = artifact_paths(out, d, 1)
        row = dict(scenario=d['scenario'], attempt=1, profile_id=d['profile_id'],
                   weather_id=d['weather_id'], condition_fingerprint=d['condition_fingerprint'],
                   controller='behavior_reference_parking', result_status='FAILED', reasons=['collision'])
        if video:
            paths['video'].write_bytes(b'video evidence')
            row.update(video_path=str(paths['video'].resolve()), video_frame_count=1,
                       video_sha256=file_hash(paths['video']), video_evidence_errors=[])
        row = seal(row, 'result_fingerprint')
        write_json(paths['result'], row)
        return d, paths, row

    def test_compact_verdict_cannot_override_signed_details(self):
        with tempfile.TemporaryDirectory() as td:
            d, _, row = self.evidence(Path(td))
            row['result_status'] = 'PASS'
            with self.assertRaisesRegex(ValueError, 'ledger differs'):
                recover_records(Path(td), [row], [d], row['controller'])

    def test_missing_details_cannot_be_skipped_on_resume(self):
        with tempfile.TemporaryDirectory() as td:
            d, paths, row = self.evidence(Path(td))
            paths['result'].unlink()
            with self.assertRaisesRegex(ValueError, 'missing finalized'):
                recover_records(Path(td), [row], [d], row['controller'])

    def test_missing_or_modified_video_prevents_resume_skip(self):
        for mutation in ('missing', 'modified'):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as td:
                out = Path(td)
                d, paths, row = self.evidence(out, video=True)
                self.assertEqual(len(recover_records(out, [row], [d], row['controller'], require_video=True)), 1)
                if mutation == 'missing':
                    paths['video'].unlink()
                else:
                    paths['video'].write_bytes(b'changed video')
                with self.assertRaisesRegex(ValueError, 'video'):
                    recover_records(out, [row], [d], row['controller'], require_video=True)

    def test_latest_attempt_controls_completion_even_if_ledger_order_changes(self):
        rows = [dict(scenario='a', attempt=2, result_status='INVALID'),
                dict(scenario='a', attempt=1, result_status='FAILED')]
        self.assertEqual(latest_records(rows)['a']['result_status'], 'INVALID')
        self.assertEqual(batch_outcome(rows, 1), 'completed_with_invalid')
        self.assertEqual(batch_outcome(rows, 2), 'interrupted')

    def test_video_finalization_cannot_leave_a_false_pass(self):
        for status, expected in (('PASS', 'INVALID'), ('FAILED', 'FAILED')):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as td:
                scene, _ = scene_fixture()
                scene.trial.end(status, 'fixture_verdict')
                scene.telemetry.append(dict(frame=101))
                record = scene.trial.result()
                path = Path(td) / 'trial.mp4'
                path.write_bytes(b'partial video')
                errors = finalize_video_evidence(scene, record, path, 1, True, ['encoder_close_failed'])
                self.assertEqual(record['result_status'], expected)
                self.assertIn('encoder_close_failed', errors)
                self.assertIsNone(record['video_sha256'])

    def test_missing_terminal_video_frame_revokes_pass(self):
        with tempfile.TemporaryDirectory() as td:
            scene, _ = scene_fixture()
            scene.trial.end('PASS', 'fixture_verdict')
            scene.telemetry.extend([dict(frame=101), dict(frame=102)])
            record = scene.trial.result()
            path = Path(td) / 'trial.mp4'
            path.write_bytes(b'one frame only')
            errors = finalize_video_evidence(scene, record, path, 1, True)
            self.assertEqual(record['result_status'], 'INVALID')
            self.assertIn('video_frame_coverage_incomplete', errors)


class BatchLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name) / 'run'
        self.cfg, _ = fixture()
        self.d = definition()
        self.d['baseline']['opendrive_sha256'] = hashlib.sha256(b'offline').hexdigest()
        self.args = SimpleNamespace(tcp_mode=None, model='behavior', model_path=None,
            no_video=True, resume=False, output_dir=str(self.out), host=None, port=None,
            town=None, headless=True, continue_invalid=False)
        self.manifest = dict(manifest_fingerprint='offline_manifest')
        self.original = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None,
                                        no_rendering_mode=False)
        self.world = Mock()
        self.world.get_settings.side_effect = lambda: copy.copy(self.original)
        self.world.get_map.return_value = SimpleNamespace(name='offline_station', to_opendrive=lambda: 'offline')
        self.world.get_actors.return_value.filter.return_value = []
        self.world.get_snapshot.return_value = SimpleNamespace(frame=100, timestamp=SimpleNamespace(elapsed_seconds=0.))
        self.client = Mock()
        self.client.get_world.return_value = self.world
        self.client.get_client_version.return_value = '0.9.16'
        self.client.get_server_version.return_value = '0.9.16'

    def fake_scene(self, verdict):
        def create(world, d, *unused):
            scene = Mock()
            scene.trial = StationStopEvaluator(d)
            scene.handoff_frame = 100
            scene.telemetry = []
            scene.initialization = {}
            scene.actors = []
            scene.spawn.side_effect = lambda **kw: scene.trial.start(0., 100)
            def step(snap):
                scene.trial.end(verdict, 'injected_' + verdict.lower())
                scene.telemetry.append(dict(frame=snap.frame))
                return False
            scene.step.side_effect = step
            scene.finish.side_effect = lambda: dict(scene.trial.result(), telemetry_samples=scene.telemetry,
                                                   control_samples=[], initialization={})
            scene.cleanup.return_value = dict(remaining_actor_ids=[], errors=[])
            return scene
        return create

    def run_batch(self, verdict='FAILED', definitions=None):
        with patch('station_stop_6c_runtime.carla.Client', return_value=self.client), \
             patch('station_stop_6c_runtime.StationStopScene', side_effect=self.fake_scene(verdict)) as factory:
            result = run_connected(self.args, self.cfg, definitions or [self.d], self.manifest)
            return result, factory.call_count

    def test_completed_resume_does_not_connect_to_or_tick_carla(self):
        self.assertEqual(self.run_batch(), (0, 1))
        self.args.resume = True
        with patch('station_stop_6c_runtime.carla.Client', side_effect=AssertionError('CARLA should not be touched')):
            self.assertEqual(run_connected(self.args, self.cfg, [self.d], self.manifest), 0)

    def test_resume_can_expand_selection_and_skips_existing_valid_result(self):
        self.run_batch()
        extra = copy.deepcopy(self.d)
        extra['scenario'] += '_second'
        extra['condition_fingerprint'] += '_second'
        self.args.resume = True
        self.assertEqual(self.run_batch(definitions=[self.d, extra]), (0, 1))
        summary = json.loads((self.out / 'summary.json').read_text())
        self.assertEqual(summary['selected_conditions'], 2)
        self.assertEqual(summary['attempts'], 2)

    def test_occupied_synchronous_clock_is_not_modified(self):
        self.original.synchronous_mode = True
        with self.assertRaisesRegex(RuntimeError, 'already in synchronous'):
            self.run_batch()
        self.world.apply_settings.assert_not_called()

    def test_settings_application_error_still_restores_world(self):
        def apply(settings):
            if settings.synchronous_mode:
                raise RuntimeError('settings application failed')
        self.world.apply_settings.side_effect = apply
        with self.assertRaisesRegex(RuntimeError, 'settings application failed'):
            self.run_batch()
        self.assertFalse(self.world.apply_settings.call_args.args[0].synchronous_mode)
        self.world.set_weather.assert_called_with(self.world.get_weather.return_value)
        status = json.loads((self.out / 'batch_status.json').read_text())
        self.assertEqual(status['status'], 'interrupted')
        self.assertIn('settings application failed', status['error'])

    def test_continue_invalid_reports_completion_with_invalid_and_resume_retries(self):
        self.args.continue_invalid = True
        self.assertEqual(self.run_batch('INVALID'), (0, 1))
        self.assertEqual(json.loads((self.out / 'batch_status.json').read_text())['status'], 'completed_with_invalid')
        self.args.resume = True
        self.assertEqual(self.run_batch('PASS'), (0, 1))
        summary = json.loads((self.out / 'summary.json').read_text())
        self.assertEqual(summary['attempts'], 2)
        self.assertEqual(summary['counts'], dict(PASS=1, FAILED=0, INVALID=0))

    def test_incompatible_video_timing_fails_before_carla_is_touched(self):
        self.args.no_video = False
        self.d['engineering']['fixed_delta_seconds'] = .1
        with patch('station_stop_6c_runtime.carla.Client') as client:
            with self.assertRaisesRegex(ValueError, '20 fps video requires'):
                run_connected(self.args, self.cfg, [self.d], self.manifest)
            client.assert_not_called()


if __name__ == '__main__':
    unittest.main()
