"""2d batch lifecycle, evidence persistence and shared algorithm scoring."""
import copy
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from cross_right_2d import CrossRightTrial, ENTRY_SPEEDS, make_definitions, speed_control
from cross_right_2d_cli import run_connected
from test_cross_right_2d import definition, fixture, observation


class FakeWorld:
    def __init__(self):
        self.settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        self.weather = 'original-weather'
        self.actors = []
        self.settings_calls, self.weather_calls = [], []
        self.frame = 0

    def get_settings(self):
        return copy.deepcopy(self.settings)

    def apply_settings(self, value):
        self.settings = copy.deepcopy(value)
        self.settings_calls.append(value)

    def get_weather(self):
        return self.weather

    def set_weather(self, value):
        self.weather = value
        self.weather_calls.append(value)

    def tick(self):
        self.frame += 1
        return self.frame

    def get_snapshot(self):
        return SimpleNamespace(frame=self.frame, timestamp=SimpleNamespace(elapsed_seconds=self.frame*.05))

    def get_actors(self):
        return list(self.actors)


class BatchTests(unittest.TestCase):
    def environment(self, fault=None):
        world = FakeWorld()
        module = ModuleType('cross_right_2d_runtime')
        state = dict(spawned=0, destroyed=0)

        class Scene:
            def __init__(self, world, d, model='behavior', model_path=None):
                self.world, self.d, self.trial = world, d, CrossRightTrial(d, model)
                self.actors, self.telemetry, self.controls = [], [], []
                self.collisions, self.target_collisions, self.lane_events = [], [], []
                self.step_count = 0

            def spawn(self, video):
                state['spawned'] += 1
                self.actors = [SimpleNamespace(id=state['spawned']*10+i) for i in range(2)]
                self.world.actors.extend(self.actors)
                if fault == 'spawn':
                    raise RuntimeError('sensor_setup_failed')

            def step(self):
                observations = [(0., observation(x=-80.)), (1.8, observation(x=-60.)),
                                (8.2, observation(x=-30., v=0., target_y=-4.3)),
                                (12., observation(x=18., v=5., target_y=-10., target_v=0.))]
                if fault == 'slow':
                    observations[0] = (0., observation(x=-60., v=20/3.6))
                    observations[1] = (1.8, observation(x=-30., v=20/3.6))
                now, obs = observations[self.step_count]
                self.step_count += 1
                self.trial.update(now, obs)
                self.telemetry.append(obs)
                return not self.trial.done

            def finish(self):
                return dict(self.trial.result(), telemetry_samples=self.telemetry, control_samples=self.controls)

            def destroy(self):
                state['destroyed'] += 1
                if fault != 'cleanup':
                    self.world.actors = []
                self.actors = []

        def validate(*args):
            if fault == 'busy':
                raise RuntimeError('world_has_traffic')

        module.CrossRightScene, module.validate_world = Scene, validate
        carla = ModuleType('carla')
        carla.Client = lambda *a: SimpleNamespace(set_timeout=lambda x: None, get_client_version=lambda: 'build',
                                                 get_server_version=lambda: 'build', get_world=lambda: world)
        carla.WeatherParameters = lambda **kw: kw
        return world, state, module, carla

    def invoke(self, out, fault=None, resume=False, weather=None, previous=None, model='behavior'):
        cfg, b = fixture()
        docs = make_definitions(b, cfg, speed=40, weather=weather or 'sunny_07')
        if fault in ('cleanup', 'slow'):
            docs += make_definitions(b, cfg, speed=40, weather='sunny_08')
        checkpoint = Path(out)/'test_checkpoint'
        if model == 'tcp' and not checkpoint.exists():
            checkpoint.write_bytes(b'fake checkpoint for batch lifecycle test')
        args = SimpleNamespace(host=None, port=None, model_path=str(checkpoint), model=model, resume=resume,
                               output_dir=str(out), video_dir=None, continuation=None, town=None,
                               no_video=True, headless=True, continue_invalid=fault != 'slow')
        world, state, runtime, carla = previous or self.environment(fault)
        with patch.dict(sys.modules, {'carla': carla, 'cross_right_2d_runtime': runtime}), \
                patch('importlib.metadata.version', return_value='0.9.16'), \
                patch('cross_right_2d_behavior.source_identity', return_value={'implementation': 'carla_behavior_agent', 'sha256': 'test'}), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), ExitStack() as locks:
            code = run_connected(args, cfg, docs, locks)
        ledger = json.loads((Path(out)/'2d_result.json').read_text())
        return code, ledger, (world, state, runtime, carla)

    def test_pass_detail_restore_and_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, ledger, env = self.invoke(tmp)
            self.assertEqual(code, 0)
            self.assertEqual(ledger['summary']['PASS'], 1)
            record = ledger['records'][0]
            detail = json.loads(Path(record['detail_path']).read_text())
            self.assertEqual(len(detail['telemetry_samples']), 4)
            self.assertNotIn('telemetry_samples', record)
            self.assertEqual(record['telemetry_sample_count'], 4)
            world, state, _, _ = env
            self.assertFalse(world.settings.synchronous_mode)
            self.assertEqual(world.weather, 'original-weather')
            self.assertEqual(world.actors, [])
            self.assertEqual(state['destroyed'], 1)
            code, resumed, _ = self.invoke(tmp, resume=True, previous=env)
            self.assertEqual(code, 0)
            self.assertEqual(state['spawned'], 1)
            self.assertEqual(len(resumed['records']), 1)
            with self.assertRaisesRegex(ValueError, 'resume differs'):
                self.invoke(tmp, resume=True, weather='sunny_08', previous=env)

    def test_same_condition_runs_with_both_models(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            code_b, behavior, _ = self.invoke(a)
            code_t, tcp, _ = self.invoke(b, model='tcp')
            self.assertEqual((code_b, code_t), (0, 0))
            rb, rt = behavior['records'][0], tcp['records'][0]
            self.assertEqual(rb['condition_fingerprint'], rt['condition_fingerprint'])
            self.assertEqual(rb['scenario'], rt['scenario'])
            self.assertEqual((rb['controller'], rt['controller']), ('behavior', 'tcp'))
            self.assertEqual((rb['required_speed_kmh'], rt['required_speed_kmh']), (40, 40))
            self.assertEqual((rb['result_status'], rt['result_status']), ('PASS', 'PASS'))
            self.assertIsNone(rt['commanded_speed_kmh'])

    def test_tcp_speed_failures_continue_and_resume_as_completed(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, ledger, env = self.invoke(tmp, fault='slow', model='tcp')
            self.assertEqual(code, 0)
            self.assertEqual(ledger['summary']['FAILED'], 2)
            self.assertEqual(ledger['summary']['INVALID'], 0)
            self.assertEqual(ledger['summary']['remaining'], 0)
            self.assertTrue(all(r['primary_reason'] == 'required_speed_not_reached' for r in ledger['records']))
            for r in ledger['records']:
                detail = json.loads(Path(r['detail_path']).read_text())
                self.assertEqual(detail['speed_requirement_evidence']['required_speed_kmh'], 40)
                self.assertAlmostEqual(detail['speed_requirement_evidence']['observed_speed_kmh'], 20)
            self.assertEqual(env[1]['spawned'], 2)
            code, resumed, _ = self.invoke(tmp, fault='slow', model='tcp', resume=True, previous=env)
            self.assertEqual(code, 0)
            self.assertEqual(len(resumed['records']), 2)
            self.assertEqual(env[1]['spawned'], 2)

    def test_partial_spawn_invalid_is_recorded_and_cleaned_then_retried(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, ledger, env = self.invoke(tmp, fault='spawn')
            self.assertEqual(code, 1)
            self.assertEqual(ledger['records'][0]['primary_reason'], 'sensor_setup_failed')
            self.assertEqual(env[0].actors, [])
            self.assertFalse(env[0].settings.synchronous_mode)
            code, ledger, env = self.invoke(tmp, fault='spawn', resume=True, previous=env)
            self.assertEqual(len(ledger['records']), 2)
            self.assertEqual(ledger['records'][-1]['attempt'], 2)

    def test_cleanup_failure_overrides_pass_and_stops_even_continue_invalid(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, ledger, env = self.invoke(tmp, fault='cleanup')
            self.assertEqual(code, 1)
            self.assertEqual(len(ledger['records']), 1)
            record = ledger['records'][0]
            self.assertEqual(record['primary_reason'], 'owned_actor_cleanup_failed')
            self.assertEqual(record['status_before_cleanup'], 'PASS')
            self.assertEqual(env[1]['spawned'], 1)

    def test_busy_world_gets_a_record_without_settings_or_weather_mutation(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, ledger, env = self.invoke(tmp, fault='busy')
            self.assertEqual(code, 1)
            self.assertEqual(ledger['records'][0]['primary_reason'], 'world_has_traffic')
            self.assertEqual(env[0].settings_calls, [])
            self.assertEqual(env[0].weather_calls, [])
            self.assertEqual(env[1]['spawned'], 0)


if __name__ == '__main__':
    unittest.main()
