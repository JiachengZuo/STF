"""Exercise the supplied scripts through the adapter without a simulator connection."""
import argparse
import contextlib
import importlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from stf_runner_dispatch import dispatch

PROFILES = {
    '3b_1': ('STF-3-b-I-t1', 'save_scenarios_3b', 'PedestrianCrossScene'),
    '5a': ('STF-5-a', 'save_scenarios_5a', 'PedestrianCrossScene'),
    '5c': ('5c_4', 'save_scenarios_5c', 'CarCutOutandStaticScene'),
}


class ParsedArguments(Exception):
    pass


class StopBeforeWorldLoad(BaseException):
    """Stops at the intended map-load call; deliberately not caught as Exception."""


class SuppliedRunnerTests(unittest.TestCase):
    def test_registry_points_to_the_three_supplied_files(self):
        root = Path(__file__).resolve().parents[1]
        for code in PROFILES:
            module = importlib.import_module('scripts.run_'+code)
            with self.subTest(code=code):
                self.assertEqual(Path(module.__file__).resolve(), root/'scripts'/('run_'+code+'.py'))

    def test_profiles_keep_their_original_scene_dependencies(self):
        import scene
        for code, (town, directory, class_name) in PROFILES.items():
            module = importlib.import_module('scripts.run_'+code)
            with self.subTest(code=code):
                self.assertEqual(module.SCENARIO_PROFILE['scenario'], code)
                self.assertEqual(module.SCENARIO_PROFILE['town'], town)
                self.assertEqual(Path(module.SCENARIO_PROFILE['input_dir']).name, directory)
                self.assertIs(module.SCENE_CLASSES[code], getattr(scene, class_name))

    def test_real_parsers_receive_all_forwarded_parameters_and_fixed_defaults(self):
        parse = argparse.ArgumentParser.parse_args
        for code, (town, _, _) in PROFILES.items():
            captured = []
            def capture(parser, *args, **kwargs):
                captured.append(parse(parser, *args, **kwargs))
                raise ParsedArguments()
            original_argv = sys.argv[:]
            module = importlib.import_module('scripts.run_'+code)
            with self.subTest(code=code), patch.object(argparse.ArgumentParser, 'parse_args', capture), \
                    patch.object(module.carla, 'Client') as client, contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(ParsedArguments):
                    dispatch(['--'+code, '--input_dir', '/example/cases', '--video_dir', '/example/videos',
                              '--model', 'tcp', '--model_path', '/example/model.ckpt', '--host', '127.0.0.2',
                              '--port', '2100', '--route_id', 'route_custom', '--max_scenario_time', '42'])
                client.assert_not_called()
            args = captured[0]
            self.assertEqual((args.scenario, args.town, args.route_id), (code, town, 'route_custom'))
            self.assertEqual((args.input_dir, args.video_dir), ('/example/cases', '/example/videos'))
            self.assertEqual((args.model, args.model_path), ('tcp', '/example/model.ckpt'))
            self.assertEqual((args.host, args.port, args.max_scenario_time), ('127.0.0.2', 2100, 42))
            self.assertEqual(sys.argv, original_argv)

    def test_wrong_town_is_rejected_before_any_client_or_map_load(self):
        for code in PROFILES:
            module = importlib.import_module('scripts.run_'+code)
            with self.subTest(code=code), patch.object(module.carla, 'Client') as client, \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()) as error:
                with self.assertRaises(SystemExit) as raised:
                    dispatch(['--'+code, '--town', 'wrong_map'])
                self.assertEqual(raised.exception.code, 2)
                self.assertIn('dedicated to town', error.getvalue())
                client.assert_not_called()

    def test_each_script_keeps_its_own_map_loading_flow(self):
        for code, (town, _, _) in PROFILES.items():
            module = importlib.import_module('scripts.run_'+code)
            with self.subTest(code=code), contextlib.ExitStack() as stack:
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                stack.enter_context(patch.object(module, 'load_collision_config', return_value={}))
                stack.enter_context(patch.object(module.os, 'makedirs'))
                files = stack.enter_context(patch.object(module, 'get_sorted_scenario_files', return_value=['case.json']))
                stack.enter_context(patch.object(module, 'load_result_records', return_value=[]))
                stack.enter_context(patch.object(module.pygame, 'init'))
                stack.enter_context(patch.object(module.pygame.display, 'get_driver', return_value='dummy'))
                client = stack.enter_context(patch.object(module.carla, 'Client'))
                client.return_value.load_world.side_effect = StopBeforeWorldLoad()
                with self.assertRaises(StopBeforeWorldLoad):
                    dispatch(['--'+code, '--model', 'behavior', '--input_dir', '/example/cases'])
                files.assert_called_once_with('/example/cases', code)
                client.assert_called_once_with('127.0.0.1', 2000)
                client.return_value.set_timeout.assert_called_once_with(600.0)
                client.return_value.load_world.assert_called_once_with(town)


if __name__ == '__main__':
    unittest.main()
