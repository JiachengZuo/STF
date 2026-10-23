"""Compatibility contract: legacy argv/imports and explicit local dispatch."""
import ast
import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from stf_runner_dispatch import ARGV_MAIN_RUNNERS, RUNNERS, dispatch, select_runner


class RunnerCompatibility(unittest.TestCase):
    def test_standalone_help_needs_no_simulator_import(self):
        with patch('stf_runner_dispatch.importlib.import_module') as load,contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(dispatch(['--help'],force_local=True),0)
            self.assertIn('2d',output.getvalue())
            load.assert_not_called()

    def test_all_legacy_scenarios_pass_through_without_importing_local_runners(self):
        for code in ['1a','1b','1c','1d','2a','2b','2c','2d','2e','2g','3a','3b_1','4a','5a','5c','5d','5e','6c']:
            argv=['--scenario',code,'--input_dir','cases','--town','roadside_1','--model','tcp']
            with self.subTest(code=code), patch('stf_runner_dispatch.importlib.import_module') as load:
                self.assertEqual(select_runner(argv),(None,argv))
                self.assertIsNone(dispatch(argv))
                load.assert_not_called()

    def test_explicit_selectors_and_old_local_syntax(self):
        for code in RUNNERS:
            for selector in [[f'-{code}'],[f'--{code}'],['--runner','local','--scenario',code],
                             ['--runner=local',f'--scenario={code}']]:
                with self.subTest(selector=selector):
                    self.assertEqual(select_runner(selector+['--model','tcp']),(code,['--model','tcp']))
            self.assertEqual(select_runner(['--scenario',code],force_local=True),(code,[]))

    def test_only_selected_module_imported_and_errors_propagate(self):
        for code,module in RUNNERS.items():
            target=Mock();target.main.return_value=7
            with patch('stf_runner_dispatch.importlib.import_module',return_value=target) as load, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(dispatch(['-'+code,'--model','behavior']),7)
                load.assert_called_once_with(module)
                expected=() if code in ARGV_MAIN_RUNNERS else (['--model','behavior'],) if code=='1a' else ('run',['--model','behavior'])
                target.main.assert_called_once_with(*expected)

    def test_conflicts_fail_before_importing_carla(self):
        cases=[['-2d','-5e'],['-2d','--scenario','3a'],['--runner','legacy','-2d'],
               ['--runner','local','--scenario','2b'],['--runner','bad'],['--runner','local'],
               ['--3b_1','--5a'],['--5a','--5c'],['--5c','--scenario','5a'],
               ['--runner','legacy','--3b_1']]
        for argv in cases:
            with self.subTest(argv=argv),patch('stf_runner_dispatch.importlib.import_module') as load,contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(dispatch(argv),2);load.assert_not_called()

    def test_double_dash_preserved(self):
        args=['--scenario','3a','--','-2d']
        self.assertEqual(select_runner(args),(None,args))

    def test_explicit_legacy_only_consumes_new_option(self):
        argv=['--runner','legacy','--scenario','2d']
        self.assertEqual(select_runner(argv),(None,['--scenario','2d']))

    def test_original_parsers_receive_scenario_and_restore_argv(self):
        old=sys.argv[:]
        for code in ARGV_MAIN_RUNNERS:
            target=Mock()
            target.main.side_effect=lambda: self.assertEqual(sys.argv[1:],['--scenario',code,'--input_dir','cases','--model','tcp'])
            with self.subTest(code=code),patch('stf_runner_dispatch.importlib.import_module',return_value=target),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(dispatch(['--'+code,'--input_dir','cases','--model','tcp']),0)
            self.assertEqual(sys.argv,old)

    def test_script_parser_exit_status_and_argv_are_preserved(self):
        old=sys.argv[:]
        for code in ('3b_1','5a','5c'):
            target=Mock();target.main.side_effect=SystemExit(2)
            with self.subTest(code=code),patch('stf_runner_dispatch.importlib.import_module',return_value=target),contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    dispatch(['--'+code,'--bad-option'])
                self.assertEqual(error.exception.code,2)
            self.assertEqual(sys.argv,old)

    def test_private_weather_does_not_modify_legacy_catalog(self):
        import weather_catalog
        import stf_weather_catalog
        self.assertFalse(hasattr(weather_catalog,'WEATHER_CATALOG_VERSION'))
        self.assertEqual(stf_weather_catalog.WEATHER_CATALOG_VERSION,2)
        self.assertNotEqual(weather_catalog.all_weather_profiles(),stf_weather_catalog.all_weather_profiles())

    def test_action_decoder_default_and_preserve_throttle_are_isolated(self):
        from roundabout_2b import decode_vehicle_action as legacy
        from stf_vehicle_control import decode_vehicle_action as local
        self.assertEqual(legacy([[.4,.2,.6]]),(0.,.2,.6))
        self.assertEqual(local([[.4,.2,.6]]),(0.,.2,.6))
        self.assertEqual(local([[.4,.2,.6]],preserve_throttle=True),(.4,.2,.6))

    def test_private_tcp_has_strict_loader_and_legacy_keeps_eager_load(self):
        root=Path(__file__).resolve().parents[1]
        def methods(path):
            tree=ast.parse((root/path).read_text())
            cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='TCPAgent')
            return {n.name:n for n in cls.body if isinstance(n,ast.FunctionDef)}
        old,new=methods('model/tcp.py'),methods('model/stf_tcp.py')
        self.assertNotIn('strict',[a.arg for a in old['load_model'].args.args])
        self.assertIn('strict',[a.arg for a in new['load_model'].args.args])
        loads=lambda n:any(isinstance(c,ast.Call) and isinstance(c.func,ast.Attribute) and c.func.attr=='load_model' for c in ast.walk(n))
        self.assertTrue(loads(old['__init__']));self.assertFalse(loads(new['__init__']))

if __name__=='__main__':
    unittest.main()
