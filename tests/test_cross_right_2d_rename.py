"""Naming migration must preserve trial parameters and both algorithms' inputs."""
import copy
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from cross_right_2d import digest, make_definitions
from cross_right_2d_cli import load_run_definitions, write_json
from scripts.rename_cross_right_2d_json import migrate_directories
from test_cross_right_2d import fixture


class RenameTests(unittest.TestCase):
    def setUp(self):
        self.cfg, baseline = fixture()
        self.documents = [make_definitions(baseline, self.cfg, speed=v, weather='sunny_07')[0]
                          for v in (5, 40)]

    def seed(self, directory):
        directory.mkdir()
        legacy = []
        for original in self.documents:
            d = copy.deepcopy(original)
            d['scenario'] = d['scenario'].replace('scenario_2d_v', 'cross_right_2d_v', 1)
            d.pop('condition_fingerprint')
            d['condition_fingerprint'] = digest(d)
            write_json(directory/(d['scenario']+'.json'), d)
            legacy.append(d)
        write_json(directory/'manifest.json', dict(schema_version=2, scenario_code='2d', count=2,
            shared_by=['behavior', 'tcp'], conditions_per_controller=2, entry_speed_tolerance_kmh=2.,
            conditions={d['scenario']: d['condition_fingerprint'] for d in legacy}))
        return legacy

    @staticmethod
    def snapshot(directory):
        return {p.name: p.read_bytes() for p in directory.glob('*.json')}

    def test_migration_preserves_parameters_backup_and_shared_loading(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory, backup = Path(tmp)/'conditions', Path(tmp)/'backup'
            self.seed(directory)
            originals = self.snapshot(directory)
            write_json(directory/'scenario_2d_0001.json', {'legacy_generic': True})
            result = migrate_directories([directory], backup_dir=backup)
            self.assertEqual(result['renamed'], 2)
            for d in self.documents:
                self.assertEqual(json.loads((directory/(d['scenario']+'.json')).read_text()), d)
            self.assertFalse(list(directory.glob('cross_right_2d_v*.json')))
            manifest = json.loads((directory/'manifest.json').read_text())
            self.assertEqual(manifest['conditions'], {d['scenario']: d['condition_fingerprint'] for d in self.documents})
            self.assertEqual(manifest['entry_speed_tolerance_kmh'], 2.)
            self.assertEqual(self.snapshot(backup/'00_conditions'), originals)
            self.assertEqual(json.loads((directory/'scenario_2d_0001.json').read_text()), {'legacy_generic': True})
            for model in ('behavior', 'tcp'):
                args = SimpleNamespace(baseline=None, input_dir=str(directory), speed=None, weather=None, model=model)
                self.assertEqual(load_run_definitions(args, self.cfg), self.documents)
                args.input_dir = str(directory/(self.documents[0]['scenario']+'.json'))
                self.assertEqual(load_run_definitions(args, self.cfg), self.documents[:1])
            migrated = self.snapshot(directory)
            self.assertEqual(migrate_directories([directory], backup_dir=backup)['renamed'], 0)
            self.assertEqual(self.snapshot(directory), migrated)

    def test_old_snapshots_keep_their_identity_and_default_expands_home(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)/'conditions'
            legacy = self.seed(directory)
            originals = self.snapshot(directory)
            args = SimpleNamespace(baseline=None, input_dir=None, speed=None, weather=None, model='tcp')
            cfg = dict(self.cfg, input_dir='~/conditions')
            with patch.dict(os.environ, HOME=tmp):
                self.assertEqual(load_run_definitions(args, cfg), legacy)
            self.assertEqual(self.snapshot(directory), originals)

    def test_preflight_collision_and_bad_fingerprint_leave_all_inputs_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            a, b, backup = Path(tmp)/'a', Path(tmp)/'b', Path(tmp)/'backup'
            self.seed(a)
            legacy = self.seed(b)
            target = b/(self.documents[0]['scenario']+'.json')
            target.write_text('existing user file')
            originals = (self.snapshot(a), self.snapshot(b))
            with self.assertRaisesRegex(ValueError, 'destination already exists'):
                migrate_directories([a, b], backup_dir=backup)
            self.assertEqual((self.snapshot(a), self.snapshot(b)), originals)
            self.assertFalse(backup.exists())
            target.unlink()
            legacy[0]['required_speed_kmh'] = 10
            write_json(b/(legacy[0]['scenario']+'.json'), legacy[0])
            originals = (self.snapshot(a), self.snapshot(b))
            with self.assertRaisesRegex(ValueError, 'fingerprint'):
                migrate_directories([a, b], backup_dir=backup)
            self.assertEqual((self.snapshot(a), self.snapshot(b)), originals)

    def test_dry_run_does_not_write_and_aliases_cannot_run_twice(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory, backup = Path(tmp)/'conditions', Path(tmp)/'backup'
            self.seed(directory)
            originals = self.snapshot(directory)
            report = migrate_directories([directory], dry_run=True, backup_dir=backup)
            self.assertEqual(report['renamed'], 2)
            self.assertFalse(backup.exists())
            self.assertEqual(self.snapshot(directory), originals)
            d = self.documents[0]
            write_json(directory/(d['scenario']+'.json'), d)
            args = SimpleNamespace(baseline=None, input_dir=str(directory), speed=None, weather=None)
            with self.assertRaisesRegex(ValueError, 'definition differs'):
                load_run_definitions(args, self.cfg)

    def test_write_failure_restores_original_files_and_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            a, b, backup = Path(tmp)/'a', Path(tmp)/'b', Path(tmp)/'backup'
            self.seed(a)
            self.seed(b)
            originals = (self.snapshot(a), self.snapshot(b))

            def fail_second_manifest(path, data):
                if path == b/'manifest.json':
                    raise OSError('simulated write failure')
                write_json(path, data)

            with patch('scripts.rename_cross_right_2d_json.write_json', side_effect=fail_second_manifest):
                with self.assertRaisesRegex(OSError, 'simulated write failure'):
                    migrate_directories([a, b], backup_dir=backup)
            self.assertEqual((self.snapshot(a), self.snapshot(b)), originals)
            self.assertEqual(json.loads((backup/'migration.json').read_text())['status'], 'rolled_back')


if __name__ == '__main__':
    unittest.main()
