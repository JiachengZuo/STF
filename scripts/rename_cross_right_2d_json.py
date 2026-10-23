#!/usr/bin/env python3
"""Migrate shared 2d definitions to scenario_2d_v*.json, with original backups.

Only the explicitly selected directories are scanned (no recursion). Scenario
parameters are preserved; scenario names, fingerprints and manifest identities
are updated together. Repeating a completed migration makes no changes.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cross_right_2d import canonical_definition, definition_paths, load_settings
from cross_right_2d_cli import write_json


def plan_directory(directory):
    directory = Path(directory).expanduser().resolve()
    if not directory.is_dir():
        raise ValueError('not a directory: '+str(directory))
    paths = definition_paths(directory)
    if not paths:
        raise ValueError('no shared 2d definitions: '+str(directory))
    originals, conditions, changes = {}, {}, []
    for source in paths:
        raw = source.read_bytes()
        document = json.loads(raw)
        if source.stem != document.get('scenario'):
            raise ValueError('filename differs from scenario: '+str(source))
        renamed = canonical_definition(document)
        target = directory/(renamed['scenario']+'.json')
        if target != source and target.exists():
            raise ValueError('destination already exists: '+str(target))
        if renamed['scenario'] in conditions:
            raise ValueError('duplicate condition: '+renamed['scenario'])
        originals[document['scenario']] = document['condition_fingerprint']
        conditions[renamed['scenario']] = renamed['condition_fingerprint']
        if target != source:
            changes.append(dict(source=source, target=target, raw=raw, document=renamed))

    manifest_path = directory/'manifest.json'
    manifest_raw = manifest = None
    if manifest_path.exists():
        manifest_raw = manifest_path.read_bytes()
        manifest = json.loads(manifest_raw)
        if (manifest.get('schema_version') != 2 or manifest.get('scenario_code') != '2d'
                or manifest.get('conditions') != originals or manifest.get('count') != len(paths)):
            raise ValueError('manifest differs from definitions: '+str(manifest_path))
        manifest = dict(manifest, conditions=conditions)
    return dict(directory=directory, count=len(paths), changes=changes,
                manifest_path=manifest_path, manifest_raw=manifest_raw, manifest=manifest)


def migrate_directories(directories, *, dry_run=False, backup_dir=None):
    # Check every input and collision before writing to any of the directories.
    directories = list(dict.fromkeys(Path(p).expanduser().resolve() for p in directories))
    plans = [plan_directory(p) for p in directories]
    report = dict(dry_run=dry_run, renamed=sum(len(p['changes']) for p in plans),
                  directories=[dict(path=str(p['directory']), conditions=p['count'],
                                    renamed=len(p['changes'])) for p in plans], backup_dir=None)
    if dry_run or not report['renamed']:
        return report

    backup = Path(backup_dir).expanduser().resolve() if backup_dir else (
        ROOT/'tmp'/('2d_json_rename_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')))
    backup.mkdir(parents=True, exist_ok=False)
    report['backup_dir'] = str(backup)
    mapping = []
    for i, plan in enumerate(plans):
        if not plan['changes']:
            continue
        folder = backup/('{:02d}_'.format(i)+plan['directory'].name)
        folder.mkdir()
        plan['backup'] = folder
        for change in plan['changes']:
            (folder/change['source'].name).write_bytes(change['raw'])
            mapping.append(dict(source=str(change['source']), target=str(change['target']),
                backup=str(folder/change['source'].name),
                old_fingerprint=json.loads(change['raw'])['condition_fingerprint'],
                new_fingerprint=change['document']['condition_fingerprint']))
        if plan['manifest_raw'] is not None:
            (folder/'manifest.json').write_bytes(plan['manifest_raw'])
    write_json(backup/'migration.json', dict(report, status='backed_up', mapping=mapping))

    created, touched = [], []
    try:
        for plan in plans:
            if not plan['changes']:
                continue
            touched.append(plan)
            for change in plan['changes']:
                # Exclusive creation prevents replacing an existing destination.
                with change['target'].open('x', encoding='utf-8') as stream:
                    created.append(change['target'])
                    json.dump(change['document'], stream, ensure_ascii=False, indent=2, allow_nan=False)
                    stream.write('\n')
            if plan['manifest'] is not None:
                write_json(plan['manifest_path'], plan['manifest'])
            for change in plan['changes']:
                change['source'].unlink()
    except (Exception, KeyboardInterrupt):
        for plan in touched:
            for change in plan['changes']:
                shutil.copyfile(plan['backup']/change['source'].name, change['source'])
            if plan['manifest_raw'] is not None:
                shutil.copyfile(plan['backup']/'manifest.json', plan['manifest_path'])
        for path in created:
            path.unlink(missing_ok=True)
        write_json(backup/'migration.json', dict(report, status='rolled_back', mapping=mapping))
        raise
    write_json(backup/'migration.json', dict(report, status='completed', mapping=mapping))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='*', help='defaults to the configured 2d input directory')
    parser.add_argument('--dry-run', action='store_true', help='check and report without writing')
    parser.add_argument('--backup-dir', help='new directory for the original JSONs and rename mapping')
    args = parser.parse_args(argv)
    try:
        directories = args.directories or [load_settings(ROOT/'config/cross_right_2d.yaml')['input_dir']]
        report = migrate_directories(directories, dry_run=args.dry_run, backup_dir=args.backup_dir)
    except (ValueError, KeyError, OSError) as exc:
        print('2d rename error: '+str(exc), file=sys.stderr)
        return 1
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
