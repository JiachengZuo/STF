"""Independent 6c editor/expansion/runner; offline commands need no CARLA."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import sys
import tempfile
from station_stop_6c import build_definitions, generation_spec, load_settings, seal, validate_definitions, verify

ROOT = Path(__file__).resolve().parent


def dispatch_arguments(argv):
    selected = '-6c' in argv
    scenarios, remaining = [], []
    i = 0
    while i < len(argv):
        item = argv[i]
        if item == '-6c':
            i += 1; continue
        if item == '--scenario' and i + 1 < len(argv):
            scenarios.append(argv[i + 1]); i += 2; continue
        if item.startswith('--scenario='):
            scenarios.append(item.split('=', 1)[1]); i += 1; continue
        remaining.append(item); i += 1
    normalize = lambda value: value.lower().replace('.', '')
    if not selected and not any(normalize(s) == '6c' for s in scenarios):
        return None
    if any(normalize(s) != '6c' for s in scenarios) or any(x in remaining for x in ('-1a', '-5d', '-5e')):
        raise ValueError('6c conflicts with another scenario selector')
    return remaining


def write_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def file_hash(path):
    sha = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            sha.update(block)
    return sha.hexdigest()


def artifact_paths(out, definition, attempt):
    """Keep scenario/weather-named MP4s flat, like the 5d/5e TCP batches."""
    scenario = definition['scenario']
    if (attempt < 1 or not re.fullmatch(r'scenario_6c_[A-Za-z0-9_-]+', scenario)
            or definition['profile_id'] not in scenario
            or definition['weather_id'] not in scenario):
        raise ValueError('invalid 6c output identity')
    stem = '{}_attempt_{}'.format(scenario, attempt)
    out = Path(out)
    return dict(video=out / (stem + '.mp4'),
                result=out / 'details' / (stem + '.json'),
                condition=out / 'details' / (stem + '.condition.json'))


def expand(args, cfg):
    baseline = json.loads(Path(args.baseline).read_text(encoding='utf-8'))
    spec = generation_spec(cfg)
    definitions = build_definitions(baseline, spec)
    out = Path(args.input_dir or cfg['input_dir'])
    if not args.dry_run:
        if out.exists():
            raise ValueError('output already exists; choose a new --input_dir (immutable batches)')
        out.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix='.6c_expand_', dir=out.parent))
        try:
            for row in definitions:
                write_json(staging / (row['scenario'] + '.json'), row)
            manifest = seal(dict(schema_version=1, scenario_code='6c', count=len(definitions),
                                 baseline_fingerprint=baseline['baseline_fingerprint'], spec=spec,
                                 conditions={d['scenario']: d['condition_fingerprint'] for d in definitions}),
                            'manifest_fingerprint')
            write_json(staging / 'manifest.json', manifest)
            validate_definitions(staging)
            staging.rename(out)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
    print('6c: {} distances x {} speeds x 120 weather = {} conditions; {}'.format(
        len(spec['matrix']['initial_distances_m']), len(spec['matrix']['initial_speeds_kmh']),
        len(definitions), 'offline dry run' if args.dry_run else out), flush=True)


def select_definitions(rows, args):
    selected = [d for d in rows if (not args.weather or d['weather_id'] == args.weather)
                and (not args.profiles or d['profile_id'] in args.profiles)]
    if not selected:
        raise ValueError('no conditions match --weather / --profiles')
    if args.profiles and set(args.profiles) - {d['profile_id'] for d in rows}:
        raise ValueError('unknown profile')
    return selected


def latest_records(records):
    latest = {}
    for row in records:
        previous = latest.get(row['scenario'])
        if previous is None or row.get('attempt', 0) >= previous.get('attempt', 0):
            latest[row['scenario']] = row
    return latest


def batch_outcome(records, selected_count, interrupted=False):
    summary = summarize(records, selected_count)
    if interrupted or summary['unattempted_conditions']:
        return 'interrupted'
    return 'completed_with_invalid' if summary['counts']['INVALID'] else 'completed'


def summarize(records, selected_count):
    latest = latest_records(records)
    counts = {s: sum(r['result_status'] == s for r in latest.values()) for s in ('PASS', 'FAILED', 'INVALID')}
    valid = counts['PASS'] + counts['FAILED']
    return dict(selected_conditions=selected_count, attempts=len(records), counts=counts,
                completed_conditions=valid, unattempted_conditions=selected_count - len(latest),
                pass_rate_valid=counts['PASS'] / valid if valid else None,
                coverage=valid / selected_count if selected_count else 0.)


def persist(out, records, selected_count):
    write_json(out / 'results.json', records)
    summary = summarize(records, selected_count)
    groups = {}
    for row in latest_records(records).values():
        groups.setdefault(row['profile_id'], {})[row['scenario']] = row
    summary['by_profile'] = {k: {s: sum(r['result_status'] == s for r in v.values())
                                     for s in ('PASS', 'FAILED', 'INVALID')} for k, v in groups.items()}
    write_json(out / 'summary.json', summary)
    fields = ['scenario', 'profile_id', 'weather_id', 'attempt', 'result_status', 'trial_valid',
              'pass', 'controller', 'reasons', 'detail_path', 'video_path',
              'started_at_utc', 'finished_at_utc', 'wall_time_s']
    temp = out / 'results.csv.tmp'
    with temp.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore'); writer.writeheader()
        for row in records:
            writer.writerow({k: json.dumps(v, ensure_ascii=False) if isinstance(v, (dict, list)) else v
                             for k, v in row.items() if k in fields})
    temp.replace(out / 'results.csv')


def recover_records(out, records, definitions, controller, *, require_video=False):
    """Rebuild from sealed details; a compact ledger alone cannot complete a trial."""
    out = Path(out)
    expected = {d['scenario']: d['condition_fingerprint'] for d in definitions}
    known = {}
    for row in records:
        if (row['scenario'] not in expected
                or row['condition_fingerprint'] != expected[row['scenario']]
                or row['controller'] != controller):
            raise ValueError('result ledger does not match this run')
        key = (row['scenario'], row['attempt'])
        if key in known:
            raise ValueError('duplicate attempt in result ledger')
        known[key] = row
    recovered = {}
    for definition in definitions:
        name = definition['scenario']
        paths = [p for p in (out / 'details').glob(name + '_attempt_*.json')
                 if re.fullmatch(re.escape(name) + r'_attempt_\d+\.json', p.name)]
        for path in paths:
            row = json.loads(path.read_text(encoding='utf-8'))
            verify(row, 'result_fingerprint')
            if (row['scenario'] != name or row['condition_fingerprint'] != expected[name]
                or row['controller'] != controller or row['attempt'] != int(path.stem.rsplit('_attempt_', 1)[1])
                or row['result_status'] not in ('PASS', 'FAILED', 'INVALID')):
                raise ValueError('attempt evidence does not match this run')
            key = (name, row['attempt'])
            compact = {k: v for k, v in row.items() if k not in ('telemetry_samples', 'control_samples')}
            if key in known:
                ledger = {k: v for k, v in known[key].items() if k != 'detail_path'}
                if ledger != compact:
                    raise ValueError('result ledger differs from finalized evidence')
            if require_video and row['result_status'] in ('PASS', 'FAILED'):
                video = artifact_paths(out, definition, row['attempt'])['video'].resolve()
                if (row.get('video_evidence_errors') or not row.get('video_path')
                        or Path(row['video_path']).resolve() != video
                        or not video.is_file() or video.stat().st_size == 0
                        or row.get('video_frame_count', 0) <= 0):
                    raise ValueError('recorded video evidence is missing or incomplete: ' + name)
                if not row.get('video_sha256') or file_hash(video) != row['video_sha256']:
                    raise ValueError('recorded video checksum mismatch: ' + name)
            compact['detail_path'] = str(path.resolve())
            recovered[key] = compact
    if set(known) - set(recovered):
        raise ValueError('result ledger references missing finalized attempt evidence')
    return [recovered[key] for key in sorted(recovered)]


def run(args, cfg):
    rows, manifest = validate_definitions(args.input_dir or cfg['input_dir'])
    definitions = select_definitions(rows, args)
    if args.dry_run:
        print('6c matrix verified: {} conditions, {} selected; offline only, map/assets not checked'.format(
            len(rows), len(definitions)))
        return 0
    from station_stop_6c_runtime import run_connected
    return run_connected(args, cfg, definitions, manifest)


def main(mode, argv=None):
    parser = argparse.ArgumentParser(description='6c: one route, four distances x two speeds x 120 weather')
    parser.add_argument('--config', default=str(ROOT / 'config/station_stop_6c.yaml'))
    for flag in ('baseline', 'save_dir', 'input_dir', 'output_dir', 'model_path', 'town'):
        aliases = {'town': ['--name'], 'output_dir': ['--video_dir']}
        parser.add_argument('--' + flag, *aliases.get(flag, []), dest=flag)
    parser.add_argument('--headless', action='store_true', help='Record the standard BEV/front video using a virtual display')
    parser.add_argument('--host'); parser.add_argument('--port', type=int)
    parser.add_argument('--model', choices=('behavior', 'tcp'), default='behavior')
    parser.add_argument('--tcp-mode', choices=('hybrid', 'raw_goal'))
    parser.add_argument('--weather', help='e.g. sunny_07; subset selection after complete matrix validation')
    parser.add_argument('--profiles', nargs='+', help='e.g. d40_v10 d40_v20')
    for flag in ('dry-run', 'resume', 'no-video', 'continue-invalid', 'expand'):
        parser.add_argument('--' + flag, action='store_true')
    args = parser.parse_args(argv)
    try:
        for key in ('config', 'baseline', 'save_dir', 'input_dir', 'output_dir', 'model_path'):
            if getattr(args, key):
                setattr(args, key, str(Path(getattr(args, key)).resolve()))
        os.chdir(ROOT); cfg = load_settings(args.config)
        if args.expand:
            mode = 'expand'
        if mode == 'expand':
            if not args.baseline:
                parser.error('--baseline is required for expansion')
            expand(args, cfg)
        elif mode == 'editor':
            from station_stop_6c_editor import StationStopEditor
            StationStopEditor(args, cfg).run()
        else:
            return run(args, cfg)
        return 0
    except (ValueError, RuntimeError, OSError, KeyError) as exc:
        print('6c: ' + str(exc), file=sys.stderr)
        return 2

