"""2d entry point; offline commands do not import CARLA or pygame."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys

from cross_right_2d import (CrossRightTrial, ENTRY_SPEEDS, definition_paths, digest, load_settings,
                            make_definitions, validate_definitions)

ROOT = Path(__file__).resolve().parent


def dispatch_arguments(argv):
    selected = '-2d' in argv
    scenarios, remaining = [], []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '-2d':
            i += 1
        elif arg == '--scenario' and i+1 < len(argv):
            scenarios.append(argv[i+1])
            i += 2
        elif arg.startswith('--scenario='):
            scenarios.append(arg.split('=', 1)[1])
            i += 1
        else:
            remaining.append(arg)
            i += 1
    normalize = lambda s: s.lower().replace('.', '')
    if not selected and not any(normalize(s) == '2d' for s in scenarios):
        return None
    if (any(s in remaining for s in ('-6c', '-1a', '-5e', '-5d', '-2c', '-2e', '--scenario'))
            or any(normalize(s) != '2d' for s in scenarios)):
        raise ValueError('-2d conflicts with another scenario selector')
    return remaining


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    tmp.replace(path)


def file_hash(path):
    sha = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            sha.update(block)
    return sha.hexdigest()


def expand(args, cfg):
    baseline = json.loads(Path(args.baseline).read_text(encoding='utf-8'))
    definitions = make_definitions(baseline, cfg)
    output = Path(args.input_dir or cfg['input_dir']).expanduser()
    if not args.dry_run:
        if output.exists() and any(output.iterdir()):
            raise ValueError('output is not empty; choose a new --input_dir')
        for d in definitions:
            write_json(output/(d['scenario']+'.json'), d)
        write_json(output/'manifest.json', dict(schema_version=2, scenario_code='2d', count=len(definitions),
            shared_by=['behavior', 'tcp'], conditions_per_controller=len(definitions), weather_catalog_version=2,
            baseline_fingerprint=baseline['baseline_fingerprint'],
            conditions={d['scenario']: d['condition_fingerprint'] for d in definitions}))
    print('2d: 8 speeds x 120 weather = 960 shared conditions (behavior / TCP); '+('dry run only' if args.dry_run else str(output)), flush=True)
    print('8 个速度是工程网格；一次筛选不是国标符合性，正式重复要求为同一方法 3 次。', flush=True)


def select_definitions(definitions, args):
    selected = [d for d in definitions if (args.speed is None or d['required_speed_kmh'] == args.speed)
                and (not args.weather or d['weather_id'] == args.weather)]
    if not selected:
        raise ValueError('no conditions match --speed / --weather')
    return selected


def load_run_definitions(args, cfg):
    if args.baseline:
        baseline = json.loads(Path(args.baseline).read_text(encoding='utf-8'))
        return make_definitions(baseline, cfg, speed=args.speed, weather=args.weather)
    source = Path(args.input_dir or cfg['input_dir']).expanduser()
    paths = [source] if source.is_file() else definition_paths(source)
    return select_definitions(validate_definitions(paths, cfg), args)


def exclude_completed_from_source(definitions, source_path, controller, checkpoint_hash):
    raw = Path(source_path).read_bytes()
    source = json.loads(raw)
    if source['metadata']['status'] == 'running':
        raise ValueError('source batch is still running')
    if source['metadata']['controller'] != controller:
        raise ValueError('source controller differs')
    if controller == 'behavior' and source['metadata'].get('controller_implementation') != 'carla_behavior_agent':
        raise ValueError('legacy scripted behavior results cannot be reused as BehaviorAgent results')
    identities = {d['scenario']: d['condition_fingerprint'] for d in definitions}
    latest = {}
    for r in source['records']:
        if r['scenario'] not in identities:
            continue
        if (r['condition_fingerprint'] != identities[r['scenario']] or r['controller'] != controller
                or r['checkpoint_sha256'] != checkpoint_hash):
            raise ValueError('source condition/controller/checkpoint differs')
        latest[r['scenario']] = r
    done = {name for name, r in latest.items() if r['result_status'] in ('PASS', 'FAILED')}
    return [d for d in definitions if d['scenario'] not in done], dict(
        source_ledger=str(Path(source_path).resolve()), source_sha256=hashlib.sha256(raw).hexdigest(),
        retained_valid_conditions=len(done))


def run(args, cfg):
    definitions = load_run_definitions(args, cfg)
    args.continuation = None
    if args.exclude_completed_from:
        output = args.output_dir or args.video_dir
        if output and Path(output).resolve() == Path(args.exclude_completed_from).resolve().parent:
            raise ValueError('use a new output directory for --exclude-completed-from')
        checkpoint_hash = file_hash(args.model_path or cfg['model_path']) if args.model == 'tcp' else None
        definitions, args.continuation = exclude_completed_from_source(
            definitions, args.exclude_completed_from, args.model, checkpoint_hash)
    if args.dry_run or not definitions:
        print('2d: {} selected conditions verified offline; CARLA/map execution not performed.'.format(len(definitions)))
        print('共用 8 个速度条件；TCP 自行达速，未满足记 FAILED；一次筛选不是国标符合性。')
        return 0
    with ExitStack() as locks:
        return run_connected(args, cfg, definitions, locks)


def run_connected(args, cfg, definitions, locks):
    import carla
    import fcntl
    from importlib.metadata import version
    from cross_right_2d_runtime import CrossRightScene, validate_world
    host, port = args.host or cfg['host'], args.port or cfg['port']
    lock = locks.enter_context(open('/tmp/stf_2d_'+digest([host, port])[:16]+'.lock', 'w'))
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    client = carla.Client(host, port)
    client.set_timeout(20)
    if version('carla') != '0.9.16' or client.get_client_version() != client.get_server_version():
        raise ValueError('2d requires CARLA 0.9.16 and matching client/server versions')
    world = client.get_world()
    checkpoint = Path(args.model_path or cfg['model_path']).resolve()
    if args.model == 'tcp' and not checkpoint.is_file():
        raise ValueError('TCP checkpoint not found: '+str(checkpoint))
    if args.resume and not (args.output_dir or args.video_dir):
        raise ValueError('--resume requires the original --output_dir')
    out = Path(args.output_dir or args.video_dir or Path(cfg['output_root'])/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_'+args.model))
    out.mkdir(parents=True, exist_ok=True)
    lock = locks.enter_context((out/'run.lock').open('w'))
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    dependencies = ['cross_right_2d.py', 'cross_right_2d_cli.py', 'cross_right_2d_runtime.py',
                    'cross_right_2d_video.py', 'cross_right_2d_routes.py', 'cross_right_2d_behavior.py', 'emergency_brake_5d.py',
                    'emergency_brake_5d_runtime.py', 'emergency_brake_5d_video.py',
                    'stf_weather_catalog.py', 'model/stf_tcp.py', 'stf_vehicle_control.py', 'render.py']
    runtime_hash = digest({p: file_hash(ROOT/p) for p in dependencies})
    checkpoint_hash = file_hash(checkpoint) if args.model == 'tcp' else None
    from cross_right_2d_behavior import source_identity
    controller_source = source_identity() if args.model == 'behavior' else None
    implementation = 'carla_behavior_agent' if args.model == 'behavior' else 'tcp'
    identities = {d['scenario']: d['condition_fingerprint'] for d in definitions}
    ledger, records = out/'2d_result.json', []
    if ledger.exists():
        if not args.resume:
            raise ValueError('existing results; use --resume or a new --output_dir')
        previous = json.loads(ledger.read_text(encoding='utf-8'))
        meta = previous['metadata']
        if (meta['controller'] != args.model or meta['runtime_sha256'] != runtime_hash
                or meta['checkpoint_sha256'] != checkpoint_hash or meta['conditions'] != identities
                or meta.get('continuation') != args.continuation
                or meta.get('controller_source') != controller_source):
            raise ValueError('resume differs in conditions, controller, code or checkpoint')
        records = previous['records']
        for r in records:
            if (r['condition_fingerprint'] != identities.get(r['scenario']) or r['controller'] != args.model
                    or r['runtime_sha256'] != runtime_hash or r['checkpoint_sha256'] != checkpoint_hash):
                raise ValueError('resume record identity mismatch')
    elif args.resume:
        raise ValueError('no existing 2d result ledger to resume')
    latest = {r['scenario']: r for r in records}
    done = {name for name, r in latest.items() if r['result_status'] in ('PASS', 'FAILED')}
    metadata = dict(status='running', controller=args.model, conditions=identities,
                    controller_implementation=implementation, controller_source=controller_source,
                    runtime_sha256=runtime_hash, checkpoint_sha256=checkpoint_hash,
                    continuation=args.continuation, started_at_utc=datetime.now(timezone.utc).isoformat(),
                    carla_version=client.get_server_version(), screening_only=True,
                    gb_t_conformity_claimed=False, physical_field_test_compliance_claimed=False,
                    collision_enhancement_enabled=False, sampling_hz=20, video_width=512, video_height=256,
                    sensor_video_width=256, sensor_video_height=256)

    def persist():
        latest = {r['scenario']: r for r in records}
        summary = {s: sum(r['result_status'] == s for r in latest.values()) for s in ('PASS', 'FAILED', 'INVALID')}
        summary.update(total=len(definitions), attempted=len(latest), remaining=len(definitions)-len(done))
        write_json(ledger, dict(metadata=metadata, summary=summary, records=records))
        write_json(out/'progress_latest.json', dict(metadata, summary=summary))
        rows = ['|{}|{}|{}|'.format(r['scenario'], r['result_status'], str(r.get('primary_reason') or '').replace('|', '/'))
                for r in latest.values()]
        (out/'2d_result.md').write_text('# 2d 仿真筛选结果\n\nbehavior 使用 CARLA BehaviorAgent；behavior 与 TCP 均为被测算法，采用相同评分。\n\n'+
            json.dumps(summary, ensure_ascii=False)+'\n\n|场景|结果|主原因|\n|---|---|---|\n'+'\n'.join(rows)+'\n', encoding='utf-8')

    original, weather = world.get_settings(), world.get_weather()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = cfg['engineering']['fixed_delta_seconds']
    configured = False
    try:
        persist()
        for d in definitions:
            if d['scenario'] in done:
                continue
            scene = writer = viz = record = None
            cleanup = dict(remaining_actor_ids=[])
            attempt = 1+sum(r['scenario'] == d['scenario'] for r in records)
            video_path = out/(d['scenario']+'_attempt_'+str(attempt)+'.mp4')
            frames = 0
            interrupted = False
            print('START', d['scenario'], flush=True)
            try:
                validate_world(world, d, cfg, args.town)
                if not configured:
                    configured = True
                    world.apply_settings(settings)
                world.set_weather(carla.WeatherParameters(**d['weather']))
                scene = CrossRightScene(world, d, model=args.model, model_path=str(checkpoint))
                scene.spawn(video=not args.no_video)
                if not args.no_video:
                    import imageio.v2 as imageio
                    from cross_right_2d_video import CrossRightVideo
                    viz = CrossRightVideo(world, scene.ego, headless=args.headless)
                    writer = imageio.get_writer(str(video_path), fps=20, codec='libx264', pixelformat='yuv420p')
                while True:
                    snap = world.get_snapshot()
                    running = scene.step()
                    if writer is not None:
                        rgb = scene.video.wait(snap.frame, snap.timestamp.elapsed_seconds, cfg['engineering']['camera_timeout_s'])
                        writer.append_data(viz.frame(rgb, scene.route))
                        frames += 1
                    if not running:
                        break
                    world.tick()
                record = scene.finish()
            except (Exception, KeyboardInterrupt) as exc:
                interrupted = isinstance(exc, KeyboardInterrupt)
                reason = 'trial_interrupted' if interrupted else str(exc) or type(exc).__name__
                print('2d error: '+reason, file=sys.stderr)
                trial = scene.trial if scene is not None else CrossRightTrial(d, args.model)
                trial.latch(reason, immediate=True)
                try:
                    record = scene.finish() if scene is not None else trial.result()
                except Exception:
                    record = trial.result()
                    if scene is not None:
                        record.update(telemetry_samples=scene.telemetry, control_samples=scene.controls,
                                      initialization=scene.initialization, initialization_samples=scene.initialization_samples,
                                      runtime_ego_route=scene.route, runtime_geometry=scene.b['geometry'],
                                      collision_events=scene.collisions, target_collision_events=scene.target_collisions,
                                      lane_events=scene.lane_events)
            finally:
                if scene is not None:
                    owned = [a.id for a in scene.actors]
                    try:
                        scene.destroy()
                        if owned:
                            world.tick()
                        remaining = sorted(a.id for a in world.get_actors() if a.id in owned)
                        cleanup = dict(deleted_actor_ids=sorted(set(owned)-set(remaining)), remaining_actor_ids=remaining)
                    except Exception as exc:
                        cleanup = dict(remaining_actor_ids=owned, error=str(exc))
                for resource in (writer, viz):
                    if resource is not None:
                        try:
                            resource.close()
                        except Exception as exc:
                            if record is not None:
                                record.update(result_status='INVALID', primary_reason='video_finalize_failed', trial_valid=False, **{'pass': None})
                                record['invalid_reasons'].append(str(exc))
            if cleanup['remaining_actor_ids'] or cleanup.get('error'):
                record['status_before_cleanup'] = record['result_status']
                record.update(result_status='INVALID', primary_reason='owned_actor_cleanup_failed', trial_valid=False, **{'pass': None})
                record['invalid_reasons'].append('owned_actor_cleanup_failed')
            record.update(controller_implementation=implementation, controller_source=controller_source, scenario=d['scenario'], weather_id=d['weather_id'], controller=args.model,
                          condition_fingerprint=d['condition_fingerprint'], runtime_sha256=runtime_hash,
                          checkpoint_sha256=checkpoint_hash, cleanup=cleanup, attempt=attempt,
                          video_path=str(video_path.resolve()) if frames else None, video_frame_count=frames,
                          video_width=512 if frames else None, video_height=256 if frames else None)
            detail_path = out/'details'/(d['scenario']+'_attempt_'+str(attempt)+'.json')
            write_json(detail_path, record)
            record['detail_path'] = str(detail_path.resolve())
            for key in ('telemetry_samples', 'control_samples', 'initialization_samples'):
                record[key.replace('_samples', '_sample_count')] = len(record.pop(key, []))
            record.pop('runtime_ego_route', None)
            records.append(record)
            if record['result_status'] in ('PASS', 'FAILED'):
                done.add(d['scenario'])
            persist()
            print('RESULT', d['scenario'], record['result_status'], flush=True)
            if (interrupted or cleanup['remaining_actor_ids'] or cleanup.get('error')
                    or (record['result_status'] == 'INVALID' and not args.continue_invalid)):
                metadata['status'] = 'interrupted'
                break
        else:
            metadata['status'] = 'completed' if len(done) == len(definitions) else 'incomplete'
    except BaseException:
        metadata['status'] = 'interrupted'
        raise
    finally:
        try:
            if configured:
                world.set_weather(weather)
        finally:
            try:
                if configured:
                    world.apply_settings(original)
            finally:
                metadata['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
                persist()
    print('OUTPUT', out, flush=True)
    return 0 if metadata['status'] == 'completed' else 1


def main(mode, argv=None):
    parser = argparse.ArgumentParser(description='2d: right-side straight crossing with live TTC; '+mode)
    parser.add_argument('--config', default=str(ROOT/'config/cross_right_2d.yaml'))
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    parser.add_argument('--town', help='check current map name; never switches maps')
    for key in ('save_dir', 'input_dir', 'output_dir', 'video_dir', 'baseline', 'model_path'):
        parser.add_argument('--'+key)
    parser.add_argument('--model', choices=('behavior', 'tcp'), default='behavior')
    parser.add_argument('--speed', type=int, choices=ENTRY_SPEEDS, help='required entry speed for either behavior or TCP')
    parser.add_argument('--weather', help='one weather profile, e.g. sunny_07')
    parser.add_argument('--exclude-completed-from')
    for key in ('expand', 'resume', 'dry-run', 'no-video', 'headless', 'continue-invalid'):
        parser.add_argument('--'+key, action='store_true')
    args = parser.parse_args(argv)
    for key in ('config', 'save_dir', 'input_dir', 'output_dir', 'video_dir', 'baseline', 'model_path', 'exclude_completed_from'):
        value = getattr(args, key, None)
        if value:
            setattr(args, key, str(Path(value).expanduser().resolve()))
    os.chdir(ROOT)
    try:
        cfg = load_settings(args.config)
        if args.expand:
            if not args.baseline:
                raise ValueError('--expand requires --baseline')
            expand(args, cfg)
        elif mode == 'editor':
            from cross_right_2d_editor import CrossRightEditor
            CrossRightEditor(args, cfg).run()
        elif mode == 'run':
            return run(args, cfg)
        else:
            raise ValueError('unknown mode: '+mode)
    except (ValueError, KeyError, OSError, RuntimeError) as exc:
        print('2d error: '+str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main('run'))
