"""Dedicated 5e CLI, imported before the legacy CARLA/editor dependencies."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys

from curve_emergency_5e import (RISK_IDS, TCP_STATIONARY_TIMEOUT_S, digest, load_settings, make_definitions,
                                 trace_route, validate_definitions, xy_distance,
                                 map_identifier, require_map, ensure_world)

ROOT = Path(__file__).resolve().parent


def dispatch_arguments(argv):
    selected = '-5e' in argv
    scenarios, remaining = [], []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '-5e':
            i += 1
            continue
        if arg == '--scenario' and i+1 < len(argv):
            scenarios.append(argv[i+1]);i+=2
            continue
        if arg.startswith('--scenario='):
            scenarios.append(arg.split('=',1)[1]);i+=1
            continue
        remaining.append(arg);i+=1
    normalize = lambda x: x.lower().replace('.','')
    if not selected and not any(normalize(s)=='5e' for s in scenarios):
        return None
    if '-5d' in remaining or any(normalize(s)!='5e' for s in scenarios):
        raise ValueError('-5e conflicts with another scenario selector')
    return remaining


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    tmp.replace(path)


def expand(args, cfg):
    baseline = json.loads(Path(args.baseline).read_text(encoding='utf-8'))
    definitions = make_definitions(baseline,cfg)
    output = Path(args.input_dir or cfg['input_dir'])
    if not args.dry_run:
        if output.exists() and any(output.iterdir()):
            raise ValueError('output is not empty; choose a new --input_dir')
        for d in definitions:
            write_json(output/(d['scenario']+'.json'),d)
        write_json(output/'manifest.json',dict(scenario_code='5e',count=len(definitions),
            baseline_fingerprint=baseline['baseline_fingerprint'],
            conditions={d['scenario']:d['condition_fingerprint'] for d in definitions}))
    print('5e: 8 risk templates x 120 weather = {} conditions; {}'.format(
          len(definitions),'dry run only' if args.dry_run else output),flush=True)


def select_definitions(definitions, args):
    if args.risks:
        definitions = [d for d in definitions if d['risk']['id'] in args.risks]
    if args.weather:
        definitions = [d for d in definitions if d['weather_id'] == args.weather]
    if not definitions:
        raise ValueError('no conditions match --risk / --weather')
    return definitions


def validate_world(world, definitions):
    b = definitions[0]['baseline']
    map_obj = world.get_map()
    if hashlib.sha256(map_obj.to_opendrive().encode()).hexdigest() != b['opendrive_sha256']:
        raise ValueError('CARLA map differs from the saved baseline')
    traced = trace_route(map_obj,[b['ego_start']]+b['route_anchors'])
    if len(traced)!=len(b['ego_route']) or any(xy_distance(a,c)>.01 or abs(a['z']-c['z'])>.01
                                              for a,c in zip(traced,b['ego_route'])):
        raise ValueError('saved dense route differs from current map topology')
    available = {bp.id for bp in world.get_blueprint_library()}
    required = {definitions[0]['ego_blueprint']} | {d['risk']['blueprint'] for d in definitions}
    missing = required-available
    if missing:
        raise ValueError('server is missing blueprints: '+', '.join(sorted(missing)))


def file_hash(path):
    sha = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            sha.update(block)
    return sha.hexdigest()


def validate_carla_version(client):
    """Source builds report a git hash rather than the wheel's release number."""
    from importlib.metadata import version
    package = version('carla')
    client_version, server_version = client.get_client_version(), client.get_server_version()
    if package != '0.9.16' or client_version != server_version:
        raise ValueError('5e requires CARLA API 0.9.16 and matching client/server builds: '
                         'package={}, client={}, server={}'.format(package,client_version,server_version))


def exclude_completed_from_source(definitions, source_path, controller, checkpoint_hash):
    """Select pending/INVALID conditions without relabeling old runtime evidence."""
    path = Path(source_path).resolve()
    raw = path.read_bytes()
    source = json.loads(raw)
    metadata = source['metadata']
    if metadata['status'] == 'running':
        raise ValueError('source batch is still running; stop it before changing evaluation rules')
    if metadata['controller'] != controller:
        raise ValueError('source controller differs from the requested controller')
    identities = {d['scenario']: d['condition_fingerprint'] for d in definitions}
    latest = {}
    for record in source['records']:
        if (record['condition_fingerprint'] != identities.get(record['scenario'])
                or record['controller'] != controller
                or record['checkpoint_sha256'] != checkpoint_hash):
            raise ValueError('source conditions, controller or checkpoint differ')
        if record['result_status'] not in ('PASS', 'FAILED', 'INVALID'):
            raise ValueError('source contains an unfinished result')
        latest[record['scenario']] = record
    completed = {name for name, r in latest.items() if r['result_status'] in ('PASS', 'FAILED')}
    provenance = dict(source_ledger=str(path), source_sha256=hashlib.sha256(raw).hexdigest(),
                      source_runtime_sha256=metadata['runtime_sha256'],
                      retained_valid_conditions=len(completed),
                      source_invalid_conditions=sum(r['result_status'] == 'INVALID' for r in latest.values()),
                      original_total=len(definitions))
    return [d for d in definitions if d['scenario'] not in completed], provenance


def run(args, cfg):
    paths = sorted(Path(args.input_dir or cfg['input_dir']).glob('scenario_5e_*.json'))
    definitions = validate_definitions(paths,cfg)
    args.continuation = None
    if getattr(args, 'exclude_completed_from', None):
        source = Path(args.exclude_completed_from).resolve()
        output = args.output_dir or args.video_dir
        if output and Path(output).resolve() == source.parent:
            raise ValueError('new evaluation rules require a separate output directory')
        checkpoint = Path(args.model_path or cfg['model_path']).resolve()
        definitions, args.continuation = exclude_completed_from_source(
            definitions, source, args.model, file_hash(checkpoint) if args.model == 'tcp' else None)
    definitions = select_definitions(definitions,args)
    if args.dry_run:
        print('5e: complete 960-condition matrix verified; {} selected. Offline only; map/assets not checked.'.format(len(definitions)))
        return
    with ExitStack() as locks:
        return run_connected(args,cfg,definitions,locks)


def run_connected(args, cfg, definitions, locks):
    import carla
    import fcntl
    from curve_emergency_5e_runtime import CurveScene
    host,port = args.host or cfg['host'], args.port or cfg['port']
    lock = locks.enter_context(open('/tmp/stf_5e_'+digest([host,port])[:16]+'.lock','w'))
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    client = carla.Client(host,port)
    client.set_timeout(20)
    validate_carla_version(client)
    town = args.town or cfg['town']
    require_map(definitions[0]['baseline']['map_name'], town)
    world = ensure_world(client, town)
    validate_world(world,definitions)
    if list(world.get_actors().filter('vehicle.*')) or list(world.get_actors().filter('walker.pedestrian.*')):
        raise RuntimeError('world contains traffic actors; use an idle test world')
    checkpoint = Path(args.model_path or cfg['model_path']).resolve()
    if args.model=='tcp' and not checkpoint.is_file():
        raise ValueError('TCP checkpoint not found: '+str(checkpoint))
    if args.resume and not (args.output_dir or args.video_dir):
        raise ValueError('--resume requires the original --output_dir')
    out = Path(args.output_dir or args.video_dir or
               Path(cfg['output_root'])/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+args.model))
    out.mkdir(parents=True,exist_ok=True)
    lock = locks.enter_context((out/'run.lock').open('w'))
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ledger = out/'5e_result.json'
    records = []
    if ledger.exists():
        if not args.resume:
            raise ValueError('existing results; use --resume or a new --output_dir')
        previous = json.loads(ledger.read_text(encoding='utf-8'))
        if previous['metadata'].get('continuation') != getattr(args, 'continuation', None):
            raise ValueError('resume continuation source has changed')
        records = previous['records']
    elif args.resume:
        raise ValueError('no existing 5e result ledger to resume')
    dependencies = ['curve_emergency_5e.py','curve_emergency_5e_runtime.py','curve_emergency_5e_cli.py',
                    'curve_emergency_5e_video.py',
                    'emergency_brake_5d.py','emergency_brake_5d_runtime.py','emergency_brake_5d_video.py',
                    'stf_weather_catalog.py','model/stf_tcp.py','stf_vehicle_control.py','render.py']
    runtime_hash = digest({p:file_hash(ROOT/p) for p in dependencies})
    checkpoint_hash = file_hash(checkpoint) if args.model=='tcp' else None
    identities = {d['scenario']:d['condition_fingerprint'] for d in definitions}
    for r in records:
        if (r['condition_fingerprint']!=identities.get(r['scenario']) or r['controller']!=args.model
                or r['runtime_sha256']!=runtime_hash or r['checkpoint_sha256']!=checkpoint_hash):
            raise ValueError('resume differs in conditions, controller, code or checkpoint')
    done = {r['scenario'] for r in records if r['result_status'] in ('PASS','FAILED')}
    status = dict(status='running',total=len(definitions),controller=args.model,
                  carla_version=client.get_server_version(),started_at_utc=datetime.now(timezone.utc).isoformat(),
                  collision_enhancement_enabled=False,screening_only=True,
                  reference_uses_ground_truth=args.model=='behavior',runtime_sha256=runtime_hash,
                  ego_stationary_timeout_s=TCP_STATIONARY_TIMEOUT_S if args.model=='tcp' else None,
                  ego_stationary_speed_threshold_mps=cfg['engineering']['stop_speed_mps'])
    if getattr(args, 'continuation', None) is not None:
        status['continuation'] = args.continuation

    def persist():
        latest = {r['scenario']:r for r in records}
        summary = {s:sum(r['result_status']==s for r in latest.values()) for s in ('PASS','FAILED','INVALID')}
        summary.update(total=len(definitions),attempted=len(latest),remaining=len(definitions)-len(done))
        write_json(ledger,dict(metadata=status,summary=summary,records=records))
        write_json(out/'progress_latest.json',dict(status,**{'summary':summary}))
        rows = ['|{}|{}|{}|'.format(r['scenario'],r['result_status'],
                                  '; '.join(r['failure_reasons']+r['invalid_reasons'])) for r in latest.values()]
        (out/'5e_result.md').write_text('# 5e 仿真结果\n\n'+json.dumps(summary,ensure_ascii=False)+
            '\n\n|场景|结果|原因|\n|---|---|---|\n'+'\n'.join(rows)+'\n',encoding='utf-8')

    original,weather = world.get_settings(),world.get_weather()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = cfg['engineering']['fixed_delta_seconds']
    try:
        world.apply_settings(settings)
        persist()
        for d in definitions:
            if d['scenario'] in done:
                continue
            scene = writer = viz = record = None
            cleanup = None
            attempt = 1+sum(r['scenario']==d['scenario'] for r in records)
            video_path = out/(d['scenario']+'_attempt_'+str(attempt)+'.mp4')
            frames = 0
            print('START',d['scenario'],flush=True)
            try:
                world.set_weather(carla.WeatherParameters(**d['weather']))
                scene = CurveScene(world,d,args.model,str(checkpoint))
                scene.spawn(video=not args.no_video)
                if not args.no_video:
                    import imageio.v2 as imageio
                    from curve_emergency_5e_video import CurveVideo
                    viz = CurveVideo(world,scene.ego,headless=args.headless)
                    writer = imageio.get_writer(str(video_path),fps=1/settings.fixed_delta_seconds,
                                               codec='libx264',pixelformat='yuv420p')
                while True:
                    snap = world.get_snapshot()
                    running = scene.step()
                    if writer is not None:
                        rgb = scene.video.wait(snap.frame,snap.timestamp.elapsed_seconds,cfg['engineering']['camera_timeout_s'])
                        writer.append_data(viz.frame(rgb,scene.route));frames+=1
                    if not running:
                        break
                    world.tick()
                record = scene.finish()
            except Exception as exc:
                reason = type(exc).__name__+': '+str(exc)
                if scene is not None:
                    scene.trial.end('INVALID',reason)
                    try:
                        record = scene.finish()
                    except Exception:
                        record = None
                if record is None:
                    record = dict(result_status='INVALID',trial_valid=False,**{'pass':None},
                                  failure_reasons=[],invalid_reasons=[reason])
            finally:
                try:
                    if scene is not None:
                        owned_ids = [actor.id for actor in getattr(scene,'actors',[])]
                        ego_id = getattr(getattr(scene,'ego',None),'id',None)
                        scene.destroy()
                        if owned_ids:
                            # Flush destruction before video finalization/result I/O
                            # so the last rendered frame does not retain a stopped Ego.
                            world.tick()
                            remaining = sorted(actor.id for actor in world.get_actors() if actor.id in owned_ids)
                            cleanup = dict(ego_id=ego_id,ego_deleted=ego_id not in remaining,
                                           deleted_actor_ids=sorted(set(owned_ids)-set(remaining)),
                                           remaining_actor_ids=remaining)
                finally:
                    try:
                        if writer is not None:writer.close()
                    finally:
                        if viz is not None:viz.close()
            if cleanup is not None:
                record['cleanup'] = cleanup
                if cleanup['remaining_actor_ids']:
                    record.update(result_status='INVALID',trial_valid=False,**{'pass':None})
                    record['invalid_reasons'].append('owned_actor_cleanup_failed')
            record.update(scenario=d['scenario'],risk_id=d['risk']['id'],weather_id=d['weather_id'],
                          condition_fingerprint=d['condition_fingerprint'],controller=args.model,
                          runtime_sha256=runtime_hash,checkpoint_sha256=checkpoint_hash,
                          video_path=str(video_path.resolve()) if frames else None,video_frame_count=frames)
            # Keep per-frame evidence once per attempt; do not rewrite the full
            # telemetry of hundreds of previous trials on every ledger update.
            detail_path = out/'details'/(d['scenario']+'_attempt_'+str(attempt)+'.json')
            write_json(detail_path,record)
            record['detail_path'] = str(detail_path.resolve())
            for key in ('telemetry_samples','control_samples'):
                record[key.replace('_samples','_sample_count')] = len(record.pop(key,[]))
            records.append(record)
            if record['result_status'] in ('PASS','FAILED'):done.add(d['scenario'])
            persist()
            print('RESULT',d['scenario'],record['result_status'],flush=True)
            if (cleanup and cleanup['remaining_actor_ids']) or (record['result_status']=='INVALID' and not args.continue_invalid):
                status['status']='interrupted';break
        else:
            status['status']='completed' if len(done)==len(definitions) else 'incomplete'
    except BaseException:
        status['status']='interrupted'
        raise
    finally:
        try:
            world.set_weather(weather)
        finally:
            try:
                world.apply_settings(original)
            finally:
                status['finished_at_utc']=datetime.now(timezone.utc).isoformat()
                persist()
    print('OUTPUT',out,flush=True)


def main(mode, argv=None):
    parser = argparse.ArgumentParser(description='5e fixed bend: 8 risks x 120 weather; '+mode)
    parser.add_argument('--config',default=str(ROOT/'config/curve_emergency_5e.yaml'))
    parser.add_argument('--host');parser.add_argument('--port',type=int)
    parser.add_argument('--town','--name',dest='town',
                        help='map name (.umap accepted) or /Game/... path; defaults to config town')
    for flag in ('save_dir','input_dir','output_dir','video_dir','baseline','model_path'):
        parser.add_argument('--'+flag)
    parser.add_argument('--model',choices=('behavior','tcp'),default='behavior',
                        help='behavior = ground-truth route/yield reference; tcp = tested TCP')
    parser.add_argument('--risk','--risks',dest='risks',nargs='+',choices=RISK_IDS)
    parser.add_argument('--weather',help='one profile, e.g. sunny_07')
    parser.add_argument('--exclude-completed-from',
                        help='Prior stopped result ledger: skip its PASS/FAILED; use a new output directory')
    for flag in ('expand','resume','dry-run','no-video','headless','continue-invalid'):
        parser.add_argument('--'+flag,action='store_true')
    args = parser.parse_args(argv)
    # Resolve explicit paths before moving to the repository for configured paths.
    for key in ('config','save_dir','input_dir','output_dir','video_dir','baseline','model_path','exclude_completed_from'):
        if getattr(args,key,None):setattr(args,key,str(Path(getattr(args,key)).resolve()))
    os.chdir(ROOT)
    try:
        cfg = load_settings(args.config)
        if args.town:
            args.town = map_identifier(args.town)
            cfg['town'] = args.town
        if args.expand:mode='expand'
        if mode=='expand':
            if not args.baseline:parser.error('--baseline is required for expansion')
            expand(args,cfg)
        elif mode=='editor':
            from curve_emergency_5e_editor import CurveEditor
            CurveEditor(args,cfg).run()
        elif mode=='run':run(args,cfg)
        else:parser.error('unknown mode')
    except (ValueError,KeyError,OSError,RuntimeError) as exc:
        print('5e error: '+str(exc),file=sys.stderr)
        return 1
    return 0
