"""1a dispatch and JSON batch execution, independent of legacy scene.tick."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import time
from pathlib import Path

from speed_limit_1a import VERSION, Trial, digest, validate_definition
from speed_limit_1a_artifacts import artifact_paths, archive_previous_video
from speed_limit_1a_video import VIDEO_PROFILE, VIDEO_METADATA

ROOT=Path(__file__).resolve().parent


def dispatch_arguments(argv):
    targets=[];remaining=[];i=0
    while i<len(argv):
        arg=argv[i]
        if arg=='-1a':targets.append('1a');i+=1
        elif arg=='--scenario' and i+1<len(argv):targets.append(argv[i+1]);i+=2
        elif arg.startswith('--scenario='):targets.append(arg.split('=',1)[1]);i+=1
        else:remaining.append(arg);i+=1
    normalized=[s.lower().replace('.','') for s in targets]
    if '1a' not in normalized:return None
    if any(s!='1a' for s in normalized) or any(s in remaining for s in ('-5d','-5e')):
        raise ValueError('1a conflicts with another scenario selection')
    return remaining


def file_hash(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    tmp.replace(path)


def load_definitions(args):
    paths=sorted(Path(args.input_dir).glob('scenario_1a_*.json'))
    if not paths:raise ValueError('no scenario_1a_*.json files found')
    found=[];names=set();conditions=set()
    for path in paths:
        doc=json.loads(path.read_text());d=doc[args.town][args.route_id][0]
        validate_definition(d)
        if d['name']!=path.stem or d['name'] in names:raise ValueError('duplicate/misnamed scenario')
        names.add(d['name'])
        key=(d['case_id'],d.get('weather_id','base'))
        if key in conditions:raise ValueError('duplicate case/weather pair')
        conditions.add(key)
        if d.get('route_sha256') and d['route_sha256']!=digest(d['ego_route']):
            raise ValueError('route fingerprint mismatch')
        if d.get('speed_rule_sha256') and d['speed_rule_sha256']!=digest(d['speed_limit_test']):
            raise ValueError('rule fingerprint mismatch')
        if args.weather and d.get('weather_id')!=args.weather:continue
        if args.case_id and d['case_id'] not in args.case_id:continue
        found.append((path,d))
    if not found:raise ValueError('no conditions match filters')
    return found[:args.max_cases] if args.max_cases else found


def main(argv=None):
    parser=argparse.ArgumentParser(description='1a virtual-rule screening: JSON routes and scoring; no speed control injection')
    parser.add_argument('--input_dir','--input-dir',default=str(ROOT/'save_scenarios/1a/expanded'))
    parser.add_argument('--town',default='1a');parser.add_argument('--route_id',default='route_01')
    parser.add_argument('--model',choices=('behavior','tcp'),default='behavior')
    parser.add_argument('--model_path','--model-path',default=str(ROOT/'tcp/best_model.ckpt'))
    parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,default=2000)
    parser.add_argument('--output_dir','--output-dir','--video_dir',dest='output_dir')
    parser.add_argument('--weather');parser.add_argument('--case-id',action='append')
    parser.add_argument('--max-cases',type=int);parser.add_argument('--resume',action='store_true')
    parser.add_argument('--dry-run',action='store_true',help='validate JSON only; no CARLA connection')
    parser.add_argument('--check-map',action='store_true',help='read current map hash only; no ticks or actors')
    parser.add_argument('--no-video',action='store_true')
    parser.add_argument('--headless',action='store_true',help='accepted; this runner never creates a GUI window')
    parser.add_argument('--camera-timeout',type=float,default=2.)
    parser.add_argument('--stall-timeout',type=float,default=30.)
    args=parser.parse_args(argv)
    import math
    if any(not math.isfinite(v) or v<=0 for v in (args.camera_timeout,args.stall_timeout)):
        parser.error('timeouts must be positive and finite')
    if args.max_cases is not None and args.max_cases<=0:parser.error('--max-cases must be positive')
    if args.town!='1a' or args.route_id!='route_01':parser.error('generated 1a JSON uses --town 1a --route_id route_01')
    if args.resume and not args.output_dir:parser.error('--resume requires --output_dir')
    definitions=load_definitions(args)
    print(f'1a: validated {len(definitions)} JSON conditions; virtual-rule screening, visual sign recognition NOT tested',flush=True)
    if args.dry_run:return 0
    import carla
    client=carla.Client(args.host,args.port);client.set_timeout(15.)
    world=client.get_world();actual_map=world.get_map()
    map_hash=hashlib.sha256(actual_map.to_opendrive().encode()).hexdigest()
    for _,d in definitions:
        if map_hash!=d['opendrive_sha256'] or actual_map.name.rsplit('/',1)[-1]!='1a':
            raise ValueError('CARLA map does not match JSON map identity')
    if args.check_map:
        print('Map hash verified; no world settings changed, no ticks or actors');return 0
    checkpoint=Path(args.model_path)
    if args.model=='tcp' and not checkpoint.is_file():raise ValueError('TCP checkpoint not found')
    if world.get_settings().synchronous_mode:raise RuntimeError('CARLA already in synchronous mode; another runner may own the clock')
    if world.get_actors().filter('vehicle.*'):raise RuntimeError('CARLA contains vehicles; cannot isolate this batch')
    import fcntl
    with open(f'/tmp/stf_1a_{args.host.replace("/","_")}_{args.port}.lock','w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return execute(args,definitions,world,client,checkpoint)


def execute(args,definitions,world,client,checkpoint):
    from speed_limit_1a_runtime import Scene, ModelFailure
    batch_started_monotonic=time.monotonic()
    batch_started_utc=datetime.now(timezone.utc).isoformat()
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    out=Path(args.output_dir or ROOT/'runs/speed_limit_1a'/args.model/stamp)
    out.mkdir(parents=True,exist_ok=True)
    runtime_files=['speed_limit_1a.py','speed_limit_1a_cli.py','speed_limit_1a_runtime.py',
                   'stf_scene.py','model/stf_tcp.py','TCP/TCP/model.py','TCP/TCP/config.py',
                   'emergency_brake_5d_runtime.py','run.py',
                   'TCP/TCP/resnet.py','agent/base_policy.py','speed_limit_1a_artifacts.py',
                   'speed_limit_1a_video.py','emergency_brake_5d_video.py','render.py']
    identity={'controller':args.model,'runtime_sha256':digest({p:file_hash(ROOT/p) for p in runtime_files}),
              'checkpoint_sha256':file_hash(checkpoint) if args.model=='tcp' else None,
              'scoring_version':VERSION,'fixed_delta_seconds':.05,
              'stall_timeout_s':args.stall_timeout,'camera_timeout_s':args.camera_timeout,
              'evaluation_scope':'virtual_rule_screening','video_enabled':not args.no_video,
              'video_profile':VIDEO_PROFILE if not args.no_video else None,
              'client_version':client.get_client_version(),'server_version':client.get_server_version()}
    ledger=out/'1a_result.json';records=[];migration_history=[]
    if ledger.exists():
        if not args.resume:raise ValueError('output already contains results; use a new directory or --resume')
        previous=json.loads(ledger.read_text())
        if previous['identity']!=identity:raise ValueError('resume controller/code/timing identity mismatch')
        records=previous['records']
        migration_history=previous.get('migration_history',[])
    hashes={d['name']:digest(d) for _,d in definitions}
    for record in records:
        if hashes.get(record['scenario'])!=record['condition_sha256']:
            raise ValueError('resume scenario identity/filter mismatch')
    latest_records={r['scenario']:r for r in records}
    done={name for name,r in latest_records.items() if r['result_status'] in ('PASS','FAILED')
          and (args.no_video or (r.get('video_format') or {}).get('profile')==VIDEO_PROFILE)}
    def persist(status):
        latest={r['scenario']:r for r in records}
        counts=Counter(r['result_status'] for r in latest.values())
        write_json(ledger,{'identity':identity,'batch_status':status,'total':len(definitions),
                          'summary':dict(counts),'records':records,
                          'batch_started_at_utc':batch_started_utc,
                          'batch_wall_time_s':time.monotonic()-batch_started_monotonic,
                          'artifact_layout':'native_json_named_videos',
                          'migration_history':migration_history})
        write_json(out/'progress_latest.json',{'status':status,'total':len(definitions),
                   'attempted':len(latest),'attempt_count':len(records),'summary':dict(counts),
                   'updated_at_utc':datetime.now(timezone.utc).isoformat()})
        report='# 1a 虚拟规则试跑结果\n\n'+json.dumps(dict(counts),ensure_ascii=False)+'\n\n'
        report+='| 场景 | 结果 | 原因 |\n|---|---|---|\n'
        report+='\n'.join('|{}|{}|{}|'.format(r['scenario'],r['result_status'],
                  '; '.join(r['failure_reasons']+r['invalid_reasons']).replace('|','/'))
                  for r in latest.values())
        (out/'1a_result.md').write_text(report+'\n')
    write_json(out/'artifact_layout.json',{'layout':'native_json_named_videos',
               'video_filename':'<input JSON basename>.mp4','details_directory':'details',
               'video_format':VIDEO_METADATA,
               'collector_required':False})
    original=world.get_settings();weather=world.get_weather();settings=world.get_settings()
    settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
    interrupted=False;fatal=False
    try:
        world.apply_settings(settings);persist('RUNNING')
        for number,(path,d) in enumerate(definitions,1):
            if d['name'] in done:continue
            attempt=sum(r['scenario']==d['name'] for r in records)+1
            paths=artifact_paths(out,path,attempt)
            archived=archive_previous_video(paths,d['name'],records)
            if archived:
                write_json(Path(archived['detail_path']),archived)
                persist('RUNNING')
            scene=None;record=None
            attempt_started_monotonic=time.monotonic()
            attempt_started_utc=datetime.now(timezone.utc).isoformat()
            print(f'[{number}/{len(definitions)}] {d["name"]}',flush=True)
            try:
                params=world.get_weather()
                for key,value in d['weather'].items():
                    if not hasattr(params,key):raise ValueError('unknown CARLA weather parameter:'+key)
                    setattr(params,key,float(value))
                world.set_weather(params)
                scene=Scene(world,d,path,args.model,checkpoint,out,video=not args.no_video,
                            camera_timeout=args.camera_timeout,stall_timeout=args.stall_timeout,attempt=attempt)
                scene.spawn()
                for _ in range(20):world.tick()
                for _ in range(int(scene.tcp.config.seq_len) if scene.tcp else 1):
                    snap=world.get_snapshot();scene.warmup(snap)
                    if world.get_snapshot().frame!=snap.frame:raise RuntimeError('external_tick_during_warmup')
                    world.tick()
                scene.handoff(world.get_snapshot())
                while scene.step(world.get_snapshot()):world.tick()
                record=scene.trial.result()
            except ModelFailure as exc:
                scene.trial.fail(str(exc));record=scene.trial.result()
            except KeyboardInterrupt:
                interrupted=True
                trial=scene.trial if scene else Trial(d)
                trial.invalidate('user_interrupted');record=trial.result()
            except Exception as exc:
                trial=scene.trial if scene else Trial(d)
                trial.invalidate(type(exc).__name__+':'+str(exc));record=trial.result()
                # Connection loss is a batch problem, not 960 separate trials.
                try:world.get_snapshot()
                except Exception:fatal=True
            finally:
                cleanup=scene.destroy() if scene else []
                if cleanup and record:
                    record['cleanup_errors']=cleanup;fatal=True
            record.update({'wall_time_s':time.monotonic()-attempt_started_monotonic,
                           'started_at_utc':attempt_started_utc,
                           'finished_at_utc':datetime.now(timezone.utc).isoformat(),
                           'scenario':d['name'],'case_id':d['case_id'],'weather_id':d.get('weather_id','base'),
                           'condition_sha256':hashes[d['name']],'input_file':str(path.resolve()),
                           'controller':args.model,'attempt':attempt,'repeat_id':1,'output_dir':str(out.resolve()),
                           'execution_identity':dict(identity),
                           'video_format':dict(VIDEO_METADATA) if not args.no_video else None,
                           'video_frame_count':scene.video_frames if scene else 0,
                           'detail_path':str(paths['result']),
                           'events_path':str(paths['events']),
                           'telemetry_path':str(paths['telemetry']) if paths['telemetry'].exists() else None,
                           'video_path':str(paths['video']) if paths['video'].exists() else None,
                           'map_review_flags':d['speed_limit_test'].get('review_flags',[]),
                           'applicability':'UNKNOWN','collision_events':scene.collisions if scene else []})
            write_json(paths['events'],record['events'])
            write_json(paths['result'],record);records.append(record);persist('RUNNING')
            print(record['result_status'],record['failure_reasons'] or record['invalid_reasons'],flush=True)
            if interrupted or fatal:break
            world.tick()
    except BaseException:
        fatal=True
        raise
    finally:
        try:world.set_weather(weather)
        finally:world.apply_settings(original)
        persist('INTERRUPTED' if interrupted else 'ABORTED' if fatal else 'FINISHED')
    print('Results:',ledger,flush=True)
    return 130 if interrupted else 1 if fatal else 0
