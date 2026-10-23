"""Independent 5d editor/expander/runner entrypoint; no legacy collision enhancement."""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
from datetime import datetime,timezone
from emergency_brake_5d import digest,load_settings,minimum_route_length,validate_geometry,speed_band,capability_labels,route_feasibility,RouteLengthError
ROOT=Path(__file__).resolve().parent


def dispatch_arguments(argv):
    """Return 5d-specific arguments, or None for unrelated legacy scenarios."""
    found=False;remaining=[];i=0;scenarios=[]
    while i<len(argv):
        arg=argv[i]
        if arg=='-5d':found=True;i+=1;continue
        if arg=='--scenario' and i+1<len(argv):
            scenarios.append(argv[i+1]);i+=2;continue
        if arg.startswith('--scenario='):scenarios.append(arg.split('=',1)[1]);i+=1;continue
        remaining.append(arg);i+=1
    if not found and not any(s.lower().replace('.','')=='5d' for s in scenarios):return None
    if any(s.lower().replace('.','')!='5d' for s in scenarios):raise ValueError('-5d conflicts with --scenario')
    return remaining


def write_json(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n');tmp.replace(path)


def apply_result_policy(record, incomplete_as_failed=False):
    """Score an attempted condition while preserving physical trial validity."""
    record['result_policy']='incomplete_as_failed' if incomplete_as_failed else 'valid_trials_only'
    if not record['trial_valid'] and incomplete_as_failed:
        record['raw_trial_result']={key:copy.deepcopy(record.get(key)) for key in
            ('trial_valid','pass','phase','failure_reasons','invalid_reasons')}
        record['pass']=False
        record['result_status']='FAILED'
    else:
        record['result_status']=('PASS' if record['pass'] else 'FAILED') if record['trial_valid'] else 'INVALID'
    return record


def condition_completed(record):
    return record.get('result_status') in ('PASS','FAILED') or record.get('trial_valid') is True


def summarize_records(records,total):
    latest={r['scenario']:r for r in records}
    valid=[r for r in latest.values() if r['trial_valid']]
    completed=[r for r in latest.values() if condition_completed(r)]
    return {'attempted_unique':len(latest),'pending_unattempted':total-len(latest),
        'route_too_short':sum('insufficient_route_length' in r.get('invalid_reasons',[]) for r in latest.values()),
        'not_established':sum(any(reason.startswith('stable_following_not_established') for reason in r.get('failure_reasons',[])+r.get('invalid_reasons',[])) for r in latest.values()),
        'completed':len(completed),'completed_valid':len(valid),
        'pass':sum(r['pass'] is True for r in completed),'fail':sum(r['pass'] is False for r in completed),
        'failed_incomplete':sum(not r['trial_valid'] for r in completed),
        'invalid_trials':sum(not r['trial_valid'] for r in latest.values()),
        'invalid':len(latest)-len(completed),'remaining':total-len(completed)}


def expand(args,cfg):
    from stf_weather_catalog import build_weather_catalog
    baseline=json.loads(Path(args.baseline).read_text())
    check=dict(baseline);fingerprint=check.pop('baseline_fingerprint')
    if digest(check)!=fingerprint:raise ValueError('baseline fingerprint mismatch')
    max_speed=max(cfg['test_speeds_kmh'])
    required=minimum_route_length(max_speed,baseline['initial_center_gap_m'],cfg['engineering'])
    plans={v:route_feasibility(v,baseline['initial_center_gap_m'],baseline['route_length_m'],cfg['engineering']) for v in cfg['test_speeds_kmh']}
    print('5d moving-start route plans:', json.dumps(plans,ensure_ascii=False))
    output=Path(args.input_dir or cfg['input_dir'])
    if not args.dry_run and output.exists() and any(output.iterdir()):raise ValueError('output is not empty; choose a new --input_dir')
    expected_count=len(cfg['test_speeds_kmh'])*sum(len(ps) for ps in build_weather_catalog().values())
    definitions=[]
    for i,v in enumerate(cfg['test_speeds_kmh'],1):
        for name,profiles in build_weather_catalog().items():
            for j,weather in enumerate(profiles,1):
                d={'schema_version':2,'scenario_code':'5d','scenario':f'scenario_5d_speed_{i:02d}_{name}_{j:02d}','baseline':baseline,'test_speed_kmh':v,'speed_band':speed_band(v),'route_plan':plans[v],'weather_id':f'{name}_{j:02d}','weather':weather,'engineering':copy.deepcopy(cfg['engineering']),'screening_only':True,'physical_field_test_compliance_claimed':False}
                d['condition_fingerprint']=digest(d);definitions.append(d)
    if len(definitions)!=expected_count or len({d['condition_fingerprint'] for d in definitions})!=expected_count:raise ValueError('incorrect unique condition count')
    if not args.dry_run:
        for d in definitions:write_json(output/(d['scenario']+'.json'),d)
        write_json(output/'manifest.json',{'count':expected_count,'baseline_fingerprint':fingerprint,'schema_version':2,'test_speeds_kmh':cfg['test_speeds_kmh'],'conditions':{d['scenario']:d['condition_fingerprint'] for d in definitions}})
    print('5d: {} speeds x 120 weather = {} conditions; '.format(len(cfg['test_speeds_kmh']),expected_count)+('dry run only' if args.dry_run else str(output)))


def validate_definitions(paths,cfg,world):
    map_hash=hashlib.sha256(world.get_map().to_opendrive().encode()).hexdigest();definitions=[];seen=set();geometries=set()
    for p in paths:
        d=json.loads(p.read_text());raw=dict(d);fp=raw.pop('condition_fingerprint')
        if digest(raw)!=fp:raise ValueError('condition fingerprint mismatch: '+str(p))
        if d['schema_version']!=2 or d['scenario_code']!='5d':raise ValueError('not a 5d definition')
        if d['scenario']!=p.stem or d['scenario'] in seen:raise ValueError('duplicate/misnamed condition')
        if d['baseline']['opendrive_sha256']!=map_hash:raise ValueError('CARLA map mismatch')
        if d['test_speed_kmh'] not in cfg['test_speeds_kmh'] or d['engineering']!=cfg['engineering']:raise ValueError('settings changed; regenerate definitions in new directory')
        if d.get('speed_band') != speed_band(d['test_speed_kmh']):raise ValueError('incorrect engineering speed band')
        b=d['baseline'];rawb=dict(b);bfp=rawb.pop('baseline_fingerprint')
        if digest(rawb)!=bfp:raise ValueError('baseline fingerprint mismatch')
        if bfp not in geometries:
            route,gap=validate_geometry(world.get_map(),b,cfg['engineering'])
            from emergency_brake_5d import project
            if abs(project(route[-1],route)['length']-b['route_length_m'])>.1 or abs(gap-b['initial_center_gap_m'])>.1:raise ValueError('baseline geometry measurements do not match map')
            geometries.add(bfp)
        if d.get('route_plan')!=route_feasibility(d['test_speed_kmh'],b['initial_center_gap_m'],b['route_length_m'],cfg['engineering']):raise ValueError('route plan mismatch; regenerate 5d definitions')
        seen.add(d['scenario']);definitions.append(d)
    from stf_weather_catalog import build_weather_catalog
    catalog={f'{n}_{i:02d}':w for n,profiles in build_weather_catalog().items() for i,w in enumerate(profiles,1)}
    if len(geometries)!=1 or len(definitions)!=len(cfg['test_speeds_kmh'])*len(catalog):raise ValueError('run requires one complete configured speed/weather matrix')
    expected={(v,w) for v in cfg['test_speeds_kmh'] for w in catalog}
    if {(d['test_speed_kmh'],d['weather_id']) for d in definitions}!=expected:raise ValueError('incomplete matrix')
    for d in definitions:
        if d['weather']!=catalog[d['weather_id']]:raise ValueError('weather profile mismatch')
    return definitions


def run(args,cfg):
    from contextlib import ExitStack
    with ExitStack() as locks:
        return _run_locked(args,cfg,locks)


def _run_locked(args,cfg,locks):
    import carla
    import fcntl
    import imageio.v2 as imageio
    from emergency_brake_5d_runtime import BrakeScene
    model=args.model
    incomplete_as_failed=getattr(args,'incomplete_as_failed',False)
    if incomplete_as_failed and model!='tcp':raise ValueError('--incomplete-as-failed requires --model tcp')
    result_policy='incomplete_as_failed' if incomplete_as_failed else 'valid_trials_only'
    declared_vmax=cfg.get('ads_vmax_kmh',{}).get(model)
    connection_lock=locks.enter_context(open('/tmp/stf_5d_%s_%s.lock'%((args.host or cfg['host']).replace('/','_'),args.port or cfg['port']),'w'))
    fcntl.flock(connection_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    checkpoint=Path(args.model_path or cfg['model_path']).resolve()
    if model=='tcp' and not checkpoint.is_file():raise ValueError('TCP checkpoint not found')
    client=carla.Client(args.host or cfg['host'],args.port or cfg['port']);client.set_timeout(20)
    world=client.get_world()
    if args.town and world.get_map().name.rsplit('/',1)[-1]!=args.town.rsplit('/',1)[-1]:raise ValueError('--town differs from current map')
    definitions=validate_definitions(sorted(Path(args.input_dir or cfg['input_dir']).glob('scenario_5d_*.json')),cfg,world)
    if getattr(args,'weather',None):
        definitions=[d for d in definitions if d['weather_id']==args.weather]
        if not definitions:raise ValueError('unknown --weather profile')
    if getattr(args,'speeds',None):
        selected=set(args.speeds)
        if not selected<=set(cfg['test_speeds_kmh']):raise ValueError('--speeds contains unconfigured speeds')
        definitions=[d for d in definitions if d['test_speed_kmh'] in selected]
    total=len(definitions)
    if args.dry_run:print('5d geometry, map, hashes and {}-condition matrix validated; no actors spawned'.format(total));return
    if list(world.get_actors().filter('vehicle.*')):raise RuntimeError('world contains vehicles; refusing to interfere')
    out=Path(args.output_dir or args.video_dir or (Path(cfg['output_root'])/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+model)))
    if args.resume and not args.output_dir and not args.video_dir:raise ValueError('--resume requires original --output_dir')
    out.mkdir(parents=True,exist_ok=True);lock=locks.enter_context((out/'run.lock').open('w'));fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ledger=out/'5d_result.json';records=[]
    if ledger.exists():
        if not args.resume:raise ValueError('existing result directory: use --resume or new --output_dir')
        records=json.loads(ledger.read_text())['records']
    code_hash=digest({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'emergency_brake_5d.py',ROOT/'emergency_brake_5d_runtime.py',ROOT/'emergency_brake_5d_cli.py',ROOT/'model/stf_tcp.py',ROOT/'stf_vehicle_control.py',ROOT/'render.py',ROOT/'emergency_brake_5d_video.py']})
    checkpoint_hash=hashlib.sha256(checkpoint.read_bytes()).hexdigest() if model=='tcp' else None
    identities={d['scenario']:d['condition_fingerprint'] for d in definitions}
    for r in records:
        if r['condition_fingerprint']!=identities.get(r['scenario']) or r['controller']!=model or r['runtime_sha256']!=code_hash or r['checkpoint_sha256']!=checkpoint_hash or r.get('declared_vmax_kmh')!=declared_vmax or r.get('result_policy','valid_trials_only')!=result_policy:raise ValueError('resume identity mismatch')
    done={r['scenario'] for r in records if condition_completed(r)}
    original=world.get_settings();weather=world.get_weather();settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.02
    status={'status':'running','total':total,'model':model,'result_policy':result_policy,'started_at_utc':datetime.now(timezone.utc).isoformat(),'fixed_delta_seconds':.02,'runtime_sha256':code_hash,'checkpoint_sha256':checkpoint_hash,'carla_version':client.get_server_version(),'screening_only':True,'collision_enhancement_enabled':False,'declared_vmax_kmh':declared_vmax}
    def persist():
        latest={r['scenario']:r for r in records}
        summary=summarize_records(records,total)
        write_json(ledger,{'metadata':status,'summary':summary,'records':records})
        write_json(out/'progress_latest.json',dict(status,**summary,checked_at_utc=datetime.now(timezone.utc).isoformat()))
        (out/'5d_result.md').write_text('# 5d 单次仿真筛选\n\n判定策略：'+result_policy+'\n\n'+json.dumps(summary,ensure_ascii=False)+'\n\n|场景|结果|原因|\n|---|---|---|\n'+'\n'.join('|{}|{}|{}|'.format(r['scenario'],r.get('result_status','INVALID' if not r['trial_valid'] else 'PASS' if r['pass'] else 'FAILED'), '; '.join(r['failure_reasons']+r['invalid_reasons'])) for r in latest.values()))
    try:
        world.apply_settings(settings)
        persist()
        for d in definitions:
            if d['scenario'] in done:continue
            scene=None;writer=None;record=None;viz=None;video_path=None;video_frames=0;video_start=None;video_end=None
            print('START',d['scenario'],'completed',len(done),'/'+str(total),flush=True)
            try:
                world.set_weather(carla.WeatherParameters(**d['weather']))
                scene=BrakeScene(world,d,model,str(checkpoint));scene.spawn(video=not args.no_video)
                attempt=1+sum(r['scenario']==d['scenario'] for r in records)
                video_path=out/(d['scenario']+f'_attempt_{attempt}.mp4')
                if not args.no_video:
                    from emergency_brake_5d_video import StandardVideo
                    viz=StandardVideo(world,scene.ego,headless=args.headless)
                    writer=imageio.get_writer(str(video_path),fps=20,codec='libx264',pixelformat='yuv420p')
                while True:
                    snap=world.get_snapshot();running=scene.step()
                    if writer:
                        rgb=scene.video.wait(snap.frame,snap.timestamp.elapsed_seconds,cfg['engineering']['camera_timeout_s'])
                        timestamp=float(snap.timestamp.elapsed_seconds)
                        if video_start is None:video_start=timestamp
                        if timestamp-video_start+1e-8>=video_frames/20.:
                            writer.append_data(viz.frame(rgb,scene.route))
                            video_frames+=1;video_end=timestamp
                    if not running:break
                    world.tick()
                record=scene.finish()
            except RouteLengthError as exc:
                record={'trial_valid':False,'pass':None,'failure_reasons':[],'invalid_reasons':['insufficient_route_length'],'detail':str(exc)}
            except Exception as exc:
                if scene:
                    scene.trial.invalid.append(type(exc).__name__+': '+str(exc));scene.trial.phase='INVALID'
                    try:record=scene.finish()
                    except Exception:record=None
                if record is None:record={'trial_valid':False,'pass':None,'failure_reasons':[],'invalid_reasons':[type(exc).__name__+': '+str(exc)]}
            finally:
                try:
                    if writer:writer.close()
                finally:
                    if scene:scene.destroy()
                    if viz:viz.close()
            record.update(scenario=d['scenario'],condition_fingerprint=d['condition_fingerprint'],controller=model,checkpoint_sha256=checkpoint_hash,runtime_sha256=code_hash,test_speed_kmh=d['test_speed_kmh'],speed_band=d['speed_band'],weather_id=d['weather_id'],**capability_labels(d['test_speed_kmh'],declared_vmax))
            record.update(video_path=str(video_path.resolve()) if video_frames else None,
                          video_metadata={'fps':20,'width':512,'height':256,'codec':'h264',
                                          'layout':'BEV + front camera','frame_count':video_frames,
                                          'start_sim_time':video_start,'end_sim_time':video_end,
                                          'encoded_duration_s':video_frames/20.})
            apply_result_policy(record,incomplete_as_failed)
            records.append(record)
            if condition_completed(record):done.add(d['scenario'])
            persist();print('RESULT',record['scenario'],record['result_status'],
                            record['failure_reasons']+record['invalid_reasons'],flush=True)
            # Stop on infrastructure/controller INVALID: avoid producing hundreds of unusable trials.
            if not condition_completed(record) and not getattr(args,'continue_invalid',False) and record['invalid_reasons'] not in (['stable_following_not_established'],['insufficient_route_length']):status['status']='interrupted';break
        else:status['status']='completed' if len(done)==total else 'incomplete'
    except BaseException:
        status['status']='interrupted';raise
    finally:
        try:
            try:world.set_weather(weather)
            finally:world.apply_settings(original)
        finally:
            status['finished_at_utc']=datetime.now(timezone.utc).isoformat();persist()
    print('OUTPUT',out,flush=True)


def main(mode,argv=None):
    parser=argparse.ArgumentParser(description='5d dedicated '+mode)
    parser.add_argument('--config',default=str(ROOT/'config/emergency_brake_5d.yaml'))
    parser.add_argument('--host');parser.add_argument('--port',type=int);parser.add_argument('--town');parser.add_argument('--name')
    parser.add_argument('--save_dir');parser.add_argument('--input_dir');parser.add_argument('--output_dir');parser.add_argument('--video_dir')
    parser.add_argument('--baseline');parser.add_argument('--model',choices=('behavior','tcp'),default='behavior');parser.add_argument('--model_path')
    parser.add_argument('--weather',help='Run one weather profile across all configured speeds, e.g. sunny_07')
    parser.add_argument('--speeds',type=float,nargs='+',help='Diagnostic subset of configured speeds')
    parser.add_argument('--continue-invalid',action='store_true',help='Record INVALID attempts and continue the selected diagnostic batch')
    parser.add_argument('--incomplete-as-failed',action='store_true',help='TCP: score every attempted incomplete/invalid condition as FAILED and continue; retain raw trial diagnostics')
    parser.add_argument('--resume',action='store_true');parser.add_argument('--dry-run',action='store_true');parser.add_argument('--no-video',action='store_true');parser.add_argument('--headless',action='store_true');parser.add_argument('--screening',action='store_true')
    args=parser.parse_args(argv)
    if args.incomplete_as_failed and (mode!='run' or args.model!='tcp'):
        parser.error('--incomplete-as-failed requires run mode and --model tcp')
    os.chdir(ROOT)
    cfg=load_settings(args.config,require_speeds=mode!='editor')
    if mode=='editor':
        from emergency_brake_5d_editor import BrakeEditor
        BrakeEditor(args,cfg).run()
    elif mode=='expand':
        if not args.baseline:parser.error('--baseline is required')
        expand(args,cfg)
    elif mode=='run':run(args,cfg)
    return 0
