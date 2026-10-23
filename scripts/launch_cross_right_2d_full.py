#!/usr/bin/env python3
"""Run the measured 2d matrix and its scheduled TCP continuation."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from cross_right_2d import digest
from cross_right_2d_cli import write_json


def now():
    return datetime.now(timezone.utc).isoformat()


def latest(ledger):
    return {r['scenario']: r for r in ledger.get('records', [])}


def completion(ledger, expected, controller):
    """Execution completion is separate from algorithm PASS/FAILED/INVALID."""
    records = latest(ledger)
    if ledger.get('metadata', {}).get('status') not in ('completed', 'incomplete'):
        return None
    if set(records) != set(expected):
        return None
    for name, r in records.items():
        if (r.get('condition_fingerprint') != expected[name]
                or r.get('controller') != controller
                or r.get('result_status') not in ('PASS', 'FAILED', 'INVALID')
                or 'cleanup' not in r
                or r['cleanup'].get('remaining_actor_ids') or r['cleanup'].get('error')):
            return None
    return 'completed_with_invalid' if any(r['result_status'] == 'INVALID' for r in records.values()) else 'completed'


def remaining_behavior(plan, pilot):
    expected = plan['expected_conditions']
    done = set()
    for name, r in latest(pilot).items():
        if (name not in expected or r['condition_fingerprint'] != expected[name]
                or r['controller'] != 'behavior'):
            raise ValueError('Timing pilot differs from the full-run conditions')
        if r['result_status'] in ('PASS', 'FAILED'):
            done.add(name)
    return {name: fp for name, fp in expected.items() if name not in done}


def estimate_remaining(samples, remaining_speeds, startup_seconds=0.):
    """Interpolate unsampled speeds by travel time (1/speed), not by counts."""
    measured = {float(s['speed_kmh']): float(s['wall_seconds']) for s in samples}
    if len(samples) != 6 or len(measured) != 6 or min(measured.values()) <= 0:
        raise ValueError('Six distinct measured speeds with positive durations are required')
    speeds = sorted(measured)
    predicted = {}
    counts = Counter(remaining_speeds)
    for speed in sorted(counts):
        if speed in measured:
            predicted[speed] = measured[speed]
            continue
        lower = max(v for v in speeds if v < speed)
        upper = min(v for v in speeds if v > speed)
        weight = (1./speed - 1./upper)/(1./lower - 1./upper)
        predicted[speed] = measured[lower]*weight + measured[upper]*(1.-weight)
    total = max(0., startup_seconds)+sum(counts[v]*predicted[v] for v in counts)
    return dict(remaining_wall_seconds=total, predicted_seconds_per_speed=predicted,
                remaining_counts_per_speed=dict(counts), startup_seconds=max(0., startup_seconds),
                method='Measured wall time including video/cleanup; interpolate missing speeds in 1/speed')


def carla_idle(plan):
    """Read the server only; never tick, change maps, or delete foreign actors."""
    lock_path = Path('/tmp/stf_2d_'+digest([plan['host'], plan['port']])[:16]+'.lock')
    with lock_path.open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False, '2d runner still holds the CARLA endpoint'
        import carla
        client = carla.Client(plan['host'], plan['port'])
        client.set_timeout(10.)
        world = client.get_world()
        if world.get_settings().synchronous_mode:
            return False, 'CARLA is still in synchronous mode'
        world.wait_for_tick(5.)  # Populate this client's actor snapshot.
        if world.get_map().name != plan['map_name']:
            return False, 'CARLA current map differs from the planned map'
        occupied = [a.id for a in world.get_actors()
                    if a.type_id.startswith(('vehicle.', 'walker.', 'sensor.'))]
        if occupied:
            return False, 'CARLA still contains test actors: '+str(occupied)
    return True, 'CARLA is idle'


def behavior_ready(plan, pilot):
    path = Path(plan['behavior_output'])
    status_file = path/'launch_status.json'
    if not status_file.exists():
        return False, 'Behavior has not started'
    state = json.loads(status_file.read_text())
    if state.get('state') not in ('completed', 'completed_with_invalid'):
        return False, 'Behavior has not finished: '+str(state.get('state'))
    ledger_path = path/'2d_result.json'
    if not ledger_path.exists():
        return False, 'Behavior ledger is not yet available'
    full = json.loads(ledger_path.read_text())
    expected = remaining_behavior(plan, pilot)
    if completion(full, expected, 'behavior') is None:
        return False, 'Behavior has not cleanly attempted every remaining condition'
    merged = dict(latest(pilot), **latest(full))
    if set(merged) != set(plan['expected_conditions']):
        return False, 'Combined behavior coverage is not the full matrix'
    return True, 'All behavior conditions have finished'


def execute(plan, stage, pilot, status_path, state):
    output = Path(plan[stage+'_output'])
    expected = remaining_behavior(plan, pilot) if stage == 'behavior' else plan['expected_conditions']
    ledger_path = output/'2d_result.json'
    if ledger_path.exists():
        old = json.loads(ledger_path.read_text())
        done = completion(old, expected, stage)
        if done:
            state.update(state=done, reason='This stage has already finished', finished_at_utc=now())
            write_json(status_path, state)
            return 0
    command = [plan['python'], '-B', '-X', 'faulthandler', '-u', 'run.py', '-2d',
               '--config', plan['config'], '--input_dir', plan['input_dir'],
               '--model', stage, '--headless', '--continue-invalid',
               '--host', plan['host'], '--port', str(plan['port']), '--output_dir', str(output)]
    if stage == 'behavior':
        command += ['--exclude-completed-from', str(Path(plan['pilot_output'])/'2d_result.json')]
    else:
        command += ['--model_path', plan['checkpoint']]
    if ledger_path.exists():
        command.append('--resume')
    state.update(state='starting', command=command, expected_conditions=len(expected))
    write_json(status_path, state)
    interrupted = False
    with (output/'run.log').open('ab', buffering=0) as log:
        child = subprocess.Popen(command, cwd=plan['repository'], stdin=subprocess.DEVNULL,
                                 stdout=log, stderr=subprocess.STDOUT)
        state.update(state='running', child_pid=child.pid, started_at_utc=now())
        write_json(status_path, state)
        print(stage, 'running', child.pid, flush=True)
        try:
            returncode = child.wait()
        except KeyboardInterrupt:
            interrupted = True
            child.send_signal(signal.SIGINT)
            returncode = child.wait()
    ledger = json.loads(ledger_path.read_text()) if ledger_path.exists() else {}
    done = completion(ledger, expected, stage)
    state.update(state='interrupted' if interrupted else done or 'stopped',
                 exit_code=returncode, finished_at_utc=now(), summary=ledger.get('summary'),
                 runner_status=ledger.get('metadata', {}).get('status'))
    write_json(status_path, state)
    if stage == 'behavior' and done and not interrupted:
        merged = dict(latest(pilot), **latest(ledger))
        summary = {label: sum(r['result_status'] == label for r in merged.values())
                   for label in ('PASS', 'FAILED', 'INVALID')}
        write_json(Path(plan['root'])/'behavior_full_coverage.json',
                   dict(total=len(merged), summary=summary, pilot_ledger=str(Path(plan['pilot_output'])/'2d_result.json'),
                        remaining_ledger=str(ledger_path), conditions={n:r['result_status'] for n,r in merged.items()}))
    print(stage, state['state'], state.get('summary'), flush=True)
    return 0 if done and not interrupted else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--stage', choices=('behavior', 'tcp'), required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    pilot = json.loads((Path(plan['pilot_output'])/'2d_result.json').read_text())
    output = Path(plan[args.stage+'_output'])
    output.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    with (output/'launch.lock').open('a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        status_path = output/'launch_status.json'
        state = dict(state='waiting', stage=args.stage, worker_pid=os.getpid(),
                     created_at_utc=now(), plan=str(args.plan.resolve()))
        while True:
            try:
                ready, reason = behavior_ready(plan, pilot) if args.stage == 'tcp' else (True, 'Behavior requested')
                if ready:
                    ready, reason = carla_idle(plan)
            except Exception as exc:
                ready, reason = False, str(exc)
            state.update(state='waiting', reason=reason, checked_at_utc=now())
            write_json(status_path, state)
            if ready:
                break
            print(args.stage, 'waiting:', reason, flush=True)
            time.sleep(60.)
        return execute(plan, args.stage, pilot, status_path, state)


if __name__ == '__main__':
    raise SystemExit(main())
