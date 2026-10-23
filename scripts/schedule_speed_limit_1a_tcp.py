#!/usr/bin/env python3
"""Launch TCP once, after the requested delay and completed Behavior batch."""
import argparse
from datetime import datetime, timedelta, timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def now():
    return datetime.now(timezone.utc)


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def completed(ledger, expected):
    latest = {r['scenario']: r for r in ledger.get('records', [])}
    known = {'PASS', 'FAILED', 'INVALID'}
    ready = (ledger.get('batch_status') == 'FINISHED'
             and ledger.get('total') == len(expected)
             and set(latest) == set(expected)
             and all(r.get('result_status') in known for r in latest.values()))
    return ready, {'batch_status': ledger.get('batch_status'),
                   'attempted_unique': len(latest), 'expected': len(expected),
                   'invalid': sum(r.get('result_status') == 'INVALID' for r in latest.values())}


def behavior_process_running(pid, output):
    try:
        args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        return str(output).encode() in args and b'behavior' in args
    except FileNotFoundError:
        return False


def check(config, expected):
    behavior = Path(config['behavior_output'])
    ledger = behavior / '1a_result.json'
    if not ledger.is_file():
        return False, 'Behavior result ledger not yet present', {}
    ready, details = completed(json.loads(ledger.read_text()), expected)
    if not ready:
        return False, 'Behavior has not normally finished every condition', details
    if behavior_process_running(config['behavior_pid'], behavior):
        return False, 'Behavior process has not exited yet', details
    # The same endpoint lock used by run.py prevents overlap with another 1a.
    lock_path = Path(f"/tmp/stf_1a_{config['host'].replace('/', '_')}_{config['port']}.lock")
    with lock_path.open('a+') as endpoint_lock:
        try:
            fcntl.flock(endpoint_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False, 'CARLA endpoint is still reserved by a 1a runner', details
        import carla
        client = carla.Client(config['host'], config['port'])
        client.set_timeout(10.)
        world = client.get_world()
        if world.get_settings().synchronous_mode:
            return False, 'CARLA synchronous clock is still occupied', details
        actors = world.get_actors()
        occupied = [a.id for a in actors if a.type_id.startswith(('vehicle.', 'walker.', 'sensor.'))]
        if occupied:
            details['occupied_actor_ids'] = occupied
            return False, 'CARLA test actors have not been cleared', details
        if world.get_map().name.rsplit('/', 1)[-1] != '1a':
            return False, 'CARLA is not on the 1a map', details
    if not Path(config['checkpoint']).is_file():
        return False, 'TCP checkpoint is missing', details
    return True, 'Behavior finished; CARLA is idle', details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    output = Path(config['tcp_output'])
    output.mkdir(parents=True, exist_ok=True)
    status_path = output / 'scheduled_launch.json'
    expected = config['expected_scenarios']
    target = datetime.fromisoformat(config['not_before_utc'])
    if target.tzinfo is None or len(expected) != 960 or len(set(expected)) != 960:
        raise ValueError('requires a timezone-aware start and 960 unique expected scenarios')
    retry = float(config['retry_interval_seconds'])
    if retry != 1800:
        raise ValueError('this schedule requires the requested 30-minute retry interval')
    with (output / 'schedule.lock').open('a+') as schedule_lock:
        fcntl.flock(schedule_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if (output / 'tcp_launch_claim.json').exists() or (output / '1a_result.json').exists():
            print('TCP launch already claimed or results exist; refusing a duplicate launch', flush=True)
            return 0
        state = {'state': 'waiting', 'scheduler_pid': os.getpid(),
                 'created_at_utc': config['created_at_utc'],
                 'not_before_utc': config['not_before_utc'],
                 'retry_interval_seconds': retry, 'checks': 0,
                 'behavior_output': config['behavior_output'], 'tcp_output': str(output)}
        while True:
            state.update(next_check_at_utc=target.isoformat(), updated_at_utc=now().isoformat())
            save(status_path, state)
            while now() < target:
                time.sleep(min(60., max(.1, (target - now()).total_seconds())))
            try:
                ready, reason, details = check(config, expected)
            except Exception as exc:
                ready, reason, details = False, f'{type(exc).__name__}: {exc}', {}
            state.update(checks=state['checks'] + 1, last_check_at_utc=now().isoformat(),
                         reason=reason, behavior_status=details)
            with (output / 'checks.jsonl').open('a') as log:
                log.write(json.dumps({'at_utc': now().isoformat(), 'ready': ready,
                                      'reason': reason, 'details': details}) + '\n')
            if ready:
                break
            state['state'] = 'waiting'
            target = now() + timedelta(seconds=retry)
            print('Deferred 30 minutes:', reason, flush=True)
        command = [config['python'], '-u', 'run.py', '-1a', '--model', 'tcp',
                   '--model_path', config['checkpoint'], '--input_dir', config['input_dir'],
                   '--host', config['host'], '--port', str(config['port']),
                   '--output_dir', str(output)]
        claim = output / 'tcp_launch_claim.json'
        with claim.open('x') as stream:
            json.dump({'claimed_at_utc': now().isoformat(), 'command': command}, stream, indent=2)
        state.update(state='starting', command=command, updated_at_utc=now().isoformat())
        save(status_path, state)
        with (output / 'run.log').open('ab', buffering=0) as log:
            process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        state.update(state='running', tcp_pid=process.pid, started_at_utc=now().isoformat())
        save(status_path, state)
        save(output / 'launch.json', {'pid': process.pid, 'command': command,
             'output_dir': str(output), 'launched_at_utc': state['started_at_utc'],
             'expected_conditions': len(expected), 'video_enabled': True})
        while process.poll() is None:
            time.sleep(30.)
        state.update(exit_code=process.returncode, finished_at_utc=now().isoformat())
        ledger_path = output / '1a_result.json'
        done, details = (completed(json.loads(ledger_path.read_text()), expected)
                         if ledger_path.exists() else (False, {}))
        state.update(state=('finished' if done and process.returncode == 0 else 'stopped'),
                     tcp_status=details, updated_at_utc=now().isoformat())
        save(status_path, state)
        return process.returncode


if __name__ == '__main__':
    raise SystemExit(main())
