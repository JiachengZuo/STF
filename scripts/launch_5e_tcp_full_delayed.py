#!/usr/bin/env python3
"""Run the scheduled 5e TCP matrix once the local CARLA test world is idle."""

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from curve_emergency_5e import digest, load_settings


PYTHON = Path('/home/fsm/miniconda3/envs/carla0916/bin/python')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def save_status(path, **values):
    current = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    current.update(values, updated_at_utc=utc_now())
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(current, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def lock_is_busy(path):
    with path.open('a+') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(stream, fcntl.LOCK_UN)
    return False


def completion_state(returncode, run_status, summary):
    """Distinguish a fully attempted batch with invalid trials from a stopped run."""
    if returncode != 0 or not summary:
        return 'stopped'
    all_attempted = summary.get('attempted') == summary.get('total')
    if run_status == 'completed' and all_attempted and summary.get('INVALID', 0) == 0:
        return 'completed'
    if (run_status == 'incomplete' and all_attempted and summary.get('INVALID', 0) > 0
            and sum(summary.get(k, 0) for k in ('PASS', 'FAILED', 'INVALID')) == summary['total']):
        return 'completed_with_invalid'
    return 'stopped'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--resume', action='store_true', help='Retry INVALID and unattempted conditions in an existing ledger')
    parser.add_argument('--exclude-completed-from', type=Path,
                        help='Prior stopped ledger to retain separately when evaluation rules change')
    parser.add_argument('--continue-invalid', action='store_true', default=True,
                        help='Record INVALID conditions and continue (default for full runs)')
    args = parser.parse_args()

    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    status_path = output / 'scheduled_launch.json'
    log_path = output / 'launch.log'
    ledger_path = output / '5e_result.json'
    config = load_settings(ROOT / 'config/curve_emergency_5e.yaml')
    global_lock = Path('/tmp/stf_5e_' + digest([config['host'], config['port']])[:16] + '.lock')
    save_status(status_path, state='waiting_for_carla', worker_pid=os.getpid())

    while lock_is_busy(global_lock):
        save_status(status_path, state='waiting_for_carla', reason='another 5e run holds the CARLA lock')
        time.sleep(60)

    if args.resume and not ledger_path.exists():
        save_status(status_path, state='blocked', reason='No result ledger to resume')
        return 1
    if ledger_path.exists() and not args.resume:
        save_status(status_path, state='already_started', reason='TCP result ledger already exists')
        return 0
    if not args.checkpoint.is_file():
        save_status(status_path, state='blocked', reason='TCP checkpoint missing')
        return 1

    dry_run = [str(PYTHON), 'run.py', '-5e', '--model', 'tcp', '--dry-run']
    dry_run.extend(['--model_path', str(args.checkpoint.resolve())])
    if args.exclude_completed_from:
        dry_run.extend(['--exclude-completed-from', str(args.exclude_completed_from.resolve())])
    checked = subprocess.run(dry_run, cwd=ROOT, capture_output=True, text=True)
    if checked.returncode or 'complete 960-condition matrix verified;' not in checked.stdout:
        save_status(status_path, state='blocked', reason='960-condition dry run failed',
                    dry_run_stdout=checked.stdout[-1000:], dry_run_stderr=checked.stderr[-1000:])
        return 1

    try:
        import carla
        client = carla.Client(config['host'], config['port'])
        client.set_timeout(20)
        actors = client.get_world().get_actors()
        occupied = [a.id for a in actors if a.type_id.startswith(('vehicle.', 'walker.', 'sensor.'))]
    except Exception as exc:
        save_status(status_path, state='blocked', reason='CARLA unavailable: ' + str(exc))
        return 1
    if occupied:
        save_status(status_path, state='blocked', reason='CARLA still contains traffic or sensors',
                    occupied_actor_ids=occupied)
        return 1

    command = [str(PYTHON), '-u', 'run.py', '-5e', '--model', 'tcp',
               '--model_path', str(args.checkpoint.resolve()), '--headless',
               '--output_dir', str(output)]
    if args.exclude_completed_from:
        command.extend(['--exclude-completed-from', str(args.exclude_completed_from.resolve())])
    if args.resume:
        command.append('--resume')
    if args.continue_invalid:
        command.append('--continue-invalid')
    environment = os.environ.copy()
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    save_status(status_path, state='starting', command=command,
                resume=args.resume, continue_invalid=args.continue_invalid,
                reason=None, summary=None, run_status=None, exit_code=None,
                finished_at_utc=None,
                policy='Record INVALID and continue' if args.continue_invalid else 'Stop on INVALID')
    with log_path.open('a', encoding='utf-8') as log:
        log.write('TCP 5e full matrix starting at ' + utc_now() + '\n')
        log.flush()
        save_status(status_path, state='running', started_at_utc=utc_now())
        result = subprocess.run(command, cwd=ROOT, env=environment,
                                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)

    summary = None
    run_status = None
    if ledger_path.exists():
        ledger = json.loads(ledger_path.read_text(encoding='utf-8'))
        summary = ledger.get('summary')
        run_status = ledger.get('metadata', {}).get('status')
    state = completion_state(result.returncode, run_status, summary)
    unattempted = max(0, summary['total'] - summary['attempted']) if summary else None
    invalid = summary.get('INVALID', 0) if summary else None
    save_status(status_path, state=state, exit_code=result.returncode,
                run_status=run_status, summary=summary, unattempted=unattempted,
                invalid_conditions=invalid, finished_at_utc=utc_now())
    # The ledger retains the runner's validity status and original trial evidence.
    # Expose execution completion separately so INVALID is not mistaken for a crash.
    progress_path = output / 'progress_latest.json'
    if progress_path.exists():
        progress = json.loads(progress_path.read_text(encoding='utf-8'))
        progress.update(execution_state=state, runner_status=run_status,
                        unattempted=unattempted, invalid_conditions=invalid)
        if state == 'completed_with_invalid':
            progress['status'] = state
        temporary = progress_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(progress, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(progress_path)
    return 0 if state in ('completed', 'completed_with_invalid') else 1


if __name__ == '__main__':
    raise SystemExit(main())
