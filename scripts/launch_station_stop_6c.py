#!/usr/bin/env python3
"""6c editor / expand / run; also available through waypoints.py/run.py -6c."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from station_stop_6c_cli import main

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in ('editor', 'expand', 'run'):
        raise SystemExit('usage: launch_station_stop_6c.py editor|expand|run [options]')
    raise SystemExit(main(sys.argv[1], sys.argv[2:]))
