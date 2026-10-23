#!/usr/bin/env python3
"""5e editor / expand / run, also reachable through waypoints.py/run.py -5e."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from curve_emergency_5e_cli import main

if __name__ == '__main__':
    if len(sys.argv)<2 or sys.argv[1] not in ('editor','expand','run'):
        raise SystemExit('usage: launch_curve_emergency_5e.py editor|expand|run [options]')
    raise SystemExit(main(sys.argv[1],sys.argv[2:]))
