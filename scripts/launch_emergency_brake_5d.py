#!/usr/bin/env python3
"""Usage: launch_emergency_brake_5d.py editor|expand|run [options]."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from emergency_brake_5d_cli import main
if __name__=='__main__':
    if len(sys.argv)<2 or sys.argv[1] not in ('editor','expand','run'):
        raise SystemExit('usage: launch_emergency_brake_5d.py editor|expand|run [options]')
    raise SystemExit(main(sys.argv[1],sys.argv[2:]))
