"""Explicit entry for local runners, including the familiar --scenario syntax."""
from stf_runner_dispatch import dispatch

if __name__ == '__main__':
    raise SystemExit(dispatch(force_local=True))
