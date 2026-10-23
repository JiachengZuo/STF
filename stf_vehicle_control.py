"""Private copy of the local action decoder; legacy decoder is unchanged."""
import math

def decode_vehicle_action(action, *, preserve_throttle=False):
    """Decode the first ``[throttle, steer, brake]`` row with safe limits.

    Plain lists/tuples and numpy-like values exposing ``tolist()`` are
    accepted.  Malformed or non-finite controller output is rejected instead
    of being silently applied to a CARLA actor.
    """
    if hasattr(action, "tolist"):
        action = action.tolist()
    if not isinstance(action, (list, tuple)) or not action:
        raise ValueError("vehicle action must contain a control row")
    first = action[0]
    row = first if isinstance(first, (list, tuple)) else action
    if len(row) < 3:
        raise ValueError("vehicle action must contain throttle, steer and brake")
    try:
        throttle, steer, brake = (float(row[index]) for index in range(3))
    except (TypeError, ValueError):
        raise ValueError("vehicle action values must be finite numbers")
    if not all(math.isfinite(value) for value in (throttle, steer, brake)):
        raise ValueError("vehicle action values must be finite numbers")
    throttle = max(0.0, min(1.0, throttle))
    steer = max(-1.0, min(1.0, steer))
    brake = max(0.0, min(1.0, brake))
    if brake > 0.05 and not preserve_throttle:
        throttle = 0.0
    return throttle, steer, brake
