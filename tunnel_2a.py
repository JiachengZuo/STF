"""Pure helpers for the GB/T 41798-2022 2.a tunnel screening scene.

The module intentionally has no CARLA dependency.  It owns the portable
scenario schema, route/gate geometry, weather-set validation and durable
screening result artifacts used by the editor, launcher and runtime scene.
"""

from __future__ import annotations

import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import pickle

from stf_weather_catalog import all_weather_profiles


SCHEMA_VERSION = 1
ROUTE_PROFILE = "tunnel_pass_through"
CAPABILITY = "pass_through"
GEOMETRY_SOURCE = "operator_marked_route_anchors"
MIN_TUNNEL_LENGTH_M = 100.0


def canonical_hash(value):
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _number(value, path):
    if isinstance(value, bool):
        raise ValueError("{} must be a finite number".format(path))
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError("{} must be a finite number".format(path))
    if not math.isfinite(value):
        raise ValueError("{} must be a finite number".format(path))
    return value


def _point_xy(point, path="point"):
    if not isinstance(point, dict):
        raise ValueError("{} must be an object".format(path))
    return (
        _number(point.get("x"), path + ".x"),
        _number(point.get("y"), path + ".y"),
    )


def _route(config):
    route = config.get("ego_route") if isinstance(config, dict) else None
    if not isinstance(route, list):
        raise ValueError("ego_route must be an array")
    if len(route) < 4:
        raise ValueError(
            "ego_route must contain at least four points: approach, entry, "
            "exit and post-exit")
    for index, point in enumerate(route, 1):
        _point_xy(point, "ego_route[P{}]".format(index))
        _number(point.get("z"), "ego_route[P{}].z".format(index))
        _number(point.get("yaw"), "ego_route[P{}].yaw".format(index))
    return route


def route_length_2d(route, first_index=0, last_index=None):
    """Return 2-D polyline length over an inclusive zero-based point range."""
    if not isinstance(route, list) or not route:
        return 0.0
    if last_index is None:
        last_index = len(route) - 1
    if first_index < 0 or last_index >= len(route) or first_index > last_index:
        raise ValueError("invalid inclusive route index range")
    length = 0.0
    previous = _point_xy(route[first_index], "route[{}]".format(first_index))
    for index in range(first_index + 1, last_index + 1):
        current = _point_xy(route[index], "route[{}]".format(index))
        length += math.hypot(current[0] - previous[0], current[1] - previous[1])
        previous = current
    return length


def route_distance_diagnostics(route, entry_number, exit_number):
    """Explain the exact XY segments included by a length validation failure."""
    lines = ["XY distance breakdown:"]
    for number in range(entry_number, exit_number):
        length = route_length_2d(route, number - 1, number)
        lines.append("P{} -> P{}: {:.4f} m".format(number, number + 1, length))
    total = route_length_2d(route, entry_number - 1, exit_number - 1)
    direct = route_length_2d([route[exit_number - 1], route[entry_number - 1]])
    lines.append("Sum (P{} -> P{}): {:.4f} m".format(entry_number, exit_number, total))
    lines.append("Direct (P{} -> P{}): {:.4f} m".format(exit_number, entry_number, direct))
    return "\n".join(lines)


def tunnel_route_direction_errors(ego_start, route):
    """Reject reverse-order anchors using their CARLA lane headings.

    This quick editor check is followed by actual lane-topology validation at
    save/expansion/runtime. Tight bends need extra anchors, not a reverse hop.
    """
    if not ego_start:
        return ["Place EGO before drawing the tunnel route"]
    points = [ego_start] + list(route)
    for index, (start, end) in enumerate(zip(points, points[1:])):
        a, b = _point_xy(start), _point_xy(end)
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        label = "{} -> P{}".format("EGO" if index == 0 else "P{}".format(index), index + 1)
        if length < 0.1:
            return [label + ": route points must be separated by at least 0.1 m"]
        for point in (start, end):
            yaw = math.radians(_number(point.get("yaw"), "lane yaw"))
            alignment = (dx * math.cos(yaw) + dy * math.sin(yaw)) / length
            if alignment < -0.1:
                return [label + ": point order opposes lane direction; follow lane arrows "
                        "and add intermediate points at bends"]
    return []


def _anchor_numbers(config):
    section = config.get("tunnel_test") if isinstance(config, dict) else None
    if not isinstance(section, dict):
        raise ValueError("tunnel_test must be an object")
    entry = section.get("entry_anchor_number")
    exit_ = section.get("exit_anchor_number")
    for name, value in (("entry_anchor_number", entry),
                        ("exit_anchor_number", exit_)):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("tunnel_test.{} must be a 1-based integer".format(name))
    return entry, exit_


def tunnel_route_payload(config):
    route = _route(config)
    entry, exit_ = _anchor_numbers(config)
    section = config.get("tunnel_test", {})
    return {
        "ego_start": config.get("ego_start"),
        "ego_route": route,
        "entry_anchor_number": entry,
        "exit_anchor_number": exit_,
        "route_profile": ROUTE_PROFILE,
        "map_name": section.get("map_name"),
        "opendrive_sha256": section.get("opendrive_sha256"),
    }


def tunnel_route_fingerprint(config):
    return canonical_hash(tunnel_route_payload(config))


def tunnel_condition_payload(config):
    section = config.get("tunnel_test", {})
    payload = {
        "route_fingerprint": tunnel_route_fingerprint(config),
        "weather_id": section.get("weather_id"),
        "weather": config.get("weather"),
        "scenario_timeout_s": section.get("scenario_timeout_s", 60.0),
    }
    if "expansion" in section:
        payload["expansion"] = section["expansion"]
    return payload


def tunnel_geometry_fingerprint(config):
    """Identity of the fixed operator-marked entrance, tunnel and exit tail."""
    entry, exit_ = _anchor_numbers(config)
    section = config["tunnel_test"]
    return canonical_hash({
        "route_from_entry": _route(config)[entry - 1:],
        "exit_offset": exit_ - entry,
        "map_name": section["map_name"],
        "opendrive_sha256": section["opendrive_sha256"],
    })


def validate_start_distances(values):
    if not isinstance(values, list) or not values:
        raise ValueError("start_distances_m must be a non-empty list")
    distances = [_number(value, "start_distances_m") for value in values]
    if any(value < 5.0 for value in distances):
        raise ValueError("start_distances_m must be at least 5 m (approach clearance)")
    if len(set(distances)) != len(distances):
        raise ValueError("start_distances_m must not contain duplicates")
    return sorted(distances)


def tunnel_condition_fingerprint(config):
    return canonical_hash(tunnel_condition_payload(config))


def make_route_gate(route, anchor_number, half_width_m=4.0):
    """Build a route-normal gate centered on a user-visible P-number."""
    if isinstance(anchor_number, bool) or not isinstance(anchor_number, int):
        raise ValueError("anchor_number must be a 1-based integer")
    index = anchor_number - 1
    if index <= 0 or index >= len(route) - 1:
        raise ValueError("gate anchor must have route points before and after it")
    before = _point_xy(route[index - 1], "route[P{}]".format(anchor_number - 1))
    center = _point_xy(route[index], "route[P{}]".format(anchor_number))
    after = _point_xy(route[index + 1], "route[P{}]".format(anchor_number + 1))
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        raise ValueError("route tangent is undefined at P{}".format(anchor_number))
    forward = (dx / norm, dy / norm)
    normal = (-forward[1], forward[0])
    half_width_m = _number(half_width_m, "half_width_m")
    if half_width_m <= 0.0:
        raise ValueError("half_width_m must be positive")
    return {
        "anchor_number": anchor_number,
        "center": {"x": center[0], "y": center[1]},
        "left": {
            "x": center[0] + normal[0] * half_width_m,
            "y": center[1] + normal[1] * half_width_m,
        },
        "right": {
            "x": center[0] - normal[0] * half_width_m,
            "y": center[1] - normal[1] * half_width_m,
        },
        "forward": {"x": forward[0], "y": forward[1]},
    }


def gate_signed_distance(point, gate):
    x, y = _point_xy(point)
    center = gate["center"]
    forward = gate["forward"]
    return ((x - float(center["x"])) * float(forward["x"])
            + (y - float(center["y"])) * float(forward["y"]))


def gate_crossing(previous_point, current_point, gate, tolerance_m=0.25):
    """Return true when motion crosses the finite gate in route direction."""
    previous = gate_signed_distance(previous_point, gate)
    current = gate_signed_distance(current_point, gate)
    if not (previous < 0.0 <= current):
        return False

    previous_xy = _point_xy(previous_point)
    current_xy = _point_xy(current_point)
    denominator = current - previous
    ratio = (-previous / denominator) if abs(denominator) > 1e-12 else 1.0
    px = previous_xy[0] + ratio * (current_xy[0] - previous_xy[0])
    py = previous_xy[1] + ratio * (current_xy[1] - previous_xy[1])
    left = _point_xy(gate["left"], "gate.left")
    right = _point_xy(gate["right"], "gate.right")
    vx, vy = right[0] - left[0], right[1] - left[1]
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-9:
        return False
    projection = ((px - left[0]) * vx + (py - left[1]) * vy) / length_sq
    margin = abs(float(tolerance_m)) / math.sqrt(length_sq)
    return -margin <= projection <= 1.0 + margin


def project_to_route(point, route):
    """Project a point to a route polyline and return distance/progress details."""
    px, py = _point_xy(point)
    if not isinstance(route, list) or len(route) < 2:
        raise ValueError("route projection requires at least two points")
    best = None
    progress_before = 0.0
    for index in range(len(route) - 1):
        ax, ay = _point_xy(route[index], "route[{}]".format(index))
        bx, by = _point_xy(route[index + 1], "route[{}]".format(index + 1))
        vx, vy = bx - ax, by - ay
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-12:
            continue
        length = math.sqrt(length_sq)
        ratio = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / length_sq))
        qx, qy = ax + ratio * vx, ay + ratio * vy
        distance = math.hypot(px - qx, py - qy)
        candidate = {
            "distance_m": distance,
            "progress_m": progress_before + ratio * length,
            "segment_index": index,
            "projected": {"x": qx, "y": qy},
        }
        if best is None or distance < best[0]:
            best = (distance, candidate)
        progress_before += length
    if best is None:
        raise ValueError("route projection has no non-zero segment")
    return best[1]


def tunnel_config_errors(config, require_fingerprints=True):
    errors = []
    try:
        route = _route(config)
    except ValueError as exc:
        return [str(exc)]

    ego_start = config.get("ego_start")
    try:
        _point_xy(ego_start, "ego_start")
        _number(ego_start.get("z"), "ego_start.z")
        _number(ego_start.get("yaw"), "ego_start.yaw")
    except ValueError as exc:
        errors.append(str(exc))

    try:
        errors.extend(tunnel_route_direction_errors(ego_start, route))
    except ValueError as exc:
        errors.append(str(exc))

    actors = config.get("other_actors", {})
    centers = actors.get("center", []) if isinstance(actors, dict) else None
    if centers not in (None, []):
        errors.append("2.a tunnel screening does not allow other_actors.center")

    section = config.get("tunnel_test")
    if not isinstance(section, dict):
        errors.append("tunnel_test must be an object")
        return errors
    if section.get("schema_version") != SCHEMA_VERSION:
        errors.append("tunnel_test.schema_version must be {}".format(SCHEMA_VERSION))
    if section.get("capability") != CAPABILITY:
        errors.append("tunnel_test.capability must be {!r}".format(CAPABILITY))
    if section.get("route_profile") != ROUTE_PROFILE:
        errors.append("tunnel_test.route_profile must be {!r}".format(ROUTE_PROFILE))
    if section.get("geometry_source") != GEOMETRY_SOURCE:
        errors.append(
            "tunnel_test.geometry_source must be {!r}".format(GEOMETRY_SOURCE))
    if section.get("screening_only") is not True:
        errors.append("tunnel_test.screening_only must be true")
    map_name = section.get("map_name")
    if not isinstance(map_name, str) or not map_name.strip():
        errors.append("tunnel_test.map_name must be a non-empty string")
    digest = section.get("opendrive_sha256")
    if (not isinstance(digest, str) or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest.lower())):
        errors.append("tunnel_test.opendrive_sha256 must be a SHA-256 hex digest")

    entry = section.get("entry_anchor_number")
    exit_ = section.get("exit_anchor_number")
    if isinstance(entry, bool) or not isinstance(entry, int):
        errors.append("tunnel_test.entry_anchor_number must be a 1-based integer")
    if isinstance(exit_, bool) or not isinstance(exit_, int):
        errors.append("tunnel_test.exit_anchor_number must be a 1-based integer")
    if isinstance(entry, int) and not isinstance(entry, bool) \
            and isinstance(exit_, int) and not isinstance(exit_, bool):
        if entry <= 1:
            errors.append("entry anchor must have at least one route point before it")
        if exit_ >= len(route):
            errors.append("exit anchor must have at least one route point after it")
        if entry >= exit_:
            errors.append("entry anchor must be before exit anchor")
        if 1 < entry < exit_ < len(route):
            try:
                length = route_length_2d(route, entry - 1, exit_ - 1)
                if length + 1e-6 < MIN_TUNNEL_LENGTH_M:
                    errors.append(
                        "operator-marked tunnel route is {:.2f} m; at least {:.0f} m "
                        "is required\n{}".format(length, MIN_TUNNEL_LENGTH_M,
                            route_distance_diagnostics(route, entry, exit_)))
                make_route_gate(route, entry)
                make_route_gate(route, exit_)
            except ValueError as exc:
                errors.append(str(exc))

    timeout = section.get("scenario_timeout_s", 60.0)
    try:
        if _number(timeout, "tunnel_test.scenario_timeout_s") <= 0.0:
            errors.append("tunnel_test.scenario_timeout_s must be positive")
    except ValueError as exc:
        errors.append(str(exc))

    weather_id = section.get("weather_id")
    if not isinstance(weather_id, str) or not weather_id.strip():
        errors.append("tunnel_test.weather_id must be a non-empty string")
    if not isinstance(config.get("weather"), dict):
        errors.append("weather must be an object")

    if require_fingerprints and not errors:
        expected_route = tunnel_route_fingerprint(config)
        expected_condition = tunnel_condition_fingerprint(config)
        if section.get("route_fingerprint") != expected_route:
            errors.append("tunnel_test.route_fingerprint does not match the route")
        if section.get("condition_fingerprint") != expected_condition:
            errors.append(
                "tunnel_test.condition_fingerprint does not match the full condition")
    return errors


def validate_tunnel_config(config, require_fingerprints=True):
    errors = tunnel_config_errors(config, require_fingerprints=require_fingerprints)
    if errors:
        raise ValueError("invalid 2.a tunnel config: " + "; ".join(errors))
    section = config["tunnel_test"]
    return {
        "route_fingerprint": tunnel_route_fingerprint(config),
        "condition_fingerprint": tunnel_condition_fingerprint(config),
        "weather_id": section["weather_id"],
        "operator_marked_tunnel_length_m": round(route_length_2d(
            config["ego_route"], section["entry_anchor_number"] - 1,
            section["exit_anchor_number"] - 1), 4),
    }


def validate_tunnel_screening_set(configs, require_all_weather=True):
    if not configs:
        raise ValueError("invalid 2.a screening set: no configurations found")
    expanded = ["expansion" in config.get("tunnel_test", {}) for config in configs]
    if any(expanded):
        if not all(expanded):
            raise ValueError("cannot mix baseline and expanded tunnel definitions")
        kinds = {config["tunnel_test"]["expansion"].get("kind") for config in configs
                 if isinstance(config["tunnel_test"]["expansion"], dict)}
        if "spawn_offsets" in kinds:
            if kinds != {"spawn_offsets"}:
                raise ValueError("cannot mix spawn-offset and distance expansions")
            return _validate_spawn_offset_set(configs, require_all_weather)
        return _validate_expanded_screening_set(configs, require_all_weather)
    summaries = [validate_tunnel_config(config) for config in configs]
    route_fingerprints = {item["route_fingerprint"] for item in summaries}
    if len(route_fingerprints) != 1:
        raise ValueError("invalid 2.a screening set: routes are not identical")
    weather_ids = [item["weather_id"] for item in summaries]
    if len(set(weather_ids)) != len(weather_ids):
        raise ValueError("invalid 2.a screening set: duplicate weather_id")
    if require_all_weather:
        expected_profiles = {
            item["id"]: item["parameters"] for item in all_weather_profiles()}
        expected = set(expected_profiles)
        actual = set(weather_ids)
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        if missing or unexpected:
            details = []
            if missing:
                details.append("missing {} weather profile(s)".format(len(missing)))
            if unexpected:
                details.append("unexpected weather ids: {}".format(
                    ", ".join(unexpected[:5])))
            raise ValueError("invalid 2.a screening set: " + "; ".join(details))
        for config in configs:
            weather_id = config["tunnel_test"]["weather_id"]
            if canonical_hash(config.get("weather")) != canonical_hash(
                    expected_profiles[weather_id]):
                raise ValueError(
                    "invalid 2.a screening set: weather parameters for {} "
                    "do not match the shared catalog".format(weather_id))
    return {
        "route_fingerprint": next(iter(route_fingerprints)),
        "condition_count": len(summaries),
        "weather_count": len(weather_ids),
        "screening_only": True,
    }


def _validate_expanded_screening_set(configs, require_all_weather):
    from stf_weather_catalog import WEATHER_CATALOG_VERSION

    groups = {}
    identities = set()
    expected_distances = None
    for config in configs:
        validate_tunnel_config(config)
        section = config["tunnel_test"]
        expansion = section["expansion"]
        if not isinstance(expansion, dict) or expansion.get("schema_version") != 1:
            raise ValueError("invalid tunnel expansion metadata")
        distances = validate_start_distances(expansion.get("distance_values_m"))
        distance = _number(expansion.get("start_distance_m"), "start_distance_m")
        if distance not in distances:
            raise ValueError("unexpected start_distance_m")
        if expansion.get("weather_catalog_version") != WEATHER_CATALOG_VERSION:
            raise ValueError("weather catalog version changed; regenerate expansion")
        if expansion.get("tunnel_geometry_fingerprint") != tunnel_geometry_fingerprint(config):
            raise ValueError("expanded route changed the fixed tunnel geometry")
        identity = dict(expansion)
        identity.pop("start_distance_m")
        identities.add(canonical_hash(identity))
        expected_distances = distances
        groups.setdefault(distance, []).append(config)
    if len(identities) != 1:
        raise ValueError("expanded definitions must share one baseline and expansion settings")
    if sorted(groups) != expected_distances:
        raise ValueError("expanded screening set is missing a start-distance group")
    summaries = []
    route_ids = set()
    for distance, group in sorted(groups.items()):
        # Reuse the strict one-route/120-weather checks. Removing expansion
        # changes the condition identity, so recompute it on private copies.
        plain = [copy.deepcopy(item) for item in group]
        for item in plain:
            item["tunnel_test"].pop("expansion")
            item["tunnel_test"]["condition_fingerprint"] = tunnel_condition_fingerprint(item)
        summary = validate_tunnel_screening_set(plain, require_all_weather)
        route_ids.add(summary["route_fingerprint"])
        summaries.append(dict(summary, start_distance_m=distance))
    if len(route_ids) != len(groups):
        raise ValueError("different start distances produced duplicate routes")
    return {
        "condition_count": len(configs),
        "weather_count": summaries[0]["weather_count"],
        "start_distance_count": len(groups),
        "start_distances_m": expected_distances,
        "baseline_route_fingerprint": configs[0]["tunnel_test"]["expansion"]["baseline_route_fingerprint"],
        "groups": summaries,
        "screening_only": True,
    }


def _validate_spawn_offset_set(configs, require_all_weather):
    from tunnel_spawn_offsets import validate_offsets

    groups = {}
    expected_sets = set()
    shared_settings = set()
    for config in configs:
        validate_tunnel_config(config)
        expansion = config["tunnel_test"]["expansion"]
        expected = expansion.get("expected_baseline_fingerprints")
        if not isinstance(expected, list) or not expected or len(set(expected)) != len(expected):
            raise ValueError("invalid expected baseline set")
        expected_sets.add(tuple(sorted(expected)))
        source = expansion["baseline_route_fingerprint"]
        if source not in expected:
            raise ValueError("unexpected baseline in spawn-offset expansion")
        requested, fallbacks = expansion.get("requested_offsets_m"), expansion.get("fallback_offsets_m")
        validate_offsets(requested, fallbacks)
        shared_settings.add(canonical_hash([requested, fallbacks, expansion["route_spacing_m"],
                                            expansion["direction_alignment"]["policy"]]))
        groups.setdefault(source, []).append(config)
    if len(expected_sets) != 1 or tuple(sorted(groups)) != next(iter(expected_sets)):
        raise ValueError("spawn-offset matrix is missing a baseline or mixes different batches")
    if len(shared_settings) != 1:
        raise ValueError("spawn-offset matrix mixes expansion settings")
    summaries = []
    legacy_keys = ("schema_version", "baseline_route_fingerprint", "source_entry_anchor_number",
                   "source_exit_anchor_number", "tunnel_geometry_fingerprint", "start_distance_m",
                   "distance_values_m", "route_spacing_m", "distance_metric", "weather_catalog_version")
    for source, group in sorted(groups.items()):
        identities, actuals, requests = set(), set(), {}
        plain = []
        for config in group:
            expansion = config["tunnel_test"]["expansion"]
            offset = _number(expansion["spawn_offset_m"], "spawn_offset_m")
            requested_offset = _number(expansion["requested_spawn_offset_m"], "requested_spawn_offset_m")
            if requested_offset not in expansion["requested_offsets_m"]:
                raise ValueError("unexpected requested spawn offset")
            if offset != requested_offset:
                if requested_offset <= 0 or offset not in expansion["fallback_offsets_m"] or not expansion["fallback_reason"]:
                    raise ValueError("invalid spawn-offset fallback")
            if requested_offset in requests and requests[requested_offset] != offset:
                raise ValueError("inconsistent fallback within a weather group")
            requests[requested_offset] = offset
            actuals.add(offset)
            expected_distance = expansion["baseline_entry_distance_m"] - offset
            if abs(expansion["start_distance_m"] - expected_distance) > 1e-6:
                raise ValueError("spawn offset does not match entrance distance")
            identity = dict(expansion)
            for key in ("spawn_offset_m", "requested_spawn_offset_m", "fallback_reason", "start_distance_m"):
                identity.pop(key)
            identities.add(canonical_hash(identity))
            item = copy.deepcopy(config)
            item["tunnel_test"]["expansion"] = {key: expansion[key] for key in legacy_keys}
            item["tunnel_test"]["condition_fingerprint"] = tunnel_condition_fingerprint(item)
            plain.append(item)
        reference = group[0]["tunnel_test"]["expansion"]
        if (len(identities) != 1 or sorted(actuals) != reference["actual_offsets_m"]
                or set(requests) != set(reference["requested_offsets_m"])
                or len(actuals) != len(requests)):
            raise ValueError("spawn-offset matrix is missing an offset or mixes baseline settings")
        summaries.append(_validate_expanded_screening_set(plain, require_all_weather))
    return dict(expansion_kind="spawn_offsets", baseline_count=len(groups),
                spawn_count=sum(item["start_distance_count"] for item in summaries),
                weather_count=summaries[0]["weather_count"], condition_count=len(configs),
                baselines=summaries, screening_only=True)


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "item"):
        try:
            return _json_safe(value.item())
        except (TypeError, ValueError):
            pass
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _markdown_cell(value):
    if value is None:
        return "—"
    if isinstance(value, (dict, list)):
        value = json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value).replace("|", "\\|").replace("\n", " ")


def load_existing_tunnel_records(output_dir, scenario_code="2a"):
    path = os.path.join(output_dir, "{}_result.json".format(scenario_code))
    if not os.path.exists(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as stream:
            payload = json.load(stream)
        records = payload.get("records", []) if isinstance(payload, dict) else []
        return records if isinstance(records, list) else []
    except (OSError, ValueError):
        return []


def write_tunnel_result_artifacts(
        records, output_dir, scenario_code="2a", metadata=None):
    os.makedirs(output_dir, exist_ok=True)
    prefix = os.path.join(output_dir, "{}_result".format(scenario_code))
    safe_records = [_json_safe(record) for record in records]
    metadata = _json_safe(metadata or {})
    generated_at = datetime.now(timezone.utc).isoformat()
    payload = {
        "schema_version": SCHEMA_VERSION,
        "scenario_code": scenario_code,
        "screening_only": True,
        "physical_field_test_compliance_claimed": False,
        "generated_at_utc": generated_at,
        "metadata": metadata,
        "records": safe_records,
    }
    with open(prefix + ".json", "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False, sort_keys=True)

    fields = sorted({key for record in safe_records for key in record})
    with open(prefix + ".csv", "w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in safe_records:
            writer.writerow({
                key: json.dumps(value, ensure_ascii=False, sort_keys=True)
                if isinstance(value, (dict, list)) else value
                for key, value in record.items()
            })

    with open(prefix + ".pkl", "wb") as stream:
        pickle.dump(safe_records, stream)

    pass_count = sum(record.get("pass") is True for record in safe_records)
    fail_count = sum(record.get("pass") is False for record in safe_records)
    invalid_count = sum(record.get("trial_valid") is False for record in safe_records)
    columns = [
        "run_id", "scenario", "controller", "weather_id", "condition_fingerprint",
        "start_distance_m", "entry_speed_mps", "approach_time_s",
        "spawn_offset_m", "requested_spawn_offset_m",
        "entry_crossed", "exit_crossed", "same_lane", "route_complete",
        "measured_tunnel_length_m", "max_tunnel_lateral_error_m", "collision",
        "timed_out", "trial_valid", "pass", "failure_reasons",
    ]
    with open(prefix + ".md", "w", encoding="utf-8") as stream:
        stream.write("# GB/T 41798—2022 2.a 隧道场景筛选结果\n\n")
        stream.write("- 结果用途：单次条件筛选；不构成国标3次正式通过结论\n")
        stream.write("- 物理场地试验符合性声明：false\n")
        stream.write("- 已记录：{}；通过：{}；失败：{}；无效：{}\n".format(
            len(safe_records), pass_count, fail_count, invalid_count))
        for key in sorted(metadata):
            stream.write("- {}：{}\n".format(key, _markdown_cell(metadata[key])))
        stream.write("\n## 逐条件结果\n\n")
        stream.write("| " + " | ".join(columns) + " |\n")
        stream.write("| " + " | ".join(["---"] * len(columns)) + " |\n")
        for record in safe_records:
            stream.write("| " + " | ".join(
                _markdown_cell(record.get(column)) for column in columns) + " |\n")

    return {
        "markdown": prefix + ".md",
        "json": prefix + ".json",
        "csv": prefix + ".csv",
        "pickle": prefix + ".pkl",
    }
