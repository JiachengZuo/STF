"""Expand one marked tunnel route along its CARLA lane topology.

The CARLA adapter reads waypoints only. Sampling, weather expansion and output
validation can also be exercised with synthetic geometry without a server.
"""

import bisect
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile

from tunnel_2a import (
    route_length_2d, tunnel_condition_fingerprint, tunnel_geometry_fingerprint,
    tunnel_route_fingerprint, validate_start_distances, validate_tunnel_config,
    validate_tunnel_screening_set,
)
from stf_weather_catalog import WEATHER_CATALOG_VERSION, all_weather_profiles


def load_baselines(source_dir, map_name, opendrive_sha256, route_id):
    paths = sorted(Path(source_dir).glob("scenario_2a_*.json"))
    if not paths:
        raise ValueError("No baseline JSON found; run editor and press S first")
    baselines = {}
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        if list(document) != [map_name]:
            raise ValueError("{}: baseline map does not match current CARLA map".format(path))
        routes = document[map_name].get(route_id)
        if not isinstance(routes, list) or len(routes) != 1:
            raise ValueError("{}: expected exactly one baseline scenario".format(path))
        candidate = routes[0]
        validate_tunnel_config(candidate)
        section = candidate["tunnel_test"]
        if "expansion" in section:
            raise ValueError("Expansion source must be an unexpanded baseline")
        if section["map_name"] != map_name or section["opendrive_sha256"] != opendrive_sha256:
            raise ValueError("{}: baseline map/OpenDRIVE mismatch".format(path))
        current = tunnel_route_fingerprint(candidate)
        baselines.setdefault(current, candidate)
    # Weather is deliberately regenerated from the current catalog. This also
    # permits expanding old baseline files without silently editing the source.
    return list(baselines.values()), paths


def load_baseline(source_dir, map_name, opendrive_sha256, route_id):
    baselines, paths = load_baselines(source_dir, map_name, opendrive_sha256, route_id)
    if len(baselines) != 1:
        raise ValueError("Source directory contains more than one baseline route")
    return baselines[0], paths


def read_approach_geometry(carla_map, baseline):
    """Return dense road-centre samples from baseline Ego to the entry."""
    from tunnel_geometry import TunnelRouteGeometry

    geometry = TunnelRouteGeometry()
    geometry.map = carla_map
    geometry.config = baseline
    geometry.tunnel_cfg = baseline["tunnel_test"]
    geometry._prepare_tunnel_geometry()
    start = geometry._project_waypoint(baseline["ego_start"], "baseline Ego")
    first = geometry._project_waypoint(baseline["ego_route"][0], "P1")
    prefix = geometry._trace_segment(start, first, "Ego to P1")
    points = []
    for waypoint in prefix[:-1]:
        tf = waypoint.transform
        points.append({"x": float(tf.location.x), "y": float(tf.location.y),
                       "z": float(tf.location.z), "yaw": float(tf.rotation.yaw)})
    points.extend({key: point[key] for key in ("x", "y", "z", "yaw")}
                  for point in geometry.tunnel_dense_route[:geometry.tunnel_entry_dense_index + 1])
    clean = []
    for point in points:
        if not clean or route_length_2d([clean[-1], point]) > 1e-6:
            clean.append(point)
    return clean


def _sample(points, lengths, position):
    index = min(len(points) - 2, max(0, bisect.bisect_right(lengths, position) - 1))
    span = lengths[index + 1] - lengths[index]
    ratio = (position - lengths[index]) / span
    a, b = points[index:index + 2]
    point = {key: float(a[key]) + ratio * (float(b[key]) - float(a[key]))
             for key in ("x", "y", "z")}
    angle = (float(b["yaw"]) - float(a["yaw"]) + 180.0) % 360.0 - 180.0
    point["yaw"] = (float(a["yaw"]) + ratio * angle + 180.0) % 360.0 - 180.0
    return point


def build_route_variants(baseline, approach, distances, route_spacing_m=5.0):
    validate_tunnel_config(baseline)
    distances = validate_start_distances(distances)
    if not math.isfinite(route_spacing_m) or not 1.0 <= route_spacing_m <= 10.0:
        raise ValueError("route_spacing_m must be between 1 and 10 m")
    if len(approach) < 2:
        raise ValueError("Baseline has no usable approach")
    lengths = [0.0]
    for a, b in zip(approach, approach[1:]):
        step = route_length_2d([a, b])
        if not math.isfinite(step) or step <= 1e-6:
            raise ValueError("Approach contains invalid or duplicate samples")
        lengths.append(lengths[-1] + step)
    available = lengths[-1]
    if max(distances) > available + 1e-6:
        raise ValueError("Baseline approach is {:.2f} m; requested {:.2f} m. "
                         "Move baseline Ego farther upstream and redraw the approach, "
                         "or reduce start_distances_m.".format(available, max(distances)))
    section = baseline["tunnel_test"]
    entry, exit_ = section["entry_anchor_number"], section["exit_anchor_number"]
    if route_length_2d([approach[-1], baseline["ego_route"][entry - 1]]) > 3.0:
        raise ValueError("Approach does not end at the marked tunnel entrance")
    tail = baseline["ego_route"][entry - 1:]
    configs = []
    for index, distance in enumerate(distances):
        start_s = max(0.0, available - distance)
        start = _sample(approach, lengths, start_s)
        start["z"] += 0.3  # Same road-to-spawn clearance as the editor.
        # Every retained waypoint is ahead of Ego; at least one remains before
        # the entrance even for the shortest supported start distance.
        positions = []
        position = start_s + 2.0
        while position <= available - 2.0 + 1e-6:
            positions.append(position)
            position += route_spacing_m
        route = [_sample(approach, lengths, position) for position in positions]
        new_entry = len(route) + 1
        route.extend(copy.deepcopy(tail))
        config = copy.deepcopy(baseline)
        config["name"] = "scenario_2a_start_{:02d}".format(index + 1)
        config["ego_start"] = copy.deepcopy(start)
        config["ego_route"] = copy.deepcopy(route)
        config.pop("trigger_position", None)
        new_section = config["tunnel_test"]
        new_section.update({
            "entry_anchor_number": new_entry,
            "exit_anchor_number": new_entry + exit_ - entry,
            "expansion": {
                "schema_version": 1,
                "baseline_route_fingerprint": tunnel_route_fingerprint(baseline),
                "source_entry_anchor_number": entry,
                "source_exit_anchor_number": exit_,
                "tunnel_geometry_fingerprint": tunnel_geometry_fingerprint(baseline),
                "start_distance_m": distance,
                "distance_values_m": distances,
                "route_spacing_m": route_spacing_m,
                "distance_metric": "CARLA_lane_centerline_xy_polyline",
                "weather_catalog_version": WEATHER_CATALOG_VERSION,
            },
        })
        new_section["route_fingerprint"] = tunnel_route_fingerprint(config)
        new_section["condition_fingerprint"] = tunnel_condition_fingerprint(config)
        configs.append(config)
    summary = validate_tunnel_screening_set(configs, require_all_weather=False)
    summary["available_approach_m"] = available
    return configs, summary


def apply_weather_to_variants(variants):
    """Apply the current full catalog only after route geometry is prepared."""
    configs = []
    for variant in variants:
        for profile in all_weather_profiles():
            config = copy.deepcopy(variant)
            config["name"] = variant["name"] + "_" + profile["id"]
            config["weather"] = copy.deepcopy(profile["parameters"])
            section = config["tunnel_test"]
            section["weather_id"] = profile["id"]
            section["expansion"]["weather_catalog_version"] = WEATHER_CATALOG_VERSION
            section["condition_fingerprint"] = tunnel_condition_fingerprint(config)
            configs.append(config)
    return configs, validate_tunnel_screening_set(configs)


def expand_baseline(baseline, approach, distances, route_spacing_m=5.0):
    variants, geometry_summary = build_route_variants(baseline, approach, distances, route_spacing_m)
    configs, summary = apply_weather_to_variants(variants)
    summary["available_approach_m"] = geometry_summary["available_approach_m"]
    return configs, summary


def write_expansion(configs, output_dir, route_id, manifest):
    """Publish a complete directory, refusing to mix with existing definitions."""
    validate_tunnel_screening_set(configs)
    output = Path(output_dir)
    if output.exists():
        raise ValueError("Expansion output already exists: {}. Choose a new directory.".format(output))
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".tunnel-expansion-", dir=output.parent))
    try:
        files = []
        for config in configs:
            name = config["name"] + ".json"
            document = {config["tunnel_test"]["map_name"]: {route_id: [config]}}
            payload = (json.dumps(document, ensure_ascii=False, sort_keys=True,
                                  separators=(",", ":")) + "\n").encode("utf-8")
            (staging / name).write_bytes(payload)
            files.append({"file": name, "sha256": hashlib.sha256(payload).hexdigest()})
        payload = dict(manifest, files=files, condition_count=len(configs))
        (staging / "expansion_manifest.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        staging.rename(output)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return output
