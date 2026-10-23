"""Configurable spawn offsets per baseline, with optional negative-offset fallbacks."""

import copy
import math

from tunnel_2a import (
    route_length_2d, tunnel_condition_fingerprint,
    tunnel_route_fingerprint,
    validate_tunnel_screening_set,
)
from tunnel_expansion import build_route_variants, apply_weather_to_variants, read_approach_geometry


def validate_offsets(requested, fallbacks):
    for name, values in (("spawn_offsets_m", requested), ("fallback_offsets_m", fallbacks)):
        if not isinstance(values, list) or (name == "spawn_offsets_m" and not values):
            raise ValueError(name + " must be a list (spawn offsets cannot be empty)")
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            raise ValueError(name + " must contain finite numbers")
        if len(set(values)) != len(values):
            raise ValueError(name + " must not contain duplicates")
    if 0 not in requested:
        raise ValueError("spawn_offsets_m must include the baseline offset 0")
    if any(value >= 0 for value in fallbacks) or set(requested) & set(fallbacks):
        raise ValueError("fallback offsets must be negative and separate from requested offsets")


def _point(waypoint, clearance=0.0):
    tf = waypoint.transform
    return dict(x=float(tf.location.x), y=float(tf.location.y),
                z=float(tf.location.z) + clearance, yaw=float(tf.rotation.yaw))


def prepare_forward_baseline(carla_map, source, direction_policy="require_forward"):
    """Require the operator's route to follow lane direction; never auto-reroute."""
    from tunnel_2a import validate_tunnel_config
    if direction_policy != "require_forward":
        raise ValueError("Only require_forward is supported; redraw incorrect routes")
    validate_tunnel_config(source)
    baseline = copy.deepcopy(source)
    approach = read_approach_geometry(carla_map, baseline)
    report = {"policy": "require_forward", "direction_corrected": False,
              "source_ego_start": source["ego_start"],
              "reference_ego_start": baseline["ego_start"]}
    return baseline, approach, report


def extend_approach_upstream(carla_map, baseline, approach, required_m):
    from tunnel_geometry import TunnelRouteGeometry

    geometry = TunnelRouteGeometry()
    geometry.map = carla_map
    current = geometry._project_waypoint(baseline["ego_start"], "reference Ego")
    extension = []
    travelled = 0.0
    seen = {geometry._waypoint_token(current)}
    reason = None
    while travelled < required_m + 1.0:
        candidates = list(current.previous(1.0))
        if len(candidates) != 1:
            reason = "upstream road ends or branches"
            break
        previous = candidates[0]
        token = geometry._waypoint_token(previous)
        step = route_length_2d([_point(previous), _point(current)])
        if token in seen or step < 1e-6:
            reason = "upstream topology repeats"
            break
        # Custom maps also mark unbranched connecting lanes as junctions.
        # Require a unique predecessor instead of rejecting that label alone;
        # the completed variant is traced forwards again before publication.
        seen.add(token)
        extension.append(_point(previous))
        travelled += step
        current = previous
    return list(reversed(extension)) + approach, reason


def select_spawn_offsets(requested, fallbacks, baseline_distance, available_distance):
    """Keep all feasible requested offsets; replace infeasible positive ones."""
    validate_offsets(requested, fallbacks)

    def rejection(offset):
        distance = baseline_distance - offset
        if distance < 5.0:
            return "less than 5 m remains before the tunnel entrance"
        if distance > available_distance + 1e-6:
            return "insufficient unambiguous upstream lane"
        return None

    choices = []
    used = set()
    for requested_offset in requested:
        offset = requested_offset
        reason = rejection(offset)
        if reason:
            if offset <= 0:
                raise ValueError("Required spawn offset {} m rejected: {}".format(offset, reason))
            offset = next((candidate for candidate in fallbacks
                           if candidate not in used and rejection(candidate) is None), None)
            if offset is None:
                raise ValueError("Cannot replace spawn offset {} m: no valid fallback".format(requested_offset))
        used.add(offset)
        choices.append(dict(requested_offset_m=float(requested_offset),
                            actual_offset_m=float(offset), rejection_reason=reason))
    return choices


def build_spawn_route_variants(carla_map, sources, requested, fallbacks, direction_policy, spacing=5.0):
    validate_offsets(requested, fallbacks)
    baseline_ids = sorted(tunnel_route_fingerprint(source) for source in sources)
    if len(set(baseline_ids)) != len(sources):
        raise ValueError("duplicate baseline routes")
    output, reports = [], []
    for route_index, source in enumerate(sources, 1):
        baseline, approach, alignment = prepare_forward_baseline(carla_map, source, direction_policy)
        baseline_distance = route_length_2d(approach)
        extended, stop_reason = extend_approach_upstream(
            carla_map, baseline, approach, -min(requested + fallbacks))
        choices = select_spawn_offsets(requested, fallbacks, baseline_distance, route_length_2d(extended))
        distances = [baseline_distance - item["actual_offset_m"] for item in choices]
        configs, _ = build_route_variants(baseline, extended, distances, spacing)
        by_distance = {distance: item for distance, item in zip(distances, choices)}
        actual_offsets = sorted(item["actual_offset_m"] for item in choices)
        source_id = tunnel_route_fingerprint(source)
        for config in configs:
            section = config["tunnel_test"]
            expansion = section["expansion"]
            distance = expansion["start_distance_m"]
            selected = by_distance[distance]
            expansion.update({
                "kind": "spawn_offsets",
                "baseline_route_fingerprint": source_id,
                "expected_baseline_fingerprints": baseline_ids,
                "reference_route_fingerprint": tunnel_route_fingerprint(baseline),
                "baseline_entry_distance_m": baseline_distance,
                "spawn_offset_m": selected["actual_offset_m"],
                "requested_spawn_offset_m": selected["requested_offset_m"],
                "requested_offsets_m": requested,
                "fallback_offsets_m": fallbacks,
                "actual_offsets_m": actual_offsets,
                "fallback_reason": selected["rejection_reason"],
                "direction_alignment": alignment,
            })
            tag = ("m" if selected["actual_offset_m"] < 0 else "p") + "{:g}".format(abs(selected["actual_offset_m"]))
            config["name"] = "scenario_2a_route_{:02d}_offset_{}".format(route_index, tag)
            if selected["actual_offset_m"] == 0:
                config["ego_start"] = copy.deepcopy(source["ego_start"])
                section["route_fingerprint"] = tunnel_route_fingerprint(config)
            section["condition_fingerprint"] = tunnel_condition_fingerprint(config)
            read_approach_geometry(carla_map, config)
            output.append(config)
        reports.append(dict(baseline_route_fingerprint=source_id, alignment=alignment,
                            baseline_entry_distance_m=baseline_distance,
                            choices=choices, upstream_stop_reason=stop_reason))
    summary = validate_tunnel_screening_set(output, require_all_weather=False)
    return output, dict(summary, baseline_reports=reports)


def expand_spawn_baselines(carla_map, sources, requested, fallbacks, direction_policy, spacing=5.0):
    variants, route_summary = build_spawn_route_variants(
        carla_map, sources, requested, fallbacks, direction_policy, spacing)
    configs, summary = apply_weather_to_variants(variants)
    return configs, dict(summary, baseline_reports=route_summary["baseline_reports"])
