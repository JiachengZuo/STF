"""5e geometry, immutable weather expansion and simulation-time scoring rules."""
from __future__ import annotations

import copy
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path

from emergency_brake_5d import digest, positive, project, route_point_at
from stf_weather_catalog import all_weather_profiles

RISK_IDS = ('box', 'barrel', 'plank', 'pedestrian', 'bicycle', 'motorcycle', 'car', 'truck')
TCP_STATIONARY_TIMEOUT_S = 30.0


def map_identifier(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('5e town must be a nonempty map name or /Game/... path')
    value = value.strip().replace('\\', '/').rstrip('/')
    return value[:-5] if value.lower().endswith('.umap') else value


def map_matches(actual, requested):
    actual, requested = map_identifier(actual), map_identifier(requested)
    return actual == requested if '/' in requested else actual.rsplit('/', 1)[-1] == requested


def require_map(actual, requested):
    if not map_matches(actual, requested):
        raise ValueError('5e map mismatch: expected {}, found {}'.format(requested, actual))


def ensure_world(client, requested):
    """Use the requested registered map; leave an already matching world intact."""
    requested = map_identifier(requested)
    world = client.get_world()
    if map_matches(world.get_map().name, requested):
        return world
    matches = [p for p in client.get_available_maps() if map_matches(p, requested)]
    if not matches:
        raise ValueError('5e map is not registered on this CARLA Server: ' + requested)
    if len(matches) != 1:
        raise ValueError('ambiguous map name; use a full /Game/... path with --town: ' + requested)
    if list(world.get_actors().filter('vehicle.*')) or list(world.get_actors().filter('walker.pedestrian.*')):
        raise RuntimeError('current world contains traffic actors; finish that session before switching maps')
    print('5e loading map: ' + matches[0], flush=True)
    client.set_timeout(120)
    try:
        world = client.load_world(map_identifier(matches[0]))
    finally:
        client.set_timeout(20)
    require_map(world.get_map().name, requested)
    return world


def load_settings(path):
    import yaml
    cfg = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if cfg.get('schema_version') != 1:
        raise ValueError('5e config schema_version must be 1')
    cfg['town'] = map_identifier(cfg['town'])
    positive(cfg['ego_speed_kmh'], 'ego_speed_kmh')
    for key in ('fixed_delta_seconds', 'timeout_s', 'camera_timeout_s', 'stop_speed_mps',
                'stop_hold_s', 'min_stop_gap_m', 'max_stop_gap_m', 'end_tolerance_m',
                'boundary_tolerance_m', 'curve_window_m', 'min_curve_angle_deg',
                'crossing_tolerance_m', 'target_clearance_m', 'target_timeout_s',
                'target_speed_tolerance_mps', 'target_speed_grace_s'):
        positive(cfg['engineering'][key], key)
    eng = cfg['engineering']
    if eng['fixed_delta_seconds'] > .1 or eng['min_stop_gap_m'] >= eng['max_stop_gap_m']:
        raise ValueError('invalid timestep or stopping interval')
    risks = cfg['risks']
    if [r['id'] for r in risks] != list(RISK_IDS):
        raise ValueError('5e requires eight ordered risk templates: ' + ', '.join(RISK_IDS))
    for i, r in enumerate(risks):
        expected = 'static' if i < 3 else 'walker' if i == 3 else 'vehicle'
        prefix = {'static': 'static.prop.', 'walker': 'walker.pedestrian.', 'vehicle': 'vehicle.'}[expected]
        if r['kind'] != expected or not r['blueprint'].startswith(prefix):
            raise ValueError('invalid risk blueprint/kind: ' + r['id'])
        if expected != 'static':
            positive(r['speed_mps'], r['id'] + ' speed')
        elif not math.isfinite(float(r['yaw_offset_deg'])):
            raise ValueError('invalid static orientation')
    if not cfg['ego_blueprint'].startswith('vehicle.'):
        raise ValueError('ego_blueprint must be a vehicle')
    return cfg


def xy_distance(a, b):
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def angle_delta(a, b):
    return (a - b + 180) % 360 - 180


def checked_point(p):
    for key in ('x', 'y', 'z'):
        if isinstance(p.get(key), bool) or not math.isfinite(float(p[key])):
            raise ValueError('nonfinite coordinate: ' + key)
    return {k: float(p[k]) for k in ('x', 'y', 'z')}


def at(route, s):
    length = project(route[-1], route)['length']
    if s >= length - 1e-7:
        return dict(route[-1])
    return route_point_at(route, max(0., s))


def trace_route(map_obj, anchors):
    """Trace forward lane topology between editor anchors, without lane changes."""
    import carla

    def waypoint(p):
        checked_point(p)
        w = map_obj.get_waypoint(carla.Location(**checked_point(p)),
                                 project_to_road=True, lane_type=carla.LaneType.Driving)
        if w is None or math.hypot(w.transform.location.x-p['x'], w.transform.location.y-p['y']) > .75:
            raise ValueError('Ego/route anchors must be on a driving lane center')
        return w

    if len(anchors) < 3:
        raise ValueError('place Ego and at least two forward route anchors')
    dense = [waypoint(anchors[0])]
    if abs(angle_delta(float(anchors[0]['yaw']), dense[0].transform.rotation.yaw)) > 20:
        raise ValueError('Ego heading must follow the route')
    for anchor in anchors[1:]:
        start, goal = dense[-1], waypoint(anchor)
        if start.transform.location.distance(goal.transform.location) < 1.1:
            raise ValueError('route anchors must be more than 1.1 m apart')
        serial = itertools.count()
        queue = [(0., next(serial), start, [], 0.)]
        visited = set()
        for _ in range(20000):
            if not queue:
                raise ValueError('next anchor is not reachable along forward lane topology')
            _, _, current, path, traveled = heapq.heappop(queue)
            key = (current.road_id, current.section_id, current.lane_id, round(current.s, 1))
            if key in visited:
                continue
            visited.add(key)
            distance = current.transform.location.distance(goal.transform.location)
            if (current.road_id, current.lane_id) == (goal.road_id, goal.lane_id) and distance <= 1.05:
                # The target must lie ahead (or at the same sample), never double back.
                delta = goal.transform.location-current.transform.location
                forward = current.transform.get_forward_vector()
                if delta.x*forward.x + delta.y*forward.y >= -.05:
                    dense.extend(path)
                    if distance > .05:
                        dense.append(goal)
                    break
            if traveled > 500:
                continue
            for nxt in current.next(1.):
                cost = traveled + current.transform.location.distance(nxt.transform.location)
                heapq.heappush(queue, (cost+nxt.transform.location.distance(goal.transform.location),
                                      next(serial), nxt, path+[nxt], cost))
        else:
            raise ValueError('route trace exceeded bound; add closer anchors')
    result = []
    for w in dense:
        l = w.transform.location
        p = dict(x=l.x, y=l.y, z=l.z, yaw=w.transform.rotation.yaw, width=w.lane_width,
                 road_id=w.road_id, lane_id=w.lane_id)
        if not result or xy_distance(p, result[-1]) > .05:
            result.append(p)
    return result


def geometry_metrics(baseline, eng):
    route = baseline['ego_route']
    if len(route) < 3:
        raise ValueError('dense Ego route is missing')
    for p in route:
        checked_point(p)
        positive(p['width'], 'lane width')
        if not math.isfinite(float(p['yaw'])):
            raise ValueError('invalid route heading')
    if any(not .01 < xy_distance(a, b) <= 2.1 for a, b in zip(route, route[1:])):
        raise ValueError('route must contain continuous dense samples')
    if xy_distance(baseline['ego_start'], route[0]) > .1:
        raise ValueError('Ego start differs from route start')
    p, t = baseline['risk_position'], baseline['trigger_position']
    checked_point(p); checked_point(t)
    pp, tp = project(p, route), project(t, route)
    if pp['lateral'] > .75 or tp['lateral'] > .75:
        raise ValueError('P and T must lie on the drawn Ego route')
    if pp['length'] - pp['s'] < 8:
        raise ValueError('leave at least 8 m of route after P')
    before, after = at(route, pp['s']-eng['curve_window_m']), at(route, pp['s']+eng['curve_window_m'])
    angle = abs(angle_delta(after['yaw'], before['yaw']))
    if angle < eng['min_curve_angle_deg']:
        raise ValueError('P must be within the bend, not a straight route segment')
    start, end = (checked_point(baseline[k]) for k in ('crossing_start', 'crossing_end'))
    dx, dy = end['x']-start['x'], end['y']-start['y']
    length = math.hypot(dx, dy)
    if length < 4:
        raise ValueError('crossing start/end must straddle the road')
    crossing = project(p, [start, end])
    if crossing['lateral'] > .5 or not 1 < crossing['s'] < length-1:
        raise ValueError('straight crossing path must pass through P between its endpoints')
    yaw = math.degrees(math.atan2(dy, dx))
    sine = abs(math.sin(math.radians(angle_delta(yaw, at(route, pp['s'])['yaw']))))
    if sine < .707:
        raise ValueError('crossing path must cross the Ego route at 45 to 135 degrees')
    return dict(route_length_m=pp['length'], risk_s=pp['s'], trigger_s=tp['s'],
                crossing_length_m=length, crossing_conflict_s=crossing['s'],
                crossing_yaw=yaw, crossing_sine=sine, curve_angle_deg=angle)


def crossing_delay(metrics, risk, ego_speed_kmh):
    """Keep positive alignment delays; release at T when alignment would need an earlier start."""
    if risk['kind'] == 'static':
        return 0.
    delay = ((metrics['risk_s']-metrics['trigger_s'])/(ego_speed_kmh/3.6)
             - metrics['crossing_conflict_s']/risk['speed_mps'])
    return max(0., delay)


def make_baseline(map_obj, draft, cfg):
    require_map(map_obj.name, cfg['town'])
    b = copy.deepcopy(draft)
    anchors = [b['ego_start']] + b['route_anchors']
    b['ego_route'] = trace_route(map_obj, anchors)
    b['ego_start'] = {k: b['ego_route'][0][k] for k in ('x', 'y', 'z', 'yaw')}
    for key in ('risk_position', 'trigger_position'):
        pr = project(b[key], b['ego_route'])
        if pr['lateral'] > .75:
            raise ValueError(key + ' must be on the drawn route')
        b[key] = at(b['ego_route'], pr['s'])
    b.update(schema_version=1, scenario_code='5e', map_name=map_obj.name,
             opendrive_sha256=hashlib.sha256(map_obj.to_opendrive().encode()).hexdigest())
    b['geometry'] = geometry_metrics(b, cfg['engineering'])
    b.pop('baseline_fingerprint', None)
    b['baseline_fingerprint'] = digest(b)
    return b


def verify_fingerprint(document, key):
    raw = dict(document)
    if raw.pop(key, None) != digest(raw):
        raise ValueError(key + ' mismatch')


def make_definitions(baseline, cfg):
    verify_fingerprint(baseline, 'baseline_fingerprint')
    if baseline.get('scenario_code') != '5e' or baseline.get('schema_version') != 1:
        raise ValueError('not a 5e baseline')
    require_map(baseline['map_name'], cfg['town'])
    metrics = geometry_metrics(baseline, cfg['engineering'])
    if metrics != baseline['geometry']:
        raise ValueError('baseline geometry mismatch')
    profiles = all_weather_profiles()
    if len(profiles) != 120:
        raise ValueError('5e requires the existing 120 weather profiles')
    definitions = []
    for risk in cfg['risks']:
        if (risk['kind'] != 'static' and metrics['crossing_length_m']/risk['speed_mps']
                + cfg['engineering']['target_speed_grace_s'] >= cfg['engineering']['target_timeout_s']):
            raise ValueError('crossing path exceeds target_timeout_s for ' + risk['id'])
        template = dict(risk, start_delay_s=crossing_delay(metrics, risk, cfg['ego_speed_kmh']))
        for weather in profiles:
            d = dict(schema_version=1, scenario_code='5e',
                     scenario='scenario_5e_' + risk['id'] + '_' + weather['id'],
                     baseline=copy.deepcopy(baseline), risk=copy.deepcopy(template),
                     ego_blueprint=cfg['ego_blueprint'], ego_speed_kmh=cfg['ego_speed_kmh'],
                     engineering=copy.deepcopy(cfg['engineering']), weather_id=weather['id'],
                     weather=copy.deepcopy(weather['parameters']), screening_only=True,
                     collision_enhancement_enabled=False)
            d['condition_fingerprint'] = digest(d)
            definitions.append(d)
    return definitions


def validate_definitions(paths, cfg):
    if not paths:
        raise ValueError('no 5e definitions; draw a baseline and expand it first')
    docs = [json.loads(Path(p).read_text(encoding='utf-8')) for p in paths]
    expected = {d['scenario']: d for d in make_definitions(docs[0]['baseline'], cfg)}
    if len(docs) != len(expected) or {d['scenario'] for d in docs} != set(expected):
        raise ValueError('run requires a complete 8 x 120 matrix before selecting a subset')
    for path, d in zip(paths, docs):
        verify_fingerprint(d, 'condition_fingerprint')
        if Path(path).stem != d['scenario'] or d != expected[d['scenario']]:
            raise ValueError('definition differs from fixed baseline/config/weather: ' + str(path))
    return sorted(docs, key=lambda d: d['scenario'])


class CurveTrial:
    """Controller-independent decisions. All durations use CARLA simulation time."""
    def __init__(self, definition, *, stationary_timeout_s=None):
        self.d = definition
        self.eng = definition['engineering']
        self.static = definition['risk']['kind'] == 'static'
        self.geometry = definition['baseline']['geometry']
        self.started = self.triggered = self.stopped_since = None
        self.stationary_timeout_s = stationary_timeout_s
        self.stationary_since = None
        self.stationary_duration_s = 0.0
        self.status = 'RUNNING'
        self.reasons = []

    def end(self, status, reason):
        if status != self.status:
            self.reasons = []
        self.status = status
        if reason not in self.reasons:
            self.reasons.append(reason)

    def update(self, now, *, ego_s, ego_speed, gap, at_end=False, collision=False,
               outside=False, target_clear=False, target_finished=False, target_error=None):
        if self.status != 'RUNNING':
            return
        if self.started is None:
            self.started = now
        if self.stationary_timeout_s is not None:
            if ego_speed <= self.eng['stop_speed_mps']:
                if self.stationary_since is None:
                    self.stationary_since = now
                self.stationary_duration_s = now - self.stationary_since
            else:
                self.stationary_since = None
                self.stationary_duration_s = 0.0
        if collision:
            self.end('FAILED', 'collision')
            return
        if outside:
            self.end('FAILED', 'outside_allowed_lane')
            return
        if target_error:
            self.end('INVALID', target_error)
            return
        if self.triggered is None and ego_s >= self.geometry['trigger_s']:
            self.triggered = now
        if at_end:
            self.end('PASS', 'ego_reached_route_end')
            return
        if not self.static and target_finished:
            self.end('PASS', 'target_reached_crossing_end')
            return
        if self.static:
            safely_stopped = (ego_s > 2 and ego_speed <= self.eng['stop_speed_mps']
                              and self.eng['min_stop_gap_m'] <= gap <= self.eng['max_stop_gap_m'])
            self.stopped_since = (now if self.stopped_since is None else self.stopped_since) if safely_stopped else None
            if self.stopped_since is not None and now-self.stopped_since >= self.eng['stop_hold_s']:
                self.end('PASS', 'safe_stop_before_obstacle')
        elif self.triggered is not None:
            if not target_clear and now-self.triggered > self.d['risk']['start_delay_s']+self.eng['target_timeout_s']:
                self.end('INVALID', 'target_did_not_clear')
        if (self.status == 'RUNNING' and self.stationary_timeout_s is not None
                and self.stationary_duration_s > self.stationary_timeout_s):
            self.end('FAILED', 'ego_stationary_timeout')
        if self.status == 'RUNNING' and now-self.started >= self.eng['timeout_s']:
            self.end('FAILED', 'ego_task_timeout')

    def result(self):
        valid = self.status in ('PASS', 'FAILED')
        return dict(result_status=self.status, trial_valid=valid,
                    **{'pass': self.status == 'PASS' if valid else None},
                    failure_reasons=self.reasons if self.status == 'FAILED' else [],
                    invalid_reasons=self.reasons if self.status == 'INVALID' else [],
                    completion_reason=self.reasons[0] if self.status == 'PASS' else None,
                    completion_policy='ego_end_or_target_end_or_collision',
                    trigger_sim_time=self.triggered,
                    ego_stationary_timeout_s=self.stationary_timeout_s,
                    ego_stationary_speed_threshold_mps=self.eng['stop_speed_mps'],
                    ego_stationary_since_sim_time=self.stationary_since,
                    ego_stationary_duration_s=self.stationary_duration_s)
