"""2d geometry, live TTC and scoring. Importable without CARLA or pygame."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from statistics import median

from emergency_brake_5d import digest, positive, project, route_point_at
import stf_weather_catalog as weather_catalog

ENTRY_SPEEDS = [5, 10, 15, 20, 25, 30, 35, 40]
DEFINITION_PREFIX = 'scenario_2d_v'
LEGACY_DEFINITION_PREFIX = 'cross_right_2d_v'
# Old configuration snapshots may still contain these removed drawing gates.
RETIRED_DRAWING_KEYS = {
    'max_straight_heading_change_deg', 'min_crossing_angle_deg', 'max_crossing_angle_deg',
    'right_side_min_lateral_m', 'approach_settle_m', 'target_spawn_margin_m',
    'downstream_margin_m', 'conflict_end_margin_m', 'max_timeout_s',
}
FAIL_PRIORITY = (
    'collision_with_target', 'required_speed_not_reached', 'yield_not_established', 'corresponding_lane_not_entered',
    'section_speed', 'infrastructure_collision', 'solid_line', 'wrong_way_lane',
)
FORBIDDEN = {'trigger_position', 'trigger_radius', 'start_delay_s', 'risk', 'other_actors',
             'target_clearance_m'}


def validate_settings(cfg):
    fixed = dict(schema_version=1, target_speed_kmh=20.0, ttc_window_s=[4.5, 5.5],
                 section_speed_limit_kmh=40.0, lane_width_m=[3.0, 3.5], entry_speeds_kmh=ENTRY_SPEEDS)
    for key, value in fixed.items():
        if cfg.get(key) != value:
            raise ValueError('2d requires {} = {}'.format(key, value))
    for key, value in cfg['engineering'].items():
        if key not in RETIRED_DRAWING_KEYS:
            positive(value, key)
    if cfg['engineering']['fixed_delta_seconds'] != .05:
        raise ValueError('2d fixed_delta_seconds must be 0.05')
    for key in ('ego_blueprint', 'target_blueprint'):
        if not cfg[key].startswith('vehicle.'):
            raise ValueError(key + ' must be a vehicle blueprint')
    return cfg


def load_settings(path):
    import yaml
    return validate_settings(yaml.safe_load(Path(path).read_text(encoding='utf-8')))


def angle_delta(a, b):
    return (a - b + 180) % 360 - 180


def stations(route):
    result = [0.0]
    for a, b in zip(route, route[1:]):
        result.append(result[-1] + math.hypot(b['x']-a['x'], b['y']-a['y']))
    return result


def at(route, s, extend_ends=False):
    length = stations(route)[-1]
    if extend_ends and (s < 0 or s > length):
        a, b = (route[0], route[1]) if s < 0 else (route[-2], route[-1])
        segment = math.hypot(b['x']-a['x'], b['y']-a['y'])
        origin = a if s < 0 else b
        offset = s if s < 0 else s-length
        return dict(x=origin['x']+offset*(b['x']-a['x'])/segment,
                    y=origin['y']+offset*(b['y']-a['y'])/segment,
                    z=origin['z']+offset*(b['z']-a['z'])/segment,
                    yaw=math.degrees(math.atan2(b['y']-a['y'], b['x']-a['x'])))
    return route_point_at(route, max(0., min(s, length-1e-8)))


def normalized_route(route):
    result = []
    for source in route:
        if not all(k in source and math.isfinite(float(source[k])) for k in ('x', 'y', 'z')):
            raise ValueError('路线坐标无效')
        p = dict(source)
        p.update({k: float(source[k]) for k in ('x', 'y', 'z')})
        if not result or math.hypot(p['x']-result[-1]['x'], p['y']-result[-1]['y']) > 1e-8:
            result.append(p)
    if len(result) < 2:
        raise ValueError('每条路线至少需要两个不同的位置')
    for i, p in enumerate(result):
        a, b = (result[i], result[i+1]) if i+1 < len(result) else (result[i-1], result[i])
        if p.get('yaw') is None or not math.isfinite(float(p['yaw'])):
            p['yaw'] = math.degrees(math.atan2(b['y']-a['y'], b['x']-a['x']))
    return result


def route_intersection(ego_route, target_route):
    """Unique XY intersection, coalescing a shared sampling vertex."""
    es, ts = stations(ego_route), stations(target_route)
    hits = []
    for i, (a, b) in enumerate(zip(ego_route, ego_route[1:])):
        dx, dy = b['x']-a['x'], b['y']-a['y']
        for j, (c, d) in enumerate(zip(target_route, target_route[1:])):
            ex, ey = d['x']-c['x'], d['y']-c['y']
            det = dx*ey-dy*ex
            if abs(det) < 1e-9:
                continue
            qx, qy = c['x']-a['x'], c['y']-a['y']
            u, v = (qx*ey-qy*ex)/det, (qx*dy-qy*dx)/det
            if -1e-8 <= u <= 1+1e-8 and -1e-8 <= v <= 1+1e-8:
                hit = (es[i]+u*(es[i+1]-es[i]), ts[j]+v*(ts[j+1]-ts[j]),
                       dict(x=a['x']+u*dx, y=a['y']+u*dy, z=a.get('z', 0.)))
                if not any(abs(hit[0]-h[0]) < 1e-6 and abs(hit[1]-h[1]) < 1e-6 for h in hits):
                    hits.append(hit)
    if not hits:
        raise ValueError('两条路线没有交点，请让两条路线穿过同一位置')
    if len(hits) > 1:
        raise ValueError('两条路线有多个交点，请调整路线以确定一个冲突点')
    return hits[0]


def directed_arms(headings, tolerance):
    """Circular nearest-neighbour clusters; opposite approaches stay distinct."""
    values = sorted(float(h) % 360 for h in headings)
    if not values or not all(math.isfinite(h) for h in values):
        raise ValueError('map_annotations_missing: junction entry headings')
    gaps = [(values[(i+1) % len(values)]-v) % 360 for i, v in enumerate(values)]
    start = (max(range(len(gaps)), key=gaps.__getitem__)+1) % len(values)
    ordered = values[start:]+values[:start]
    clusters = [[ordered[0]]]
    for value in ordered[1:]:
        if (value-clusters[-1][-1]) % 360 <= tolerance:
            clusters[-1].append(value)
        else:
            clusters.append([value])
    return [math.degrees(math.atan2(sum(math.sin(math.radians(h)) for h in group),
                                   sum(math.cos(math.radians(h)) for h in group))) % 360
            for group in clusters]


def build_geometry(ego_route, target_route, cfg):
    """Derive runnable geometry. Map annotations are observations, not save gates."""
    eng = cfg['engineering']
    ego_route, target_route = normalized_route(ego_route), normalized_route(target_route)
    routes = (ego_route, target_route)
    ec, tc, point = route_intersection(*routes)
    lengths = [stations(r)[-1] for r in routes]
    halves = []
    for route, center in zip(routes, (ec, tc)):
        samples = list(zip(route, stations(route)))
        valid = [(p, arc) for p, arc in samples if p.get('width') is not None
                 and math.isfinite(float(p['width'])) and p['width'] > 0]
        upstream = [(p, arc) for p, arc in valid if arc < center and p.get('is_junction') is False]
        nearby = [(p, arc) for p, arc in upstream if arc >= center-eng.get('straight_window_m', 20.)]
        selected = nearby or upstream or valid
        halves.append(median(p['width']/2 for p, _ in selected) if selected else eng['kinematic_half_width_m'])
    eyaw, tyaw = at(ego_route, ec)['yaw'], at(target_route, tc)['yaw']
    angle = abs(angle_delta(eyaw, tyaw))
    sine = abs(math.sin(math.radians(angle)))
    if sine < 1e-8:
        raise ValueError('两条路线在交点处重合或平行，无法确定横穿冲突区')
    upstream = at(target_route, max(0., tc-eng.get('right_side_sample_m', 8.)))
    yaw = math.radians(eyaw)
    dx, dy = upstream['x']-point['x'], upstream['y']-point['y']
    right = -dx*math.sin(yaw)+dy*math.cos(yaw)
    headings = ego_route[0].get('junction_entry_headings')
    try:
        arms = directed_arms(headings, eng.get('arm_heading_cluster_deg', 25.)) if headings else None
    except (TypeError, ValueError):
        arms = None
    eh, th = halves
    wait = tc-cfg['target_speed_kmh']/3.6*cfg['ttc_window_s'][1]
    if abs(wait) < 1e-7:
        wait = 0.
    far = tc+eh/sine
    origin = at(target_route, wait, extend_ends=True)
    ty = math.radians(origin['yaw'])
    samples = [origin, at(target_route, far, extend_ends=True)] + [p for p, arc in zip(target_route, stations(target_route)) if wait <= arc <= far]
    ray_error = max(abs(-(p['x']-origin['x'])*math.sin(ty)+(p['y']-origin['y'])*math.cos(ty)) for p in samples)
    timeout = max(eng['min_timeout_s'], lengths[0]/(5/3.6)+eng['timeout_margin_s'])
    limits_by_route = [[p['speed_limit_kmh'] for p in route if p.get('speed_limit_kmh') is not None] for route in routes]
    map_limit_verified = all(limits and all(v == cfg['section_speed_limit_kmh'] for v in limits) for limits in limits_by_route)
    signal_observations = [p.get('signal_hit') for r in routes for p in r]
    signal_clear = False if any(v is True for v in signal_observations) else (
        True if all(v is False for v in signal_observations) else None)
    return dict(conflict_point=point, ego_conflict_s=ec, target_conflict_s=tc,
                crossing_angle_deg=angle, ego_half_width_m=eh, target_half_width_m=th,
                ego_conflict_near_s=ec-th/sine, ego_conflict_far_s=ec+th/sine,
                target_conflict_near_s=tc-eh/sine, target_conflict_far_s=far,
                target_wait_s=wait, target_wait_pose=origin,
                exit_gate_s=min(ec+eng['exit_gate_m'], lengths[0]),
                directed_arm_count=None if arms is None else len(arms), right_side_lateral_m=right,
                target_ray_max_error_m=ray_error, timeout_s=timeout,
                section_speed_limit_verified=map_limit_verified, signal_query_clear=signal_clear,
                ego_length_m=lengths[0], target_length_m=lengths[1])


def verify_fingerprint(document, key):
    raw = dict(document)
    if raw.pop(key, None) != digest(raw):
        raise ValueError(key + ' mismatch')


def reject_legacy_fields(value):
    if isinstance(value, dict):
        if FORBIDDEN.intersection(value):
            raise ValueError('2d does not accept legacy trigger/delay/risk fields')
        for child in value.values():
            reject_legacy_fields(child)
    elif isinstance(value, list):
        for child in value:
            reject_legacy_fields(child)


def make_baseline(ego_route, target_route, cfg, *, map_name, opendrive_sha256,
                  ego_anchors=None, target_anchors=None):
    validate_settings(cfg)
    ego_route, target_route = normalized_route(ego_route), normalized_route(target_route)
    b = dict(schema_version=1, scenario_code='2d', map_name=map_name,
             opendrive_sha256=opendrive_sha256, ego_route=copy.deepcopy(ego_route),
             target_route=copy.deepcopy(target_route),
             ego_anchors=copy.deepcopy(ego_anchors or [ego_route[0], ego_route[-1]]),
             target_anchors=copy.deepcopy(target_anchors or [target_route[0], target_route[-1]]),
             engineering=copy.deepcopy(cfg['engineering']), geometry=build_geometry(ego_route, target_route, cfg))
    b['baseline_fingerprint'] = digest(b)
    return b


def validate_baseline(baseline, cfg):
    reject_legacy_fields(baseline)
    verify_fingerprint(baseline, 'baseline_fingerprint')
    if baseline.get('scenario_code') != '2d' or baseline.get('schema_version') != 1:
        raise ValueError('not a 2d baseline')
    active = lambda values: {k: v for k, v in values.items() if k not in RETIRED_DRAWING_KEYS | {'entry_speed_tolerance_kmh'}}
    if active(baseline['engineering']) != active(cfg['engineering']):
        raise ValueError('baseline engineering settings changed; save a new baseline')
    if build_geometry(baseline['ego_route'], baseline['target_route'], cfg) != baseline['geometry']:
        raise ValueError('baseline geometry mismatch')


def definition_name(speed, weather):
    return '{}{:02d}_{}'.format(DEFINITION_PREFIX, int(speed), weather)


def definition_paths(directory):
    """Include archived schema-2 names without selecting generic 2d JSONs."""
    directory = Path(directory).expanduser()
    return sorted(path for prefix in (DEFINITION_PREFIX, LEGACY_DEFINITION_PREFIX)
                  for path in directory.glob(prefix+'??_*.json'))


def canonical_definition(document):
    """Verify an existing identity, then change only its name and fingerprint."""
    if (document.get('scenario_code') != '2d' or document.get('schema_version') != 2
            or 'controller' in document or 'commanded_speed_kmh' in document):
        raise ValueError('old controller-specific definition; expand the baseline into shared conditions')
    verify_fingerprint(document, 'condition_fingerprint')
    name = definition_name(document['required_speed_kmh'], document['weather_id'])
    legacy_name = LEGACY_DEFINITION_PREFIX+name[len(DEFINITION_PREFIX):]
    if document.get('scenario') not in (name, legacy_name):
        raise ValueError('unexpected 2d scenario name: '+str(document.get('scenario')))
    result = dict(document, scenario=name)
    result.pop('condition_fingerprint')
    result['condition_fingerprint'] = digest(result)
    return result


def make_definitions(baseline, cfg, speed=None, weather=None):
    """One speed/weather condition set, shared byte-for-byte by both algorithms."""
    validate_settings(cfg)
    validate_baseline(baseline, cfg)
    if speed is not None and speed not in ENTRY_SPEEDS:
        raise ValueError('unknown required entry speed')
    profiles = weather_catalog.all_weather_profiles()
    if weather_catalog.WEATHER_CATALOG_VERSION != 2 or len(profiles) != 120:
        raise ValueError('2d requires weather catalog version 2 with 120 profiles')
    if weather:
        profiles = [p for p in profiles if p['id'] == weather]
        if not profiles:
            raise ValueError('unknown weather: '+weather)
    docs = []
    for v in ([speed] if speed is not None else ENTRY_SPEEDS):
        for profile in profiles:
            name = definition_name(v, profile['id'])
            d = dict(schema_version=2, scenario_code='2d', scenario=name,
                     baseline=copy.deepcopy(baseline), required_speed_kmh=v,
                     target_speed_kmh=20., ttc_window_s=[4.5, 5.5], section_speed_limit_kmh=40.,
                     weather_id=profile['id'], weather=copy.deepcopy(profile['parameters']),
                     weather_catalog_version=2, ego_blueprint=cfg['ego_blueprint'],
                     target_blueprint=cfg['target_blueprint'], engineering=copy.deepcopy(cfg['engineering']),
                     screening_only=True, official_speed_case=False, collision_enhancement_enabled=False)
            d['condition_fingerprint'] = digest(d)
            docs.append(d)
    return docs


def validate_definitions(paths, cfg, require_complete=False):
    """Validate each supplied condition; small runsets are first-class inputs."""
    if not paths:
        raise ValueError('no 2d definitions; use --baseline or an expanded --input_dir')
    docs, seen, baseline_cache = [], set(), {}
    for path in paths:
        d = json.loads(Path(path).read_text(encoding='utf-8'))
        reject_legacy_fields(d)
        canonical = canonical_definition(d)
        name = canonical['scenario']
        fp = d['baseline']['baseline_fingerprint']
        if fp not in baseline_cache:
            expected = make_definitions(d['baseline'], cfg)
            baseline_cache[fp] = {x['scenario']: x for x in expected}
        if (Path(path).stem != d['scenario'] or name in seen
                or canonical != baseline_cache[fp].get(name)):
            raise ValueError('definition differs from baseline/config/weather: '+str(path))
        seen.add(name)
        docs.append(d)
    if require_complete and (len(baseline_cache) != 1 or seen != set(next(iter(baseline_cache.values())))):
        raise ValueError('incomplete shared 8 speeds x 120 weather matrix')
    return sorted(docs, key=lambda d: d['scenario'])


def live_ttc_s(remaining_arc_m, speed_mps, stop_speed_mps):
    if remaining_arc_m <= 0:
        return 0.0
    if speed_mps <= stop_speed_mps:
        return None
    return remaining_arc_m/speed_mps


def release_transition(previous_sample_ttc, ttc, saw_defined_above,
                       already_released=False, ttc_latched=False, low=4.5, high=5.5):
    if already_released or ttc_latched or ttc is None:
        return 'hold'
    if previous_sample_ttc is not None and previous_sample_ttc > high and ttc < low:
        return 'skipped'
    if low <= ttc <= high:
        return 'release'
    if ttc < low and not saw_defined_above:
        return 'never_observed'
    return 'hold'


def final_status(latches, released, controller, gate_crossed, lane_success, fail_reasons):
    speed_failed = 'required_speed_not_reached' in fail_reasons
    # An algorithm speed failure must not become INVALID just because its TTC window
    # was missed. Simulator/sensor/target faults still invalidate the trial.
    ttc_reasons = {'ttc_window_skipped', 'ttc_window_never_observed'}
    blocking = [r for r in latches if not (speed_failed and r in ttc_reasons)]
    if blocking:
        return 'INVALID', blocking[0]
    if 'collision_with_target' in fail_reasons:
        return 'FAILED', 'collision_with_target'
    if speed_failed:
        return 'FAILED', 'required_speed_not_reached'
    if not released and not gate_crossed:
        return 'FAILED', 'corresponding_lane_not_entered'
    if not released:
        return 'INVALID', 'ttc_window_not_entered'
    reasons = set(fail_reasons)
    if not lane_success:
        reasons.add('corresponding_lane_not_entered')
    ordered = [r for r in FAIL_PRIORITY if r in reasons]
    return ('FAILED', ordered[0]) if ordered else ('PASS', None)


def speed_control(desired_mps, measured_mps, eng):
    error = desired_mps-measured_mps
    return (max(0., min(eng['speed_hold_throttle_max'], eng['speed_hold_throttle_gain']*error)),
            max(0., min(eng['speed_hold_brake_max'], -eng['speed_hold_brake_gain']*error)))



def reference_cruise_control(desired_mps, measured_mps, eng, integral, feedforward=0.):
    """PI cruise compensates rolling resistance; yielding keeps the brake law."""
    if desired_mps <= 0:
        throttle, brake = speed_control(0., measured_mps, eng)
        return throttle, brake, 0.
    error = desired_mps-measured_mps
    kp = eng['speed_hold_throttle_gain']
    limit = eng['speed_hold_throttle_max']
    lower = -eng['speed_hold_brake_max']*kp/eng['speed_hold_brake_gain']
    # Integrate only when the output can respond, and keep the learned drag
    # compensation bounded. These are reference-controller gains, not scoring.
    effort = kp*error+integral+feedforward
    if lower < effort < limit or (effort >= limit and error < 0) or (effort <= lower and error > 0):
        integral = max(lower, min(limit, integral+0.1*error*eng['fixed_delta_seconds']))
    effort = kp*error+integral+feedforward
    return (max(0., min(limit, effort)),
            max(0., min(eng['speed_hold_brake_max'], -effort*eng['speed_hold_brake_gain']/kp)), integral)



class CrossRightTrial:
    """Sample-driven state machine; observations contain no simulator objects."""
    def __init__(self, definition, controller='behavior', runtime_baseline=None):
        if controller not in ('behavior', 'tcp'):
            raise ValueError('unknown controller')
        self.d = definition
        self.b, self.eng = runtime_baseline or definition['baseline'], definition['engineering']
        self.g = self.b['geometry']
        self.controller = controller
        self.entry_speed_tolerance = self.eng.get('entry_speed_tolerance_kmh', self.eng['speed_hold_tolerance_kmh'])
        self.required_speed = definition['required_speed_kmh']
        self.speed_check_met = None
        self.speed_check_time = self.speed_at_check = None
        self.speed_ever_qualified = False
        self.maximum_entry_speed = self.longest_speed_hold = 0.
        self.latches, self.failures = [], set()
        self.previous_sample_ttc = None
        self.saw_defined_above = self.ttc_latched = False
        self.release_time = self.ttc_at_release = self.speed_at_release = None
        self.start_time = self.stable_since = None
        self.last_sample_time = self.termination_reason = None
        self.t_enter = self.t_clear = self.gate_time = None
        self.gate_frame = None
        self.gate_crossed = self.lane_success = self.early_passage = self.done = False
        self.target_speed_error = None
        self.target_motion_fault_frames = {}
        self.collision_frame = None
        self.lane_evidence = {}

    @property
    def released(self):
        return self.release_time is not None

    def latch(self, reason, immediate=False):
        if reason not in self.latches:
            self.latches.append(reason)
        if immediate:
            self.done = True
            self.termination_reason = reason

    def in_conflict(self, corner):
        for role in ('ego', 'target'):
            p = project(corner, self.b[role+'_route'], extend_ends=True)
            if not (self.g[role+'_conflict_near_s'] <= p['s'] <= self.g[role+'_conflict_far_s']
                    and p['lateral'] <= self.g[role+'_half_width_m']):
                return False
        return True

    def add_events(self, collision_target=False, failure_reasons=(), collision_frame=None):
        if collision_target:
            self.failures.add('collision_with_target')
            if collision_frame is not None:
                self.collision_frame = collision_frame if self.collision_frame is None else min(self.collision_frame, collision_frame)
                # Sensor callbacks can arrive after the motion sample. A target
                # fault already present before contact remains INVALID; a fault
                # first observed at/after contact must not hide the collision.
                self.latches = [reason for reason in self.latches
                    if not (reason in self.target_motion_fault_frames
                            and self.target_motion_fault_frames[reason] is not None
                            and self.target_motion_fault_frames[reason] >= self.collision_frame)]
        self.failures.update(failure_reasons)

    def target_motion_fault(self, reason, frame):
        self.target_motion_fault_frames.setdefault(reason, frame)
        if 'collision_with_target' not in self.failures:
            self.latch(reason)

    def update(self, now, obs):
        if self.done:
            return 'hold'
        if self.start_time is None:
            self.start_time = now
        self.last_sample_time = now
        self.add_events(obs.get('collision_target', False), collision_frame=obs.get('collision_target_frame'))
        ego = project(obs['ego'], self.b['ego_route'], extend_ends=True)
        ttc = live_ttc_s(self.g['ego_conflict_s']-ego['s'], obs['ego_speed_mps'], self.eng['stop_speed_mps'])
        qualified = False
        if not self.released and self.speed_check_met is None:
            observed = obs['ego_speed_mps']*3.6
            self.maximum_entry_speed = max(self.maximum_entry_speed, observed)
            error = abs(observed-self.required_speed)
            if error <= self.entry_speed_tolerance+1e-9:
                if self.stable_since is None:
                    self.stable_since = now
                duration = now-self.stable_since
                self.longest_speed_hold = max(self.longest_speed_hold, duration)
                # The initializer establishes stable speed before this clock
                # starts. At the TTC window, check the actual speed without
                # demanding a second hold period from the tested controller.
                qualified = True
                self.speed_ever_qualified = self.speed_ever_qualified or qualified
            else:
                self.stable_since = None
        transition = release_transition(self.previous_sample_ttc, ttc, self.saw_defined_above,
                                        self.released, self.ttc_latched)
        if transition in ('release', 'skipped', 'never_observed') and self.speed_check_met is None:
            self.speed_check_met = qualified
            self.speed_check_time, self.speed_at_check = now, obs['ego_speed_mps']*3.6
            if not qualified:
                self.failures.add('required_speed_not_reached')
        if transition == 'release':
            self.release_time, self.ttc_at_release, self.speed_at_release = now, ttc, obs['ego_speed_mps']*3.6
        elif transition in ('skipped', 'never_observed'):
            self.latch('ttc_window_'+transition)
            self.ttc_latched = True
        if ttc is not None and ttc > 5.5:
            self.saw_defined_above = True
        self.previous_sample_ttc = ttc
        ego_corners, target_corners = obs['ego_corners'], obs['target_corners']
        if self.t_enter is None and any(self.in_conflict(p) for p in ego_corners):
            self.t_enter = now
        if self.released and self.t_clear is None:
            pr = project(obs['target'], self.b['target_route'], extend_ends=True)
            if (self.g['target_wait_s'] <= pr['s'] <= self.g['target_conflict_far_s']
                    and pr['lateral'] > self.eng['target_lateral_tolerance_m']):
                self.target_motion_fault('target_left_route', obs.get('frame'))
            if now-self.release_time > self.eng['target_speed_grace_s']:
                error = abs(obs['target_speed_mps']*3.6-20.)
                self.target_speed_error = max(self.target_speed_error or 0., error)
                if error > 2:
                    self.target_motion_fault('target_speed_not_held', obs.get('frame'))
            if (all(project(p, self.b['target_route'], extend_ends=True)['s'] > self.g['target_conflict_far_s'] for p in target_corners)
                    and not any(self.in_conflict(p) for p in target_corners)):
                self.t_clear = now
        projections = [project(p, self.b['ego_route'], extend_ends=True) for p in ego_corners]
        if self.t_clear is None and any(p['s'] >= self.g['ego_conflict_near_s'] and
                                       p['lateral'] <= 2*self.g['ego_half_width_m'] for p in projections):
            self.early_passage = True
        if self.early_passage or (self.t_enter is not None and (self.t_clear is None or self.t_clear >= self.t_enter)):
            self.failures.add('yield_not_established')
        if not self.gate_crossed:
            self.add_events(obs.get('collision_target', False), obs.get('failure_reasons', ()))
            if obs['ego_speed_mps']*3.6 > 40:
                self.failures.add('section_speed')
        else:
            self.add_events(obs.get('collision_target', False))
        lateral = max(p['lateral'] for p in projections)
        bound = self.g['ego_half_width_m']+self.eng['boundary_tolerance_m']
        heading = abs(angle_delta(obs['ego_yaw'], at(self.b['ego_route'], ego['s'])['yaw']))
        crossed = all(p['s'] >= self.g['exit_gate_s'] for p in projections) and lateral <= bound and heading <= self.eng['max_heading_error_deg']
        if crossed:
            if not self.gate_crossed:
                self.gate_crossed, self.gate_time, self.gate_frame = True, now, obs.get('frame')
                self.lane_evidence = dict(sim_time_s=now, max_lateral_m=lateral,
                                          lateral_bound_m=bound, heading_error_deg=heading)
            if self.t_clear is not None and now > self.t_clear:
                self.lane_success = True
        yield_complete = self.t_clear is not None or bool(self.failures & {'yield_not_established', 'collision_with_target'})
        if 'collision_with_target' in self.failures:
            self.termination_reason = 'collision_with_target'
        elif 'required_speed_not_reached' in self.failures:
            self.termination_reason = 'required_speed_not_reached'
        elif self.gate_crossed and (not self.released or yield_complete):
            self.termination_reason = 'exit_gate_crossed'
        elif now-self.start_time >= self.g['timeout_s']:
            self.termination_reason = 'timeout'
        self.done = self.termination_reason is not None
        return transition

    def result(self):
        reasons = set(self.failures)
        speed_met = (None if self.start_time is None else
                     self.speed_ever_qualified if self.speed_check_met is None else self.speed_check_met)
        if speed_met is False:
            reasons.add('required_speed_not_reached')
        # A trial closed on entry-speed failure did not attempt the subsequent
        # yield/exit task. Preserve observed violations without inventing these
        # later failures simply because the run ended at its entry check.
        if self.start_time is not None and self.termination_reason not in ('required_speed_not_reached', 'collision_with_target'):
            if (self.released and not self.lane_success) or (not self.released and not self.gate_crossed):
                reasons.add('corresponding_lane_not_entered')
            if self.released and self.t_clear is None:
                reasons.add('yield_not_established')
        status, primary = final_status(self.latches, self.released, self.controller,
                                       self.gate_crossed, self.lane_success, reasons)
        invalid = list(self.latches) if status == 'INVALID' else []
        if status == 'INVALID' and primary not in invalid:
            invalid.append(primary)
        return dict(result_status=status, primary_reason=primary, trial_valid=status != 'INVALID',
                    termination_reason=self.termination_reason,
                    elapsed_sim_time_s=None if self.last_sample_time is None else self.last_sample_time-self.start_time,
                    **{'pass': None if status == 'INVALID' else status == 'PASS'},
                    failure_reasons=[r for r in FAIL_PRIORITY if r in reasons], invalid_reasons=invalid,
                    latches=list(self.latches), controller=self.controller, required_speed_kmh=self.required_speed,
                    commanded_speed_kmh=self.required_speed if self.controller == 'behavior' else None,
                    speed_requirement_met=speed_met,
                    speed_requirement_evidence=dict(required_speed_kmh=self.required_speed,
                        tolerance_kmh=self.entry_speed_tolerance,
                        confirm_s=0., initialization_confirm_s=self.eng['speed_hold_confirm_s'], met=speed_met,
                        checked_at_sim_time_s=self.speed_check_time, observed_speed_kmh=self.speed_at_check,
                        maximum_entry_speed_kmh=self.maximum_entry_speed,
                        longest_speed_hold_s=self.longest_speed_hold),
                    observed_speed_kmh_at_release=self.speed_at_release, ttc_at_release_s=self.ttc_at_release,
                    release_sim_time_s=self.release_time,
                    closure_mismatch_s=None if not self.released else 5.5-self.ttc_at_release,
                    target_speed_error_kmh=self.target_speed_error,
                    collision_frame=self.collision_frame, target_motion_fault_frames=dict(self.target_motion_fault_frames),
                    ttc_reference='actor_origin',
                    target_speed_hold='carla_constant_velocity', yield_definition='engineering_body_clearance_v1',
                    yield_evidence=dict(t_enter=self.t_enter, t_clear=self.t_clear,
                        early_passage=self.early_passage, early_passage_full_width_m=2*self.g['ego_half_width_m'],
                        clear_before_enter=self.t_clear is not None and (self.t_enter is None or self.t_clear < self.t_enter),
                        **{k: v for k, v in self.g.items() if '_conflict_' in k}),
                    lane_entry_evidence=dict(self.lane_evidence, gate_crossed=self.gate_crossed,
                                            lane_success=self.lane_success, gate_frame=self.gate_frame,
                                            exit_gate_s=self.g['exit_gate_s']),
                    source_geometry_observations={k: self.g.get(k) for k in (
                        'directed_arm_count', 'right_side_lateral_m', 'target_ray_max_error_m',
                        'signal_query_clear', 'section_speed_limit_verified')},
                    screening_only=True, official_speed_case=False, gb_t_conformity_claimed=False,
                    physical_field_test_compliance_claimed=False, instrument_accuracy_claimed=False,
                    repetition_index=1, repetitions_required_by_5_2_1=3,
                    controller_role='system_under_test',
                    collision_enhancement_enabled=False, sampling_hz=20, clause_5_2_2_complete=False)
