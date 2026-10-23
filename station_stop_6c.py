"""6c immutable geometry/conditions and controller-independent parking scoring.

No CARLA, Pygame or Torch imports: expansion and replay tests work offline.
Distances are longitudinal projections in the straight station frame, from
actor origin to the fixed goal. They are not Euclidean distances to the curb.
"""
from __future__ import annotations

import copy
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path

from stf_weather_catalog import all_weather_profiles


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def seal(value, key):
    result = copy.deepcopy(value)
    result.pop(key, None)
    result[key] = digest(result)
    return result


def verify(value, key):
    raw = dict(value)
    if raw.pop(key, None) != digest(raw):
        raise ValueError(key + ' mismatch')


def finite(value, name, *, positive=False):
    if isinstance(value, bool):
        raise ValueError(name + ' must be numeric')
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(name + ' must be finite' + (' and positive' if positive else ''))
    return result


def checked_point(p):
    return {k: finite(p[k], k) for k in ('x', 'y', 'z')}


def angle_delta(a, b):
    return (a - b + 180) % 360 - 180


def distance(a, b):
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def frame_coordinates(p, goal):
    yaw = math.radians(goal['yaw'])
    dx, dy = p['x'] - goal['x'], p['y'] - goal['y']
    return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)


def frame_point(s, lateral, goal, z=None):
    yaw = math.radians(goal['yaw'])
    return dict(x=goal['x'] + s * math.cos(yaw) - lateral * math.sin(yaw),
                y=goal['y'] + s * math.sin(yaw) + lateral * math.cos(yaw),
                z=goal['z'] if z is None else z, yaw=goal['yaw'])


def footprint(pose, geometry):
    ext, offset = geometry['extent'], geometry['location']
    return [frame_point(offset['x'] + x * ext['x'], offset['y'] + y * ext['y'], pose)
            for x, y in ((-1, -1), (-1, 1), (1, 1), (1, -1))]


def target_pose(a, b, clicked, forward_yaw):
    checked_point(a); checked_point(b); checked_point(clicked)
    if distance(a, b) < 3:
        raise ValueError('站台边界 A/B 至少相距 3 m')
    yaw = math.degrees(math.atan2(b['y'] - a['y'], b['x'] - a['x']))
    if abs(angle_delta(yaw, forward_yaw)) > 90:
        yaw = (yaw + 180) % 360
    return dict(checked_point(clicked), yaw=yaw)


def validate_spec(spec):
    if spec['evaluation']['scope'] != 'parking_only':
        raise ValueError('6c currently implements parking_only; door/step compliance is not supported')
    for group in ('engineering', 'evaluation'):
        for key, value in spec[group].items():
            if key != 'scope':
                finite(value, group + '.' + key, positive=True)
    if spec['engineering']['fixed_delta_seconds'] > .1:
        raise ValueError('fixed_delta_seconds must be <= 0.1')
    if spec['evaluation']['hold_s'] < spec['evaluation']['stop_confirm_s']:
        raise ValueError('hold_s must include the stop confirmation interval')
    for key in ('initial_distances_m', 'initial_speeds_kmh'):
        values = spec['matrix'][key]
        if not values or len(values) != len(set(values)):
            raise ValueError('empty or duplicate matrix axis: ' + key)
        for value in values:
            finite(value, key, positive=True)
    if spec['matrix']['weather_catalog'] != 'all':
        raise ValueError('weather_catalog must be all; use --weather to select a diagnostic run')
    if not str(spec['ego_blueprint']).startswith('vehicle.'):
        raise ValueError('invalid ego blueprint')
    if len(spec['weather_profiles']) != 120 or len({w['id'] for w in spec['weather_profiles']}) != 120:
        raise ValueError('weather catalog must contain 120 unique IDs')


def generation_spec(cfg):
    spec = {key: copy.deepcopy(cfg[key]) for key in
            ('matrix', 'evaluation', 'engineering', 'ego_blueprint')}
    spec['weather_profiles'] = copy.deepcopy(all_weather_profiles())
    validate_spec(spec)
    return spec


def load_settings(path):
    import yaml
    cfg = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if cfg.get('schema_version') != 1:
        raise ValueError('6c schema_version must be 1')
    generation_spec(cfg)
    if cfg.get('tcp_mode') not in ('hybrid', 'raw_goal'):
        raise ValueError('tcp_mode must be hybrid or raw_goal')
    return cfg


def geometry_metrics(b, eng):
    if b.get('scenario_code') != '6c' or b.get('schema_version') != 1:
        raise ValueError('not a 6c baseline')
    route, goal = b['ego_route'], b['stop_pose']
    if len(route) < 3:
        raise ValueError('需要至少三个稠密路线点')
    checked_point(goal); finite(goal['yaw'], 'goal yaw')
    a, end = b['station_edge']
    expected = target_pose(a, end, goal, route[-1]['yaw'])
    if abs(angle_delta(expected['yaw'], goal['yaw'])) > .01:
        raise ValueError('停车方向必须由站台边界和行驶方向确定')
    projections = []
    travelled = [0.]
    for p in route:
        checked_point(p)
        finite(p['width'], 'lane width', positive=True)
        finite(p['yaw'], 'route yaw')
        projections.append(frame_coordinates(p, goal))
    for i, (p, q) in enumerate(zip(route, route[1:])):
        step = distance(p, q)
        if not .01 < step <= 2.1 or projections[i + 1][0] <= projections[i][0]:
            raise ValueError('接近路线必须稠密、连续并沿站台方向前进')
        travelled.append(travelled[-1] + step)
    # A curved junction connector may precede the station. The last 8 m of
    # drawn approach must be a straight lane aligned with the stopping pose.
    final_zone = [i for i, value in enumerate(travelled)
                  if value >= travelled[-1] - min(8., travelled[-1]) - 1e-7]
    if (any(abs(angle_delta(route[i]['yaw'], goal['yaw'])) > eng['route_heading_tolerance_deg']
            for i in final_zone)
            or max(projections[i][1] for i in final_zone)
            - min(projections[i][1] for i in final_zone) > .5):
        raise ValueError('Pn 前至少 8 m 应在与站台平行的直车道；请继续沿连接弯道后的车道画 P 点')
    if -projections[-1][0] < eng['minimum_docking_length_m']:
        raise ValueError('Pn 到停车目标的纵向距离不足；请将 Pn 放在更上游')
    if abs(projections[-1][1]) > eng['maximum_goal_offset_m']:
        raise ValueError('停车目标离接近车道过远')
    edges = [frame_coordinates(p, goal) for p in (a, end)]
    geom = b['vehicle_geometry']
    for group in ('extent', 'location'):
        for key in ('x', 'y', 'z'):
            finite(geom[group][key], 'vehicle geometry', positive=group == 'extent')
    for key in ('wheelbase_m', 'max_steer_angle_deg'):
        if key in geom:
            finite(geom[key], key, positive=True)
    for pose in geom.get('camera_mounts', {}).values():
        checked_point(pose)
        for key in ('pitch', 'yaw', 'roll'):
            finite(pose.get(key, 0.), 'camera ' + key)
    edge_start, edge_end = sorted(p[0] for p in edges)
    if not edge_start <= 0 <= edge_end:
        raise ValueError('停车目标 G 不在站台边界 A/B 的纵向范围内；请移动 G 或修正 A/B')
    # A/B fixes the station heading and longitudinal span, not a side of the
    # approach lane or a body-clearance threshold.
    return dict(docking_length_m=-projections[-1][0])


def spawn_for_distance(b, initial_distance):
    route, goal = b['ego_route'], b['stop_pose']
    wanted = -float(initial_distance)
    for i, (a, z) in enumerate(zip(route, route[1:])):
        sa, _ = frame_coordinates(a, goal)
        sz, _ = frame_coordinates(z, goal)
        if sa <= wanted < sz or (i == len(route) - 2 and wanted == sz):
            t = (wanted - sa) / (sz - sa)
            p = dict(a)
            p.update({key: a[key] + t * (z[key] - a[key]) for key in ('x', 'y', 'z', 'width')})
            p['yaw'] = a['yaw'] + t * angle_delta(z['yaw'], a['yaw'])
            suffix = [p] + [copy.deepcopy(q) for q in route[i + 1:] if frame_coordinates(q, goal)[0] > wanted + 1e-7]
            if len(suffix) < 2:
                raise ValueError('起点必须在 Pn 上游')
            return p, suffix
    raise ValueError('路线未覆盖距 S {} m 的起点；请延长基准路线'.format(initial_distance))


def _trace_forward_segment(start, goal):
    """Follow CARLA's directed lane topology, including junction connectors."""
    serial = itertools.count()
    start_location, goal_location = start.transform.location, goal.transform.location
    direct_distance = start_location.distance(goal_location)
    limit = min(300., max(30., direct_distance * 2. + 15.))
    queue = [(direct_distance, next(serial), start, [], 0.)]
    visited = set()
    for _ in range(6000):
        if not queue:
            break
        _, _, current, path, travelled = heapq.heappop(queue)
        token = (current.road_id, current.section_id, current.lane_id, round(current.s, 1))
        if token in visited:
            continue
        visited.add(token)
        here = current.transform.location
        remaining = here.distance(goal_location)
        if ((current.road_id, current.lane_id) == (goal.road_id, goal.lane_id)
                and remaining <= 1.05):
            f = current.transform.get_forward_vector()
            delta = goal_location - here
            if delta.x * f.x + delta.y * f.y >= -.05:
                return path + ([goal] if remaining > .05 else [])
        if travelled >= limit:
            continue
        for nxt in current.next(.75):
            step = here.distance(nxt.transform.location)
            cost = travelled + step
            if .01 < step <= 2.1 and cost <= limit:
                heapq.heappush(queue, (cost + nxt.transform.location.distance(goal_location),
                                       next(serial), nxt, path + [nxt], cost))
    return None


def trace_straight_approach(map_obj, anchors, labels=None):
    """Sample the actual Driving lane center for this straight 6c approach.

    Follow directed waypoint.next links through real junction connectors.
    If a custom map breaks one such link, accept a geometrically verified
    straight Driving lane center only. Never invent a turn or lane change.
    """
    import carla

    if len(anchors) < 2:
        raise ValueError('先放 E 和至少一个顺行 P 点')

    def lane_anchor(point, label):
        coords = checked_point(point)
        wp = map_obj.get_waypoint(carla.Location(**coords), project_to_road=True,
                                  lane_type=carla.LaneType.Driving)
        if wp is None or distance(coords, dict(x=wp.transform.location.x,
                                               y=wp.transform.location.y)) > .75:
            raise ValueError('{}：未吸附到 Driving 车道中心，请重新选点'.format(label))
        return wp

    def route_point(wp):
        loc = wp.transform.location
        return dict(x=loc.x, y=loc.y, z=loc.z, yaw=wp.transform.rotation.yaw,
                    width=wp.lane_width, road_id=wp.road_id,
                    section_id=wp.section_id, lane_id=wp.lane_id)

    names = labels or (['E'] + ['P{}'.format(i) for i in range(1, len(anchors))])
    if len(names) != len(anchors):
        raise ValueError('anchor labels must match anchor count')
    waypoints = [lane_anchor(p, label) for p, label in zip(anchors, names)]
    route = [route_point(waypoints[0])]
    for i, (start_wp, goal_wp) in enumerate(zip(waypoints, waypoints[1:])):
        name = '{}→{}'.format(names[i], names[i + 1])
        start, goal = route_point(start_wp), route_point(goal_wp)
        length = distance(start, goal)
        if length <= 1.1:
            raise ValueError('{}：两个锚点过近（{:.2f} m），请将下一个 P 点向前移'.format(name, length))
        heading = math.degrees(math.atan2(goal['y'] - start['y'], goal['x'] - start['x']))
        direction_change = abs(angle_delta(start['yaw'], goal['yaw']))
        if direction_change > 90:
            raise ValueError('{}：两点的车道行驶方向相反（起点 {:.1f}°，终点 {:.1f}°，相差 {:.1f}°）；'
                             '起点 road/lane={}/{}, 终点={}/{}。请在同向车道重选 P 点；'
                             'road ID 不同本身不导致拒绝。'.format(
                                 name, start['yaw'], goal['yaw'], direction_change,
                                 start['road_id'], start['lane_id'],
                                 goal['road_id'], goal['lane_id']))
        connected = _trace_forward_segment(start_wp, goal_wp)
        if connected is not None:
            for wp in connected:
                actual = route_point(wp)
                step = distance(route[-1], actual)
                if step <= .05:
                    continue
                if step > 2.1 or abs(actual['z'] - route[-1]['z']) > .5:
                    raise ValueError('{}：CARLA 前向车道在 ({:.1f}, {:.1f}) 出现几何断缝；'
                                     '请检查地图。'.format(name, actual['x'], actual['y']))
                route.append(actual)
            continue
        if (abs(angle_delta(heading, start['yaw'])) > 5
                or abs(angle_delta(heading, goal['yaw'])) > 5):
            raise ValueError('{}：锚点连线与车道前进方向偏差过大（起点 {:.1f}°，终点 {:.1f}°）；'
                             '起点 road/lane={}/{}, 终点={}/{}。6c 接近路线在这里要求顺向直行。'.format(
                                 name, start['yaw'], goal['yaw'],
                                 start['road_id'], start['lane_id'],
                                 goal['road_id'], goal['lane_id']))
        count = math.ceil(length / .75)
        for j in range(1, count + 1):
            fraction = j / count
            sample = dict(x=start['x'] + (goal['x'] - start['x']) * fraction,
                          y=start['y'] + (goal['y'] - start['y']) * fraction,
                          z=start['z'] + (goal['z'] - start['z']) * fraction)
            wp = map_obj.get_waypoint(carla.Location(**sample), project_to_road=False,
                                      lane_type=carla.LaneType.Driving)
            if wp is None:
                raise ValueError('{}：在 ({:.1f}, {:.1f}) 未找到连续的 Driving 路面；'
                                 '请把相邻 P 点放在同一段实际可行驶道路上。'.format(
                                     name, sample['x'], sample['y']))
            actual = route_point(wp)
            offset = distance(sample, actual)
            if (offset > .55 or abs(angle_delta(actual['yaw'], heading)) > 5
                    or abs(actual['z'] - sample['z']) > .5 or actual['width'] <= 0):
                raise ValueError('{}：在 ({:.1f}, {:.1f}) 偏离直车道中心 {:.2f} m；'
                                 '起点 road/lane={}/{}，终点={}/{}。请检查是否跨车道或选错方向。'.format(
                                     name, sample['x'], sample['y'], offset,
                                     start['road_id'], start['lane_id'],
                                     goal['road_id'], goal['lane_id']))
            step = distance(route[-1], actual)
            progress = ((actual['x'] - route[-1]['x']) * math.cos(math.radians(heading))
                        + (actual['y'] - route[-1]['y']) * math.sin(math.radians(heading)))
            if step < .05:
                continue
            if step > 1.25 or progress <= 0:
                raise ValueError('{}：在 ({:.1f}, {:.1f}) 车道中心出现断缝或反向跳变；'
                                 '请检查该段地图拓扑。'.format(name, sample['x'], sample['y']))
            route.append(actual)
    return route


def extend_approach_upstream(map_obj, route, goal, farthest_distance):
    """Follow the Driving lane behind E until the farthest spawn is covered.

    Prefer CARLA predecessor links. A broken predecessor link may be bridged
    only by short samples on a verified straight lane, as for E→P tracing.
    """
    import carla

    wanted = -float(farthest_distance)
    if frame_coordinates(route[0], goal)[0] <= wanted + 1e-7:
        return route
    first = route[0]
    current = map_obj.get_waypoint(carla.Location(x=first['x'], y=first['y'], z=first['z']),
                                   project_to_road=True, lane_type=carla.LaneType.Driving)
    if current is None or distance(first, dict(x=current.transform.location.x,
                                                y=current.transform.location.y)) > .75:
        raise ValueError('E 处未找到可向上游追溯的 Driving 车道')

    def route_point(wp):
        loc = wp.transform.location
        return dict(x=loc.x, y=loc.y, z=loc.z, yaw=wp.transform.rotation.yaw,
                    width=wp.lane_width, road_id=wp.road_id,
                    section_id=wp.section_id, lane_id=wp.lane_id)

    def valid_predecessor(wp):
        here, before = route_point(current), route_point(wp)
        step = distance(here, before)
        return (.05 < step <= 1.25 and abs(here['z'] - before['z']) <= .5
                and before['width'] > 0
                and frame_coordinates(before, goal)[0] < frame_coordinates(here, goal)[0] - .05
                and abs(angle_delta(before['yaw'], here['yaw'])) <= 45.)

    prefix = []
    for _ in range(500):
        if frame_coordinates(route_point(current), goal)[0] <= wanted + 1e-7:
            return list(reversed(prefix)) + route
        previous = getattr(current, 'previous', None)
        candidates = [wp for wp in previous(.75) if valid_predecessor(wp)] if callable(previous) else []
        unique = {}
        for wp in candidates:
            point = route_point(wp)
            token = (point['road_id'], point['section_id'], point['lane_id'],
                     round(point['x'], 2), round(point['y'], 2))
            unique[token] = wp
        if len(unique) > 1:
            raise ValueError('E 上游车道存在分叉，无法自动确定距 S {:.0f} m 的起点；请将 E 放在分叉上游'.format(
                farthest_distance))
        if unique:
            predecessor = next(iter(unique.values()))
        else:
            here = route_point(current)
            yaw = math.radians(here['yaw'])
            sample = carla.Location(x=here['x'] - .75 * math.cos(yaw),
                                    y=here['y'] - .75 * math.sin(yaw), z=here['z'])
            predecessor = map_obj.get_waypoint(sample, project_to_road=False,
                                               lane_type=carla.LaneType.Driving)
            if (predecessor is None or not valid_predecessor(predecessor)
                    or distance(dict(x=sample.x, y=sample.y), route_point(predecessor)) > .55
                    or abs(angle_delta(route_point(predecessor)['yaw'], here['yaw'])) > 5.):
                raise ValueError('E 上游 {:.1f} m 处无连续的顺行 Driving 车道，无法覆盖距 S {:.0f} m 的起点；'
                                 '请将 E 放到更上游的连通车道'.format(
                                     frame_coordinates(first, goal)[0]
                                     - frame_coordinates(here, goal)[0], farthest_distance))
        current = predecessor
        prefix.append(route_point(current))
    raise ValueError('E 上游追溯超过 500 个路点，仍未覆盖距 S {:.0f} m 的起点'.format(
        farthest_distance))


def make_baseline(map_obj, draft, cfg):
    b = copy.deepcopy(draft)
    b.update(schema_version=1, scenario_code='6c', map_name=map_obj.name,
             opendrive_sha256=hashlib.sha256(map_obj.to_opendrive().encode()).hexdigest(),
             ego_blueprint=cfg['ego_blueprint'])
    b['drawn_ego_start'] = copy.deepcopy(b['ego_start'])
    b['ego_route'] = trace_straight_approach(map_obj, [b['ego_start']] + b['route_anchors'])
    b['route_geometry_source'] = 'directed_topology_with_verified_straight_fallback'
    b['stop_pose'] = target_pose(*b['station_edge'], b['stop_pose'], b['ego_route'][-1]['yaw'])
    b['ego_route'] = extend_approach_upstream(
        map_obj, b['ego_route'], b['stop_pose'], max(cfg['matrix']['initial_distances_m']))
    b['ego_start'] = copy.deepcopy(b['ego_route'][0])
    b['geometry'] = geometry_metrics(b, cfg['engineering'])
    for value in cfg['matrix']['initial_distances_m']:
        spawn_for_distance(b, value)
    return seal(b, 'baseline_fingerprint')


def build_definitions(b, spec):
    verify(b, 'baseline_fingerprint'); validate_spec(spec)
    geometry = geometry_metrics(b, spec['engineering'])
    if geometry != b['geometry'] or b['ego_blueprint'] != spec['ego_blueprint']:
        raise ValueError('baseline geometry/vehicle differs from expansion specification')
    result = []
    for d in spec['matrix']['initial_distances_m']:
        spawn, route = spawn_for_distance(b, d)
        for v in spec['matrix']['initial_speeds_kmh']:
            profile = 'd{:g}_v{:g}'.format(d, v).replace('.', 'p')
            for weather in spec['weather_profiles']:
                row = dict(schema_version=1, scenario_code='6c',
                           scenario='scenario_6c_' + profile + '_' + weather['id'],
                           profile_id=profile, baseline=copy.deepcopy(b),
                           ego_start=copy.deepcopy(spawn), ego_route=copy.deepcopy(route),
                           initial_distance_m=d, initial_speed_kmh=v,
                           ego_blueprint=b['ego_blueprint'], weather_id=weather['id'],
                           weather=copy.deepcopy(weather['parameters']),
                           engineering=copy.deepcopy(spec['engineering']),
                           evaluation=copy.deepcopy(spec['evaluation']),
                           generation_fingerprint=digest(spec), screening_only=True,
                           physical_field_test_compliance_claimed=False)
                result.append(seal(row, 'condition_fingerprint'))
    if len({r['scenario'] for r in result}) != len(result):
        raise ValueError('duplicate condition names')
    return result


def validate_definitions(directory):
    directory = Path(directory)
    manifest = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
    verify(manifest, 'manifest_fingerprint')
    paths = sorted(directory.glob('scenario_6c_*.json'))
    if not paths:
        raise ValueError('no 6c definitions')
    documents = [json.loads(p.read_text(encoding='utf-8')) for p in paths]
    expected = {d['scenario']: d for d in build_definitions(documents[0]['baseline'], manifest['spec'])}
    if len(paths) != len(expected) or manifest['count'] != len(expected):
        raise ValueError('incomplete 6c matrix')
    if manifest['baseline_fingerprint'] != documents[0]['baseline']['baseline_fingerprint']:
        raise ValueError('manifest baseline mismatch')
    if manifest['conditions'] != {name: row['condition_fingerprint'] for name, row in expected.items()}:
        raise ValueError('manifest condition list differs from generation specification')
    for path, row in zip(paths, documents):
        verify(row, 'condition_fingerprint')
        if path.stem != row['scenario'] or row != expected.get(row['scenario']):
            raise ValueError('condition changed: ' + str(path))
    return documents, manifest


class StationStopEvaluator:
    """Pure observer. It never changes control or searches for a better stop."""
    def __init__(self, definition):
        self.d, self.e = definition, definition['evaluation']
        self.status, self.phase = 'RUNNING', 'SETUP'
        self.started = self.last_time = self.last_frame = None
        self.stop = self.first_stop = self.reverse_since = None
        self.reasons, self.events = [], []

    def end(self, status, reason, now=None):
        # A later platform/cleanup error cannot erase a demonstrated failure.
        if self.status == 'FAILED':
            return
        if self.status == 'PASS' and status == 'PASS':
            return
        if self.status == 'PASS':
            self.reasons = []
        self.status = self.phase = status
        if reason not in self.reasons:
            self.reasons.append(reason)
        self.events.append(dict(event=status, reason=reason, sim_time=now))

    def invalidate_initialization(self, reason):
        # A failed prerequisite means the trial was never valid, unlike an
        # operational error discovered after a demonstrated valid failure.
        self.status = self.phase = 'INVALID'
        self.reasons = [reason]
        self.events.append(dict(event='INVALID', reason=reason, sim_time=None))

    def start(self, now, frame):
        if self.started is not None:
            raise ValueError('trial already started')
        self.started = finite(now, 'start time')
        self.phase = 'ACTIVE'
        self.events.append(dict(event='START', sim_time=now, frame=frame))

    def errors(self, pose):
        goal = self.d['baseline']['stop_pose']
        longitudinal, lateral = frame_coordinates(pose, goal)
        return dict(longitudinal_error_m=longitudinal, lateral_error_m=lateral,
                    heading_error_deg=angle_delta(pose['yaw'], goal['yaw']))

    def update(self, now, frame, pose, speed_mps, signed_speed_mps,
               *, collision=False, outside=False, solid_line=False):
        if self.status != 'RUNNING':
            return
        if self.started is None:
            raise ValueError('call start after initialization has been verified')
        if (self.last_time is not None and (now <= self.last_time or frame != self.last_frame + 1
                or abs(now - self.last_time - self.d['engineering']['fixed_delta_seconds']) > 1e-4)):
            self.end('INVALID', 'nonconsecutive_observation', now); return
        try:
            checked_point(pose); finite(pose['yaw'], 'yaw')
            finite(now, 'time'); finite(speed_mps, 'speed'); finite(signed_speed_mps, 'signed speed')
            if speed_mps < 0:
                raise ValueError('negative speed magnitude')
        except (ValueError, KeyError, TypeError):
            self.end('INVALID', 'nonfinite_observation', now); return
        self.last_time, self.last_frame = now, frame
        for failed, reason in ((collision, 'collision'), (outside, 'outside_allowed_area'),
                               (solid_line, 'crossed_solid_line')):
            if failed:
                self.end('FAILED', reason, now); return
        if signed_speed_mps < -self.e['reverse_speed_mps']:
            if self.reverse_since is None:
                self.reverse_since = now
            if now - self.reverse_since >= self.e['reverse_duration_s'] - 1e-7:
                self.end('FAILED', 'reverse_adjustment', now); return
        else:
            self.reverse_since = None
        if self.first_stop is not None:
            if speed_mps > self.e['stop_speed_mps'] or distance(pose, self.first_stop['pose']) > self.e['stop_drift_m']:
                self.end('FAILED', 'moved_after_first_stop', now); return
            if abs(angle_delta(pose['yaw'], self.first_stop['pose']['yaw'])) > self.e['heading_tolerance_deg']:
                self.end('FAILED', 'rotated_after_first_stop', now); return
            current = self.errors(pose)
            if (abs(current['longitudinal_error_m']) > self.e['longitudinal_tolerance_m']
                    or abs(current['lateral_error_m']) > self.e['lateral_tolerance_m']
                    or abs(current['heading_error_deg']) > self.e['heading_tolerance_deg']):
                self.end('FAILED', 'pose_outside_tolerance_during_hold', now); return
            if now - self.first_stop['sim_time'] >= self.e['hold_s'] - 1e-7:
                self.end('PASS', 'first_stop_satisfied', now); return
        elif speed_mps <= self.e['stop_speed_mps']:
            if self.stop is None:
                self.stop = dict(sim_time=now, frame=frame, pose=copy.deepcopy(pose), **self.errors(pose))
                self.phase = 'STOP_CANDIDATE'
            if distance(pose, self.stop['pose']) > self.e['stop_drift_m']:
                # Creeping is not a stop. It cannot start the hold timer.
                self.stop = None
                self.phase = 'ACTIVE'
            elif now - self.stop['sim_time'] >= self.e['stop_confirm_s'] - 1e-7:
                self.first_stop = copy.deepcopy(self.stop)
                self.events.append(dict(event='FIRST_STOP', **self.first_stop))
                self.phase = 'HOLD'
                for field, limit, reason in (
                    ('longitudinal_error_m', 'longitudinal_tolerance_m', 'stop_longitudinal_error'),
                    ('lateral_error_m', 'lateral_tolerance_m', 'stop_lateral_error'),
                    ('heading_error_deg', 'heading_tolerance_deg', 'stop_heading_error')):
                    if abs(self.first_stop[field]) > self.e[limit]:
                        self.end('FAILED', reason, now); return
                # Also require the pose to remain acceptable during confirmation.
                current = self.errors(pose)
                if (abs(current['longitudinal_error_m']) > self.e['longitudinal_tolerance_m']
                    or abs(current['lateral_error_m']) > self.e['lateral_tolerance_m']
                    or abs(current['heading_error_deg']) > self.e['heading_tolerance_deg']):
                    self.end('FAILED', 'pose_changed_during_stop_confirmation', now); return
        else:
            self.stop = None
            self.phase = 'ACTIVE'
        if now - self.started >= self.e['timeout_s'] - 1e-7:
            self.end('FAILED', 'task_timeout', now)

    def result(self):
        return dict(result_status=self.status, trial_valid=self.status in ('PASS', 'FAILED'),
                    **{'pass': True if self.status == 'PASS' else False if self.status == 'FAILED' else None},
                    evaluation_scope=self.e['scope'], reasons=list(self.reasons),
                    first_stop=copy.deepcopy(self.first_stop), events=copy.deepcopy(self.events),
                    screening_only=True, physical_field_test_compliance_claimed=False,
                    standard_checks=dict(door_alignment='NOT_EVALUATED', step_gap='NOT_EVALUATED',
                                         door_open_delay='NOT_EVALUATED', general_rules='PARTIAL'))


def reference_path(definition):
    """Internal trajectory of the NAMED reference parking controller, not a test fixture command."""
    route = copy.deepcopy(definition['ego_route'])
    goal = definition['baseline']['stop_pose']
    s0, n0 = frame_coordinates(route[-1], goal)
    start_z = route[-1]['z']
    steps = max(2, int(math.ceil(-s0 / .4)))
    for i in range(1, steps + 1):
        u = i / steps
        blend = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
        p = frame_point(s0 * (1 - u), n0 * (1 - blend), goal,
                        start_z + u * (goal['z'] - start_z))
        route.append(p)
    return route


def reference_control(pose, speed_mps, definition, path):
    """Pure-pursuit lateral guidance plus distance-based deceleration; no reverse."""
    goal = definition['baseline']['stop_pose']
    longitudinal, _ = frame_coordinates(pose, goal)
    remaining = max(0., -longitudinal)
    eng = definition['engineering']
    nearest = min(range(len(path)), key=lambda i: distance(pose, path[i]))
    look = max(1.2, speed_mps * .6)
    target, accumulated = path[-1], 0.
    for a, b in zip(path[nearest:], path[nearest + 1:]):
        accumulated += distance(a, b)
        if accumulated >= look:
            target = b; break
    forward, side = frame_coordinates(target, pose)
    length2 = max(.1, forward * forward + side * side)
    geometry = definition['baseline']['vehicle_geometry']
    wheelbase = geometry.get('wheelbase_m', 2.8)
    max_steer = geometry.get('max_steer_angle_deg', 35.)
    steer = max(-1., min(1., math.atan2(2 * wheelbase * side, length2) / math.radians(max_steer)))
    desired = min(eng['reference_cruise_kmh'] / 3.6,
                  math.sqrt(2 * eng['reference_deceleration_mps2'] * max(0., remaining - .08)))
    # Avoid a late full-speed crawl: near the target the position loop approaches zero.
    desired = min(desired, remaining * .9)
    error = desired - speed_mps
    if remaining <= .12:
        return (0., steer, 1.)
    if error < -.1:
        return (0., steer, min(1., -error * .3))
    return (max(0., min(.55, .12 + error * .35)), steer, 0.)
