"""2d lane tracing with optional anchors and automatic target approach extension."""
import heapq
import itertools
import math


def _key(w):
    return w.road_id, w.section_id, w.lane_id, round(w.s, 3)


def _sample(w):
    p = w.transform.location
    return dict(x=p.x, y=p.y, z=p.z, yaw=w.transform.rotation.yaw,
                width=w.lane_width, road_id=w.road_id, section_id=w.section_id,
                lane_id=w.lane_id, road_s=w.s, is_junction=w.is_junction)


def trace_waypoints(map_obj, anchors):
    """Follow the directed lane graph; a start and an end are sufficient."""
    import carla
    if len(anchors) < 2:
        raise ValueError('请放置路线起点和终点')
    goals = []
    for p in anchors:
        if not all(math.isfinite(float(p[k])) for k in ('x', 'y', 'z')):
            raise ValueError('路点坐标无效')
        w = map_obj.get_waypoint(carla.Location(x=p['x'], y=p['y'], z=p['z']),
                                 project_to_road=True, lane_type=carla.LaneType.Driving)
        if w is None:
            raise ValueError('路点附近没有可行驶车道')
        if not goals or w.transform.location.distance(goals[-1].transform.location) > 1e-6:
            goals.append(w)
    if len(goals) < 2:
        raise ValueError('路线起点和终点不能相同')
    dense = [goals[0]]
    for goal in goals[1:]:
        start, serial = dense[-1], itertools.count()
        queue = [(0., next(serial), start, None, 0.)]
        visited, parents, points = set(), {}, {}
        while queue:
            _, _, current, parent, cost = heapq.heappop(queue)
            # Sampling at one metre gives a finite visited graph, including loops.
            key = (current.road_id, current.section_id, current.lane_id, round(current.s, 1))
            if key in visited:
                continue
            visited.add(key)
            parents[key], points[key] = parent, current
            distance = current.transform.location.distance(goal.transform.location)
            if (current.road_id, current.lane_id) == (goal.road_id, goal.lane_id) and distance <= 1.05:
                delta = goal.transform.location-current.transform.location
                forward = current.transform.get_forward_vector()
                if delta.x*forward.x+delta.y*forward.y >= -1e-6:
                    path, cursor = [], key
                    while parents[cursor] is not None:
                        path.append(points[cursor])
                        cursor = parents[cursor]
                    dense.extend(reversed(path))
                    if distance > 1e-6:
                        dense.append(goal)
                    break
            for nxt in current.next(1.):
                next_cost = cost+current.transform.location.distance(nxt.transform.location)
                heapq.heappush(queue, (next_cost+nxt.transform.location.distance(goal.transform.location),
                                      next(serial), nxt, key, next_cost))
        else:
            raise ValueError('无法沿当前车道到达下一路点，请检查行驶方向或补一个中间路点')
    return dense


def extend_target_upstream(waypoints, extra_m):
    """Add only the actual lane distance needed to place the waiting target."""
    result = list(waypoints)
    current = result[0]
    visited = {_key(current)}
    prefix = []
    remaining = extra_m
    while remaining > 1e-6:
        choices = [w for w in current.previous(min(1., remaining)) if _key(w) not in visited]
        if not choices:
            break  # Saving is still allowed; runtime reports an unavailable wait pose.
        yaw = current.transform.rotation.yaw
        nxt = min(choices, key=lambda w: (abs((w.transform.rotation.yaw-yaw+180) % 360-180), _key(w)))
        distance = current.transform.location.distance(nxt.transform.location)
        if distance <= 1e-6:
            break
        prefix.append(nxt)
        remaining -= distance
        visited.add(_key(nxt))
        current = nxt
    return list(reversed(prefix))+result


def route_samples(waypoints):
    return [_sample(w) for w in waypoints]
