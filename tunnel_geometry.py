"""CARLA lane geometry shared by tunnel expansion and runtime; no actors spawned."""

import heapq
import math

import carla

from tunnel_2a import MIN_TUNNEL_LENGTH_M, gate_signed_distance, make_route_gate, route_length_2d

TRACE_STEP_M = 1.0
TRACE_GOAL_TOLERANCE_M = 1.6


def _planar_distance(first, second):
    return math.hypot(float(first.x) - float(second.x), float(first.y) - float(second.y))


class TunnelRouteGeometry:
    @staticmethod
    def _waypoint_token(waypoint):
        location = waypoint.transform.location
        return (
            int(getattr(waypoint, "road_id", 0)),
            int(getattr(waypoint, "section_id", 0)),
            int(getattr(waypoint, "lane_id", 0)),
            round(float(getattr(waypoint, "s", 0.0)), 2),
            round(float(location.x), 2),
            round(float(location.y), 2),
        )

    def _project_waypoint(self, point, label):
        location = carla.Location(
            x=float(point["x"]), y=float(point["y"]),
            z=float(point.get("z", 0.0)))
        waypoint = self.map.get_waypoint(
            location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if waypoint is None:
            raise RuntimeError("{} cannot be projected to a Driving lane".format(label))
        if _planar_distance(waypoint.transform.location, location) > 3.0:
            raise RuntimeError("{} is more than 3 m from its Driving lane".format(label))
        return waypoint

    def _trace_segment(self, start, target, segment_number):
        """Trace one anchor segment through forward CARLA lane topology."""
        target_location = target.transform.location
        direct = _planar_distance(start.transform.location, target_location)
        budget_m = max(30.0, direct * 3.0 + 20.0)
        frontier = []
        counter = 0
        start_distance = _planar_distance(start.transform.location, target_location)
        heapq.heappush(frontier, (start_distance, counter, 0.0, start, [start]))
        best_cost = {self._waypoint_token(start): 0.0}
        expansions = 0

        while frontier and expansions < 20000:
            _score, _counter, cost, waypoint, path = heapq.heappop(frontier)
            expansions += 1
            distance = _planar_distance(waypoint.transform.location, target_location)
            if distance <= TRACE_GOAL_TOLERANCE_M:
                if self._waypoint_token(path[-1]) != self._waypoint_token(target):
                    path.append(target)
                return path
            if cost >= budget_m:
                continue
            try:
                next_waypoints = waypoint.next(TRACE_STEP_M)
            except (AttributeError, RuntimeError) as exc:
                raise RuntimeError(
                    "cannot trace 2.a route segment {}: {}".format(
                        segment_number, exc)) from exc
            for candidate in next_waypoints or []:
                step = max(0.05, _planar_distance(
                    waypoint.transform.location, candidate.transform.location))
                next_cost = cost + step
                if next_cost > budget_m:
                    continue
                token = self._waypoint_token(candidate)
                if next_cost + 1e-6 >= best_cost.get(token, float("inf")):
                    continue
                best_cost[token] = next_cost
                counter += 1
                heuristic = _planar_distance(
                    candidate.transform.location, target_location)
                heapq.heappush(
                    frontier,
                    (next_cost + heuristic, counter, next_cost, candidate,
                     path + [candidate]))

        raise RuntimeError(
            "2.a route segment {} is not forward-connected within {:.1f} m".format(
                segment_number, budget_m))

    @staticmethod
    def _forward_xy(waypoint):
        transform = waypoint.transform
        try:
            vector = transform.get_forward_vector()
            x, y = float(vector.x), float(vector.y)
        except (AttributeError, TypeError):
            yaw = math.radians(float(transform.rotation.yaw))
            x, y = math.cos(yaw), math.sin(yaw)
        norm = math.hypot(x, y)
        return (x / norm, y / norm) if norm > 1e-9 else (0.0, 0.0)

    @classmethod
    def _has_same_direction_adjacent_driving_lane(cls, waypoint):
        forward = cls._forward_xy(waypoint)
        for accessor in ("get_left_lane", "get_right_lane"):
            try:
                adjacent = getattr(waypoint, accessor)()
            except (AttributeError, RuntimeError):
                adjacent = None
            if adjacent is None:
                continue
            lane_type = getattr(adjacent, "lane_type", carla.LaneType.Driving)
            if lane_type != carla.LaneType.Driving:
                continue
            adjacent_forward = cls._forward_xy(adjacent)
            if (forward[0] * adjacent_forward[0]
                    + forward[1] * adjacent_forward[1]) >= 0.5:
                return True
        return False

    def _prepare_tunnel_geometry(self):
        route = self.config["ego_route"]
        anchors = [
            self._project_waypoint(point, "P{}".format(index))
            for index, point in enumerate(route, 1)
        ]
        dense_waypoints = [anchors[0]]
        anchor_dense_indexes = [0]
        for index, target in enumerate(anchors[1:], 1):
            segment = self._trace_segment(
                dense_waypoints[-1], target, segment_number=index)
            for waypoint in segment[1:]:
                if (self._waypoint_token(waypoint)
                        != self._waypoint_token(dense_waypoints[-1])):
                    dense_waypoints.append(waypoint)
            anchor_dense_indexes.append(len(dense_waypoints) - 1)

        dense = []
        for waypoint in dense_waypoints:
            location = waypoint.transform.location
            dense.append({
                "x": float(location.x),
                "y": float(location.y),
                "z": float(getattr(location, "z", 0.0)),
                "lane_width": float(getattr(waypoint, "lane_width", 3.5)),
                "yaw": float(waypoint.transform.rotation.yaw),
                "road_id": int(getattr(waypoint, "road_id", 0)),
                "section_id": int(getattr(waypoint, "section_id", 0)),
                "lane_id": int(getattr(waypoint, "lane_id", 0)),
            })

        entry_number = int(self.tunnel_cfg["entry_anchor_number"])
        exit_number = int(self.tunnel_cfg["exit_anchor_number"])
        entry_dense = anchor_dense_indexes[entry_number - 1]
        exit_dense = anchor_dense_indexes[exit_number - 1]
        corridor = dense[entry_dense:exit_dense + 1]
        corridor_waypoints = dense_waypoints[entry_dense:exit_dense + 1]
        measured = route_length_2d(corridor)
        if measured + 1e-6 < MIN_TUNNEL_LENGTH_M:
            raise RuntimeError(
                "CARLA topology measures the tunnel route at {:.2f} m; at least "
                "{:.0f} m is required".format(measured, MIN_TUNNEL_LENGTH_M))
        missing_adjacent = [
            index for index, waypoint in enumerate(corridor_waypoints)
            if not self._has_same_direction_adjacent_driving_lane(waypoint)
        ]
        if missing_adjacent:
            location = corridor_waypoints[missing_adjacent[0]].transform.location
            raise RuntimeError(
                "operator-marked tunnel is not a continuous one-way two-lane "
                "road near ({:.2f}, {:.2f})".format(location.x, location.y))

        lane_widths = [max(1.0, item["lane_width"]) for item in corridor]
        gate_half_width = max(4.0, max(lane_widths) * 1.5)
        entry_gate = make_route_gate(route, entry_number, gate_half_width)
        exit_gate = make_route_gate(route, exit_number, gate_half_width)
        if gate_signed_distance(self.config["ego_start"], entry_gate) >= -0.25:
            raise RuntimeError("ego_start must be before the tunnel entry gate")
        if gate_signed_distance(route[-1], exit_gate) <= 0.25:
            raise RuntimeError("the final route point must be after the tunnel exit gate")

        self.tunnel_entry_dense_index = entry_dense
        self.tunnel_dense_route = dense
        self.tunnel_corridor = corridor
        self.tunnel_entry_gate = entry_gate
        self.tunnel_exit_gate = exit_gate
        self.measured_tunnel_length_m = measured

