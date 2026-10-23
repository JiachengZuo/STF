import importlib.util
from pathlib import Path
import sys
import types
import unittest

from tests.test_tunnel_2a import valid_tunnel_config
from tunnel_2a import make_route_gate


class _Location:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5


class _Transform:
    def __init__(self, location=None):
        self.location = location or _Location()


class _ParentScene:
    def __init__(self, *args, **kwargs):
        pass

    def spawn(self):
        pass

    def tick(self):
        return getattr(self, "_parent_running", True)

    def destroy(self):
        pass


def _load_runtime_module():
    carla = types.ModuleType("carla")
    carla.Location = _Location
    carla.Transform = _Transform
    carla.VehicleControl = lambda **kwargs: types.SimpleNamespace(**kwargs)
    carla.LaneType = types.SimpleNamespace(Driving="Driving")
    scene = types.ModuleType("stf_scene")
    scene.EgoRouteFollowScene = _ParentScene
    replacements = {"carla": carla, "stf_scene": scene}
    previous = {name: sys.modules.get(name) for name in replacements}
    sys.modules.update(replacements)
    try:
        path = Path(__file__).resolve().parents[1] / "tunnel_scene_2a.py"
        spec = importlib.util.spec_from_file_location(
            "_tunnel_scene_runtime_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, value in previous.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


RUNTIME = _load_runtime_module()


class _Ego:
    def __init__(self, x=-5.0, y=0.0):
        self.location = _Location(x, y)
        self.velocity = _Location()

    def get_location(self):
        return self.location

    def get_velocity(self):
        return self.velocity

    def get_light_state(self):
        return "Position|LowBeam"


class _World:
    def __init__(self, now=0.0):
        self.now = now

    def get_snapshot(self):
        return types.SimpleNamespace(
            timestamp=types.SimpleNamespace(elapsed_seconds=self.now))


def runtime_scene():
    config = valid_tunnel_config()
    scene = RUNTIME.TunnelScene.__new__(RUNTIME.TunnelScene)
    scene.config = config
    scene.tunnel_cfg = config["tunnel_test"]
    scene.model = "behavior"
    scene.ego = _Ego()
    scene.world = _World()
    scene.tunnel_corridor = [
        {"x": 0.0, "y": 0.0, "lane_width": 3.5},
        {"x": 50.0, "y": 0.0, "lane_width": 3.5},
        {"x": 100.0, "y": 0.0, "lane_width": 3.5},
    ]
    scene.tunnel_entry_gate = make_route_gate(config["ego_route"], 2, 5.0)
    scene.tunnel_exit_gate = make_route_gate(config["ego_route"], 4, 5.0)
    scene.measured_tunnel_length_m = 100.0
    scene.tunnel_phase = "APPROACH"
    scene.tunnel_start_sim_time = 0.0
    scene.tunnel_entry_sim_time = None
    scene.tunnel_exit_sim_time = None
    scene.tunnel_previous_location = {"x": -5.0, "y": 0.0, "z": 0.0}
    scene.tunnel_driven_distance_m = 0.0
    scene.max_tunnel_lateral_error_m = 0.0
    scene.same_lane = True
    scene.timed_out = False
    scene.route_complete = False
    scene.finished = False
    scene._collision_recorded = False
    scene._wrong_gate_order = False
    scene._lane_invasion_events = []
    scene._telemetry = []
    scene.tunnel_map_name = "FakeTunnelMap"
    scene.tunnel_opendrive_sha256 = "abc"
    scene.tunnel_timeout_s = 60.0
    scene._parent_running = True
    return scene


class _Waypoint:
    def __init__(self, x, road_id, lane_id=-1):
        self.transform = types.SimpleNamespace(
            location=_Location(x, 0.0, 0.0),
            rotation=types.SimpleNamespace(yaw=0.0),
            get_forward_vector=lambda: _Location(1.0, 0.0, 0.0),
        )
        self.road_id = road_id
        self.section_id = road_id
        self.lane_id = lane_id
        self.lane_type = "Driving"
        self.s = x
        self.lane_width = 3.5
        self._next = []
        self._adjacent = None

    def next(self, _distance):
        return list(self._next)

    def get_left_lane(self):
        return self._adjacent

    def get_right_lane(self):
        return None


class TunnelRuntimeTest(unittest.TestCase):
    def test_operator_route_can_cross_road_and_section_ids(self):
        points = [
            _Waypoint(-10, 1), _Waypoint(0, 1), _Waypoint(50, 2),
            _Waypoint(100, 3), _Waypoint(115, 4),
        ]
        for current, following in zip(points, points[1:]):
            current._next = [following]
        for point in points:
            point._adjacent = _Waypoint(
                point.transform.location.x, point.road_id, lane_id=-2)
        scene = RUNTIME.TunnelScene.__new__(RUNTIME.TunnelScene)
        scene.config = valid_tunnel_config()
        scene.tunnel_cfg = scene.config["tunnel_test"]
        scene.map = types.SimpleNamespace(
            get_waypoint=lambda location, **_kwargs: min(
                points, key=lambda point: abs(point.transform.location.x - location.x)))

        scene._prepare_tunnel_geometry()

        self.assertGreaterEqual(scene.measured_tunnel_length_m, 100.0)
        self.assertEqual(
            {item["road_id"] for item in scene.tunnel_corridor}, {1, 2, 3})

    def test_runtime_rejects_a_tunnel_route_without_second_same_direction_lane(self):
        points = [
            _Waypoint(-10, 1), _Waypoint(0, 1), _Waypoint(50, 2),
            _Waypoint(100, 3), _Waypoint(115, 4),
        ]
        for current, following in zip(points, points[1:]):
            current._next = [following]
        scene = RUNTIME.TunnelScene.__new__(RUNTIME.TunnelScene)
        scene.config = valid_tunnel_config()
        scene.tunnel_cfg = scene.config["tunnel_test"]
        scene.map = types.SimpleNamespace(
            get_waypoint=lambda location, **_kwargs: min(
                points, key=lambda point: abs(point.transform.location.x - location.x)))
        with self.assertRaisesRegex(RuntimeError, "one-way two-lane"):
            scene._prepare_tunnel_geometry()

    def test_success_requires_ordered_gates_same_lane_and_route_completion(self):
        scene = runtime_scene()
        for now, x in ((1.0, 1.0), (2.0, 50.0), (3.0, 101.0)):
            scene.world.now = now
            scene.ego.location = _Location(x, 0.0)
            scene._sample_runtime(now)
        scene.route_complete = True

        result = scene.get_result()

        self.assertTrue(result["entry_crossed"])
        self.assertTrue(result["exit_crossed"])
        self.assertTrue(result["same_lane"])
        self.assertTrue(result["pass"])

    def test_corridor_departure_fails_same_lane(self):
        scene = runtime_scene()
        scene.ego.location = _Location(1.0, 0.0)
        scene._sample_runtime(1.0)
        scene.ego.location = _Location(50.0, 3.0)
        scene._sample_runtime(2.0)
        scene.route_complete = True
        result = scene.get_result()
        self.assertFalse(result["same_lane"])
        self.assertIn("did_not_remain_in_same_lane", result["failure_reasons"])

    def test_lane_invasion_between_gates_fails(self):
        scene = runtime_scene()
        scene.ego.location = _Location(1.0, 0.0)
        scene._sample_runtime(1.0)
        scene._lane_invasion_events.append({
            "sim_time": 1.5, "frame": 30, "markings": []})
        scene.ego.location = _Location(101.0, 0.0)
        scene._sample_runtime(2.0)
        scene.route_complete = True
        result = scene.get_result()
        self.assertFalse(result["pass"])
        self.assertEqual(result["lane_invasion"], 1)

    def test_exit_before_entry_is_rejected(self):
        scene = runtime_scene()
        scene.tunnel_previous_location = {"x": 99.0, "y": 0.0, "z": 0.0}
        scene.ego.location = _Location(101.0, 0.0)
        scene._sample_runtime(1.0)
        self.assertTrue(scene._wrong_gate_order)
        self.assertIn(
            "tunnel_exit_crossed_before_entry",
            scene.get_result()["failure_reasons"])

    def test_simulation_time_timeout_does_not_use_wall_clock(self):
        scene = runtime_scene()
        scene.tunnel_timeout_s = 1.0
        scene.world.now = 1.1
        scene._sample_runtime = lambda _now: None
        self.assertFalse(scene.tick())
        self.assertTrue(scene.timed_out)
        self.assertEqual(scene.tunnel_phase, "TIMEOUT")

    def test_collision_stops_screening_and_fails(self):
        scene = runtime_scene()
        scene.record_collision()
        scene._sample_runtime = lambda _now: None
        self.assertFalse(scene.tick())
        self.assertIn("collision", scene.get_result()["failure_reasons"])

    def test_expansion_adapter_includes_ego_to_first_anchor_in_available_distance(self):
        from tunnel_expansion import read_approach_geometry
        from tunnel_2a import route_length_2d
        points = [_Waypoint(x, 1 if x < 0 else 2) for x in range(-110, 116)]
        for current, following in zip(points, points[1:]):
            current._next = [following]
        for point in points:
            point._adjacent = _Waypoint(point.transform.location.x, point.road_id, -2)
        baseline = valid_tunnel_config()
        baseline["ego_start"]["x"] = -110
        baseline["ego_route"][0]["x"] = -100
        carla_map = types.SimpleNamespace(get_waypoint=lambda location, **_kwargs: min(
            points, key=lambda point: abs(point.transform.location.x - location.x)))
        approach = read_approach_geometry(carla_map, baseline)
        self.assertAlmostEqual(approach[0]["x"], -110)
        self.assertAlmostEqual(approach[-1]["x"], 0)
        self.assertAlmostEqual(route_length_2d(approach), 110)

    def test_entry_metrics_use_the_first_entry_sample_and_are_null_before_entry(self):
        scene = runtime_scene()
        scene.tunnel_cfg["expansion"] = {"start_distance_m": 20}
        before = scene.get_result()
        self.assertIsNone(before["entry_speed_mps"])
        self.assertIsNone(before["approach_time_s"])
        scene.ego.velocity = _Location(10, 0)
        scene.ego.location = _Location(1, 0)
        scene._sample_runtime(2.0)
        scene.ego.velocity = _Location(15, 0)
        scene.ego.location = _Location(101, 0)
        scene._sample_runtime(9.0)
        result = scene.get_result()
        self.assertEqual(result["start_distance_m"], 20)
        self.assertEqual(result["entry_speed_mps"], 10)
        self.assertEqual(result["approach_time_s"], 2)


if __name__ == "__main__":
    unittest.main()
