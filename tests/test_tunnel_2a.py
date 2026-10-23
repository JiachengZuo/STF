import copy
import ast
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

from tunnel_2a import (
    CAPABILITY,
    GEOMETRY_SOURCE,
    ROUTE_PROFILE,
    SCHEMA_VERSION,
    gate_crossing,
    make_route_gate,
    project_to_route,
    route_length_2d,
    tunnel_condition_fingerprint,
    tunnel_route_fingerprint,
    validate_tunnel_config,
    validate_tunnel_screening_set,
    write_tunnel_result_artifacts,
)
from stf_weather_catalog import all_weather_profiles


def _load_editor_with_carla_stub():
    carla = types.ModuleType("carla")
    previous = sys.modules.get("carla")
    sys.modules["carla"] = carla
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    try:
        path = Path(__file__).resolve().parents[1] / "waypoints_stf.py"
        spec = importlib.util.spec_from_file_location(
            "_tunnel_editor_serialization_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        if previous is None:
            sys.modules.pop("carla", None)
        else:
            sys.modules["carla"] = previous


EDITOR_MODULE = _load_editor_with_carla_stub()


def valid_tunnel_config(weather_id="sunny_01", weather=None):
    config = {
        "name": "scenario_2a_0000_{}".format(weather_id),
        "ego_start": {"x": "-20.0", "y": "0.0", "z": "0.3", "yaw": "0.0"},
        "ego_route": [
            {"x": -10.0, "y": 0.0, "z": 0.2, "yaw": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.2, "yaw": 0.0},
            {"x": 50.0, "y": 0.0, "z": 0.2, "yaw": 0.0},
            {"x": 100.0, "y": 0.0, "z": 0.2, "yaw": 0.0},
            {"x": 115.0, "y": 0.0, "z": 0.2, "yaw": 0.0},
        ],
        "weather": weather or {"cloudiness": 0.0},
        "other_actors": {"center": []},
        "timeout": 60.0,
        "active": True,
        "tunnel_test": {
            "schema_version": SCHEMA_VERSION,
            "capability": CAPABILITY,
            "route_profile": ROUTE_PROFILE,
            "geometry_source": GEOMETRY_SOURCE,
            "entry_anchor_number": 2,
            "exit_anchor_number": 4,
            "scenario_timeout_s": 60.0,
            "screening_only": True,
            "weather_id": weather_id,
            "map_name": "FakeTunnelMap",
            "opendrive_sha256": "a" * 64,
        },
    }
    config["tunnel_test"]["route_fingerprint"] = tunnel_route_fingerprint(config)
    config["tunnel_test"]["condition_fingerprint"] = \
        tunnel_condition_fingerprint(config)
    return config


class TunnelHelpersTest(unittest.TestCase):
    def test_run_dispatch_registers_tunnel_scene(self):
        source = (Path(__file__).resolve().parents[1] / "stf_run_2a.py").read_text(
            encoding="utf-8")
        tree = ast.parse(source)
        mapping = None
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(
                    isinstance(target, ast.Name) and target.id == "SCENE_CLASSES"
                    for target in node.targets):
                mapping = {
                    key.value: value.id
                    for key, value in zip(node.value.keys, node.value.values)
                    if isinstance(key, ast.Constant) and isinstance(value, ast.Name)
                }
                break
        self.assertIsNotNone(mapping)
        self.assertEqual(mapping.get("2a"), "TunnelScene")

    def test_valid_config_has_one_hundred_metre_operator_segment(self):
        summary = validate_tunnel_config(valid_tunnel_config())
        self.assertEqual(summary["operator_marked_tunnel_length_m"], 100.0)

    def test_anchor_validation_requires_approach_exit_and_order(self):
        for field, value, message in (
                ("entry_anchor_number", 1, "before"),
                ("exit_anchor_number", 5, "after"),
                ("entry_anchor_number", 4, "before exit")):
            config = valid_tunnel_config()
            config["tunnel_test"][field] = value
            with self.assertRaisesRegex(ValueError, message):
                validate_tunnel_config(config, require_fingerprints=False)

    def test_short_tunnel_is_rejected(self):
        config = valid_tunnel_config()
        config["ego_route"][2]["x"] = 20.0
        config["ego_route"][3]["x"] = 40.0
        with self.assertRaisesRegex(ValueError, "at least 100 m"):
            validate_tunnel_config(config, require_fingerprints=False)

    def test_short_tunnel_error_lists_segments_sum_and_direct_distance(self):
        config = valid_tunnel_config()
        config["ego_route"] = [dict(x=-10, y=0, z=0, yaw=0)] + [
            dict(x=i * 3, y=4 if i % 2 else 0, z=i * 100, yaw=0)
            for i in range(7)] + [dict(x=25, y=0, z=0, yaw=0)]
        config["tunnel_test"]["exit_anchor_number"] = 8
        with self.assertRaises(ValueError) as raised:
            validate_tunnel_config(config, require_fingerprints=False)
        message = str(raised.exception)
        for number in range(2, 8):
            self.assertIn("P{} -> P{}: 5.0000 m".format(number, number + 1), message)
        self.assertIn("Sum (P2 -> P8): 30.0000 m", message)
        self.assertIn("Direct (P8 -> P2): 18.0000 m", message)
        self.assertNotIn("P1 -> P2:", message)
        self.assertNotIn("P8 -> P9:", message)

    def test_other_actor_is_rejected(self):
        config = valid_tunnel_config()
        config["other_actors"]["center"].append({"type": "car"})
        with self.assertRaisesRegex(ValueError, "does not allow"):
            validate_tunnel_config(config, require_fingerprints=False)

    def test_route_fingerprint_does_not_change_with_weather(self):
        clear = valid_tunnel_config("sunny_01", {"cloudiness": 0.0})
        fog = valid_tunnel_config("fog_01", {"fog_density": 30.0})
        self.assertEqual(tunnel_route_fingerprint(clear), tunnel_route_fingerprint(fog))
        self.assertNotEqual(
            tunnel_condition_fingerprint(clear), tunnel_condition_fingerprint(fog))

    def test_gate_crossing_is_directional_and_finite(self):
        config = valid_tunnel_config()
        gate = make_route_gate(config["ego_route"], 2, half_width_m=3.5)
        self.assertTrue(gate_crossing(
            {"x": -1.0, "y": 0.0}, {"x": 1.0, "y": 0.0}, gate))
        self.assertFalse(gate_crossing(
            {"x": 1.0, "y": 0.0}, {"x": -1.0, "y": 0.0}, gate))
        self.assertFalse(gate_crossing(
            {"x": -1.0, "y": 20.0}, {"x": 1.0, "y": 20.0}, gate))

    def test_projection_supports_continuous_corridor_progress(self):
        route = [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 20, "y": 5}]
        detail = project_to_route({"x": 5, "y": 1}, route)
        self.assertAlmostEqual(detail["distance_m"], 1.0)
        self.assertAlmostEqual(detail["progress_m"], 5.0)
        self.assertEqual(detail["segment_index"], 0)
        self.assertGreater(route_length_2d(route), 20.0)

    def test_full_weather_screening_set_requires_120_unique_conditions(self):
        configs = []
        for profile in all_weather_profiles():
            configs.append(valid_tunnel_config(
                profile["id"], copy.deepcopy(profile["parameters"])))
        summary = validate_tunnel_screening_set(configs)
        self.assertEqual(summary["condition_count"], 120)
        duplicate = copy.deepcopy(configs)
        duplicate[-1] = copy.deepcopy(duplicate[0])
        with self.assertRaisesRegex(ValueError, "duplicate weather_id"):
            validate_tunnel_screening_set(duplicate)
        changed = copy.deepcopy(configs)
        changed[0]["weather"]["cloudiness"] = 99.0
        changed[0]["tunnel_test"]["condition_fingerprint"] = \
            tunnel_condition_fingerprint(changed[0])
        with self.assertRaisesRegex(ValueError, "shared catalog"):
            validate_tunnel_screening_set(changed)

    def test_tunnel_result_writer_uses_screening_heading(self):
        with tempfile.TemporaryDirectory() as directory:
            artifacts = write_tunnel_result_artifacts([{
                "scenario": "scenario_2a_0000_sunny_01",
                "trial_valid": True,
                "pass": True,
                "route_complete": True,
            }], directory)
            payload = json.loads(Path(artifacts["json"]).read_text(encoding="utf-8"))
            report = Path(artifacts["markdown"]).read_text(encoding="utf-8")
            self.assertTrue(payload["screening_only"])
            self.assertFalse(payload["physical_field_test_compliance_claimed"])
            self.assertIn("2.a 隧道场景筛选结果", report)
            self.assertNotIn("机动车信号灯", report)


class TunnelEditorTest(unittest.TestCase):
    def test_reverse_click_does_not_append_and_reverse_ego_move_does_not_commit(self):
        editor = EDITOR_MODULE.CarlaMapEditor0916.__new__(EDITOR_MODULE.CarlaMapEditor0916)
        editor.is_roundabout_2b = False
        editor.is_tunnel_2a = True
        editor.ego_point = dict(x=0, y=0, z=0.3, yaw=0)
        editor.ego_route_points = []
        editor._feedback = mock.Mock()
        waypoint = types.SimpleNamespace(transform=types.SimpleNamespace(
            location=types.SimpleNamespace(x=-10, y=0, z=0), rotation=types.SimpleNamespace(yaw=0)))
        editor.get_nearest_waypoint = lambda x, y: waypoint
        editor.add_ego_route_point(-10, 0)
        self.assertEqual(editor.ego_route_points, [])
        self.assertTrue(editor._feedback.call_args.args[1])
        waypoint.transform.location.x = 10
        editor.add_ego_route_point(10, 0)
        self.assertEqual(len(editor.ego_route_points), 1)
        self.assertFalse(editor.place_tunnel_ego(dict(x=20, y=0, z=0.3, yaw=0)))
        self.assertEqual(editor.ego_point["x"], 0)

    def test_reverse_saved_definition_is_rejected(self):
        config = valid_tunnel_config()
        config["ego_start"]["yaw"] = 180
        for point in config["ego_route"]:
            point["yaw"] = 180
        with self.assertRaisesRegex(ValueError, "opposes lane direction"):
            validate_tunnel_config(config, require_fingerprints=False)

    def test_tunnel_save_reuses_route_and_writes_all_weather_without_trigger(self):
        with tempfile.TemporaryDirectory() as directory:
            editor = EDITOR_MODULE.CarlaMapEditor0916.__new__(
                EDITOR_MODULE.CarlaMapEditor0916)
            editor.is_tunnel_2a = True
            editor.route_profile = ROUTE_PROFILE
            editor.tunnel_entry_anchor_number = 2
            editor.tunnel_exit_anchor_number = 4
            editor.tunnel_scenario_timeout_s = 60.0
            editor.route_index = 0
            editor.save_dir = directory
            editor.town_name = "FakeTunnelMap"
            editor.map = types.SimpleNamespace(
                name="/Game/Carla/Maps/FakeTunnelMap",
                to_opendrive=lambda: "<OpenDRIVE><road/></OpenDRIVE>")
            editor.ego_point = {
                "x": -20.0, "y": 0.0, "z": 0.3, "yaw": 0.0}
            editor.ego_route_points = copy.deepcopy(
                valid_tunnel_config()["ego_route"])
            editor.actor_points = []
            editor.trigger_point = None
            editor._feedback = mock.Mock()
            editor.clear_all_points = mock.Mock()
            editor._validate_tunnel_editor_topology = mock.Mock()

            generated = editor.save_tunnel_screening_definitions()

            self.assertEqual(len(generated), 120)
            editor._validate_tunnel_editor_topology.assert_called_once()
            self.assertEqual(len(list(Path(directory).glob("scenario_2a_*.json"))), 120)
            first = json.loads(Path(generated[0]).read_text(encoding="utf-8"))
            config = first["FakeTunnelMap"]["route_01"][0]
            self.assertNotIn("trigger_position", config)
            self.assertEqual(config["ego_route"], editor.ego_route_points)
            self.assertEqual(config["tunnel_test"]["entry_anchor_number"], 2)
            self.assertEqual(config["tunnel_test"]["exit_anchor_number"], 4)
            self.assertTrue(config["tunnel_test"]["screening_only"])
            validate_tunnel_config(config)

    def test_tunnel_save_rejects_other_actors_before_writing(self):
        editor = EDITOR_MODULE.CarlaMapEditor0916.__new__(
            EDITOR_MODULE.CarlaMapEditor0916)
        editor.is_tunnel_2a = True
        editor.route_profile = ROUTE_PROFILE
        editor.ego_point = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        editor.actor_points = [{"type": "car"}]
        editor._feedback = mock.Mock()
        self.assertEqual(editor.save_tunnel_screening_definitions(), [])
        self.assertTrue(editor._feedback.call_args.args[1])


if __name__ == "__main__":
    unittest.main()
