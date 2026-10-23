import copy
import hashlib
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

from tests.test_tunnel_2a import valid_tunnel_config
from scripts.tunnel_2a_launcher import load_config, launch_expansion, validate_screening_definitions
from tunnel_2a import (
    canonical_hash, tunnel_condition_fingerprint, tunnel_geometry_fingerprint,
    tunnel_route_fingerprint, validate_start_distances, validate_tunnel_screening_set,
)
from tunnel_expansion import expand_baseline, load_baseline, write_expansion
from stf_weather_catalog import all_weather_profiles


def baseline_and_approach():
    baseline = valid_tunnel_config()
    baseline["ego_start"]["x"] = -110.0
    baseline["ego_route"][0]["x"] = -100.0
    section = baseline["tunnel_test"]
    section["route_fingerprint"] = tunnel_route_fingerprint(baseline)
    section["condition_fingerprint"] = tunnel_condition_fingerprint(baseline)
    approach = [{"x": float(x), "y": 0.0, "z": 0.2, "yaw": 0.0}
                for x in range(-110, 1)]
    return baseline, approach


def refresh_condition(config):
    section = config["tunnel_test"]
    section["route_fingerprint"] = tunnel_route_fingerprint(config)
    section["condition_fingerprint"] = tunnel_condition_fingerprint(config)


class TunnelExpansionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline, cls.approach = baseline_and_approach()
        cls.configs, cls.summary = expand_baseline(
            cls.baseline, cls.approach, list(range(20, 101, 10)))

    def test_full_matrix_has_1080_unique_conditions_and_fixed_tunnel(self):
        self.assertEqual(self.summary["condition_count"], 1080)
        self.assertEqual(self.summary["start_distance_count"], 9)
        self.assertEqual(len({c["name"] for c in self.configs}), 1080)
        self.assertEqual(len({c["tunnel_test"]["condition_fingerprint"] for c in self.configs}), 1080)
        geometry = tunnel_geometry_fingerprint(self.baseline)
        for config in self.configs:
            section = config["tunnel_test"]
            self.assertEqual(tunnel_geometry_fingerprint(config), geometry)
            distance = section["expansion"]["start_distance_m"]
            self.assertAlmostEqual(config["ego_start"]["x"], -distance)
            entry = section["entry_anchor_number"]
            self.assertTrue(all(p["x"] > -distance for p in config["ego_route"][:entry - 1]))
            self.assertAlmostEqual(config["ego_start"]["z"], 0.5)

    def test_curved_approach_uses_path_length_not_world_x_offset(self):
        baseline = copy.deepcopy(self.baseline)
        baseline["ego_start"].update(x=-60, y=-50, yaw=90)
        baseline["ego_route"][0].update(x=-60, y=0, yaw=0)
        refresh_condition(baseline)
        approach = [{"x": -60.0, "y": float(y), "z": 0.2, "yaw": 90.0}
                    for y in range(-50, 0)]
        approach.extend({"x": float(x), "y": 0.0, "z": 0.2, "yaw": 0.0}
                        for x in range(-60, 1))
        configs, _ = expand_baseline(baseline, approach, [80])
        start = configs[0]["ego_start"]
        self.assertAlmostEqual(start["x"], -60)
        self.assertAlmostEqual(start["y"], -20)
        self.assertAlmostEqual(start["yaw"], 90)

    def test_short_approach_and_bad_distance_lists_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "Baseline approach"):
            expand_baseline(self.baseline, self.approach, [120])
        for values in ([], [20, 20], [0], [-20], [True], [float("nan")]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_start_distances(values)

    def test_missing_whole_distance_group_is_detected(self):
        with self.assertRaisesRegex(ValueError, "missing a start-distance group"):
            validate_tunnel_screening_set(self.configs[120:])

    def test_missing_and_duplicate_weather_are_detected_within_each_group(self):
        with self.assertRaisesRegex(ValueError, "missing 1 weather"):
            validate_tunnel_screening_set(self.configs[1:])
        with self.assertRaisesRegex(ValueError, "duplicate weather_id"):
            validate_tunnel_screening_set(self.configs + [self.configs[0]])

    def test_mixed_baselines_and_changed_fixed_geometry_are_rejected(self):
        changed = copy.deepcopy(self.configs)
        changed[0]["tunnel_test"]["expansion"]["baseline_route_fingerprint"] = "b" * 64
        refresh_condition(changed[0])
        with self.assertRaisesRegex(ValueError, "share one baseline"):
            validate_tunnel_screening_set(changed)
        changed = copy.deepcopy(self.configs)
        changed[0]["ego_route"][-1]["x"] += 1
        refresh_condition(changed[0])
        with self.assertRaisesRegex(ValueError, "fixed tunnel geometry"):
            validate_tunnel_screening_set(changed)

    def test_write_read_and_launcher_validation_of_full_matrix(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "expanded"
            write_expansion(self.configs, output, "route_01", {"summary": self.summary})
            manifest = json.loads((output / "expansion_manifest.json").read_text())
            self.assertEqual(len(manifest["files"]), 1080)
            item = manifest["files"][0]
            self.assertEqual(hashlib.sha256((output / item["file"]).read_bytes()).hexdigest(), item["sha256"])
            # Current-map hash is mocked; no CARLA server is involved.
            config = load_config()
            config.update(input_dir=output, exit_anchor_number=4)
            fake_world = types.SimpleNamespace(get_map=lambda: types.SimpleNamespace(to_opendrive=lambda: "fake"))
            with mock.patch("scripts.tunnel_2a_launcher.hashlib.sha256") as digest:
                digest.return_value.hexdigest.return_value = "a" * 64
                # Hashlib is shared by modules: use real hash except for map bytes.
                digest.side_effect = lambda data: types.SimpleNamespace(hexdigest=lambda: "a" * 64) if data == b"fake" else real_sha256(data)
                paths, summary = validate_screening_definitions(config, {"map_name": "FakeTunnelMap", "world": fake_world})
            self.assertEqual(len(paths), 1080)
            self.assertEqual(summary["start_distance_count"], 9)
            with self.assertRaisesRegex(ValueError, "already exists"):
                write_expansion(self.configs, output, "route_01", {})
            self.assertEqual(len(list(output.glob("scenario_2a_*.json"))), 1080)

    def test_dry_run_builds_matrix_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            config = load_config()
            config.update(save_dir=Path(directory) / "base", input_dir=Path(directory) / "out", exit_anchor_number=4,
                          expansion_mode="entry_distances")
            info = {"world": types.SimpleNamespace(get_map=lambda: types.SimpleNamespace(to_opendrive=lambda: "fake")),
                    "map_name": "FakeTunnelMap"}
            with mock.patch("scripts.tunnel_2a_launcher.wait_for_carla", return_value=info), \
                    mock.patch("scripts.tunnel_2a_launcher.print_connection"), \
                    mock.patch("tunnel_expansion.load_baseline", return_value=(self.baseline, [])), \
                    mock.patch("tunnel_expansion.read_approach_geometry", return_value=self.approach):
                self.assertEqual(launch_expansion(config, dry_run=True), 0)
            self.assertFalse(config["input_dir"].exists())

    def test_source_weather_can_be_migrated_but_source_routes_cannot_be_mixed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = copy.deepcopy(self.baseline)
            baseline["weather"] = {"cloudiness": 135}
            refresh_condition(baseline)
            (root / "scenario_2a_old.json").write_text(json.dumps({"FakeTunnelMap": {"route_01": [baseline]}}))
            loaded, _ = load_baseline(root, "FakeTunnelMap", "a" * 64, "route_01")
            configs, _ = expand_baseline(loaded, self.approach, [20])
            self.assertTrue(all(c["weather"]["cloudiness"] <= 100 for c in configs))
            baseline["ego_start"]["x"] -= 1
            refresh_condition(baseline)
            (root / "scenario_2a_other.json").write_text(json.dumps({"FakeTunnelMap": {"route_01": [baseline]}}))
            with self.assertRaisesRegex(ValueError, "more than one baseline"):
                load_baseline(root, "FakeTunnelMap", "a" * 64, "route_01")


real_sha256 = hashlib.sha256


class WeatherBoundsTest(unittest.TestCase):
    def test_120_distinct_weather_parameter_sets_are_within_documented_bounds(self):
        profiles = all_weather_profiles()
        self.assertEqual(len({canonical_hash(p["parameters"]) for p in profiles}), 120)
        for profile in profiles:
            parameters = profile["parameters"]
            for key in ("cloudiness", "precipitation", "precipitation_deposits", "wind_intensity", "fog_density", "wetness", "dust_storm"):
                with self.subTest(profile=profile["id"], key=key):
                    self.assertTrue(0 <= parameters[key] <= 100)
            self.assertTrue(-90 <= parameters["sun_altitude_angle"] <= 90)
            self.assertTrue(0 <= parameters["sun_azimuth_angle"] <= 360)
            self.assertGreaterEqual(parameters["fog_distance"], 0)


if __name__ == "__main__":
    unittest.main()
