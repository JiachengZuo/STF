import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

from tests.test_tunnel_geometry_carla import carla, OPENDRIVE
from tests.test_tunnel_2a import valid_tunnel_config
from tunnel_2a import tunnel_route_fingerprint, tunnel_condition_fingerprint, validate_tunnel_screening_set
from tunnel_expansion import write_expansion
from tunnel_spawn_offsets import select_spawn_offsets, expand_spawn_baselines


class SpawnOffsetSelectionTest(unittest.TestCase):
    def test_five_preferred_positions_and_positive_fallbacks(self):
        offsets, fallbacks = [-20, -10, 0, 10, 20], [-30, -40]
        cases = ((40, 85, [-20, -10, 0, 10, 20]),
                 (18, 65, [-20, -10, 0, 10, -30]),
                 (9, 55, [-20, -10, 0, -30, -40]))
        for baseline_distance, available, expected in cases:
            with self.subTest(distance=baseline_distance):
                choices = select_spawn_offsets(offsets, fallbacks, baseline_distance, available)
                self.assertEqual([item["actual_offset_m"] for item in choices], expected)
        with self.assertRaisesRegex(ValueError, "no valid fallback"):
            select_spawn_offsets(offsets, fallbacks, 9, 40)


@unittest.skipUnless(carla is not None and hasattr(carla, "Map"), "CARLA Python API unavailable")
class SpawnOffsetGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import tunnel_geometry
        cls.carla_map = carla.Map("FakeTunnelMap", OPENDRIVE)
        cls.sources = []
        for lane_id, start_s in ((-1, 112), (-2, 121)):
            def point(s, clearance=0.2):
                tf = cls.carla_map.get_waypoint_xodr(1, lane_id, s).transform
                return dict(x=tf.location.x, y=tf.location.y, z=tf.location.z + clearance,
                            yaw=tf.rotation.yaw)
            config = valid_tunnel_config()
            config["ego_start"] = point(start_s, 0.3)
            config["ego_route"] = [point(s) for s in (start_s + 3, 130, 155, 180, 205, 230, 255, 280, 300)]
            config["tunnel_test"]["exit_anchor_number"] = 8
            config["tunnel_test"]["opendrive_sha256"] = hashlib.sha256(cls.carla_map.to_opendrive().encode()).hexdigest()
            config["tunnel_test"]["route_fingerprint"] = tunnel_route_fingerprint(config)
            config["tunnel_test"]["condition_fingerprint"] = tunnel_condition_fingerprint(config)
            cls.sources.append(config)
        with mock.patch.object(tunnel_geometry, "carla", carla):
            cls.configs, cls.summary = expand_spawn_baselines(
                cls.carla_map, cls.sources, [-20, -10, 0, 10, 20], [-30, -40], "require_forward")

    def test_two_routes_generate_1200_conditions_with_exact_zero_spawns(self):
        self.assertEqual(self.summary["condition_count"], 1200)
        self.assertEqual(self.summary["spawn_count"], 10)
        self.assertEqual(len({c["name"] for c in self.configs}), 1200)
        originals = {tunnel_route_fingerprint(c): c for c in self.sources}
        zeros = [c for c in self.configs if c["tunnel_test"]["expansion"]["spawn_offset_m"] == 0]
        self.assertEqual(len(zeros), 240)
        for config in zeros:
            source = originals[config["tunnel_test"]["expansion"]["baseline_route_fingerprint"]]
            self.assertEqual(config["ego_start"], source["ego_start"])
        selections = [sorted(c["actual_offset_m"] for c in r["choices"])
                      for r in self.summary["baseline_reports"]]
        self.assertEqual(selections, [[-30, -20, -10, 0, 10], [-40, -30, -20, -10, 0]])

    def test_matrix_rejects_missing_route_and_missing_spawn(self):
        first = self.configs[0]["tunnel_test"]["expansion"]["baseline_route_fingerprint"]
        with self.assertRaisesRegex(ValueError, "missing a baseline"):
            validate_tunnel_screening_set([c for c in self.configs
                                          if c["tunnel_test"]["expansion"]["baseline_route_fingerprint"] == first])
        with self.assertRaisesRegex(ValueError, "missing an offset"):
            validate_tunnel_screening_set([c for c in self.configs
                                          if c["tunnel_test"]["expansion"]["spawn_offset_m"] != 0])

    def test_every_variant_passes_forward_lane_geometry_and_zero_source_is_unchanged(self):
        import tunnel_geometry
        seen = set()
        with mock.patch.object(tunnel_geometry, "carla", carla):
            for config in self.configs:
                fp = config["tunnel_test"]["route_fingerprint"]
                if fp in seen:
                    continue
                seen.add(fp)
                geometry = tunnel_geometry.TunnelRouteGeometry()
                geometry.map, geometry.config, geometry.tunnel_cfg = self.carla_map, config, config["tunnel_test"]
                geometry._prepare_tunnel_geometry()
        self.assertEqual(len(seen), 10)

    def test_full_batch_write_is_unique_and_records_fallbacks(self):
        from scripts.tunnel_2a_launcher import load_config, validate_screening_definitions
        with tempfile.TemporaryDirectory() as directory:
            output = write_expansion(self.configs, Path(directory) / "expanded", "route_01", {"summary": self.summary})
            self.assertEqual(len(list(output.glob("scenario_2a_*.json"))), 1200)
            manifest = json.loads((output / "expansion_manifest.json").read_text())
            self.assertEqual(manifest["summary"]["baseline_count"], 2)
            config = load_config()
            config["input_dir"] = output
            config.update(expected_baseline_count=2, spawn_offsets_m=[-20, -10, 0, 10, 20],
                          fallback_offsets_m=[-30, -40])
            map_info = {"map_name": "FakeTunnelMap",
                        "world": SimpleNamespace(get_map=lambda: self.carla_map)}
            paths, summary = validate_screening_definitions(config, map_info)
            self.assertEqual(len(paths), 1200)
            self.assertEqual(summary["spawn_count"], 10)
            config["spawn_offsets_m"] = [-20, -10, 0, 10, 30]
            with self.assertRaisesRegex(ValueError, "偏移设置"):
                validate_screening_definitions(config, map_info)

    def test_reverse_route_cannot_be_automatically_repaired(self):
        import tunnel_geometry
        source = copy.deepcopy(self.sources[0])
        source["ego_start"]["yaw"] += 180
        for point in source["ego_route"]:
            point["yaw"] += 180
        with mock.patch.object(tunnel_geometry, "carla", carla), self.assertRaisesRegex(ValueError, "opposes lane direction"):
            expand_spawn_baselines(self.carla_map, [source], [-20, -10, 0, 10, 20], [-30, -40], "require_forward")

    def test_four_baselines_create_eight_weather_free_routes_then_960_scenarios(self):
        import tunnel_geometry
        from tunnel_spawn_offsets import build_spawn_route_variants
        from tunnel_route_artifacts import write_route_variants, load_route_variants
        from scripts.tunnel_2a_launcher import load_config, launch_weather_expansion
        sources = copy.deepcopy(self.sources)
        for source, lane_id, station in zip(self.sources, (-1, -2), (117, 123)):
            source = copy.deepcopy(source)
            tf = self.carla_map.get_waypoint_xodr(1, lane_id, station).transform
            source["ego_start"] = dict(x=tf.location.x, y=tf.location.y, z=tf.location.z + 0.3,
                                       yaw=tf.rotation.yaw)
            first = self.carla_map.get_waypoint_xodr(1, lane_id, station + 3).transform
            source["ego_route"][0] = dict(x=first.location.x, y=first.location.y,
                                           z=first.location.z + 0.2, yaw=first.rotation.yaw)
            source["tunnel_test"]["route_fingerprint"] = tunnel_route_fingerprint(source)
            source["tunnel_test"]["condition_fingerprint"] = tunnel_condition_fingerprint(source)
            sources.append(source)
        with mock.patch.object(tunnel_geometry, "carla", carla):
            variants, summary = build_spawn_route_variants(
                self.carla_map, sources, [0, -10], [], "require_forward")
        self.assertEqual(len(variants), 8)
        self.assertEqual(summary["baseline_count"], 4)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_route_variants(variants, root / "routes", "route_01", {})
            for path in (root / "routes").glob("route_*offset*.json"):
                document = json.loads(path.read_text())
                template = document["route_template"]
                self.assertNotIn("weather", template)
                self.assertNotIn("weather_id", template["tunnel_test"])
                self.assertNotIn("weather_catalog_version", template["tunnel_test"]["expansion"])
            loaded, _ = load_route_variants(root / "routes", "route_01")
            self.assertEqual(len(loaded), 8)
            config = load_config()
            config.update(route_output_dir=root / "routes", input_dir=root / "weather")
            config.update(expected_baseline_count=4, spawn_offsets_m=[0, -10], fallback_offsets_m=[])
            with mock.patch("scripts.tunnel_2a_launcher.wait_for_carla") as connect:
                self.assertEqual(launch_weather_expansion(config), 0)
                connect.assert_not_called()
            files = list((root / "weather").glob("scenario_2a_*.json"))
            self.assertEqual(len(files), 960)
            documents = [json.loads(path.read_text())["FakeTunnelMap"]["route_01"][0] for path in files]
            validated = validate_tunnel_screening_set(documents)
            self.assertEqual(validated["spawn_count"], 8)
            self.assertEqual({c["tunnel_test"]["expansion"]["spawn_offset_m"] for c in documents}, {0, -10})
            changed = next((root / "routes").glob("route_*offset*.json"))
            changed.write_text(changed.read_text() + " ")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                load_route_variants(root / "routes", "route_01")


if __name__ == "__main__":
    unittest.main()
