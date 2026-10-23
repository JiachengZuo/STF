import copy
import hashlib
import json
from pathlib import Path
import tempfile
import types
import unittest

from scripts.tunnel_2a_launcher import (
    ConfigError,
    editor_command,
    load_config,
    run_command,
    validate_screening_definitions,
)
from tests.test_tunnel_2a import valid_tunnel_config
from tunnel_2a import tunnel_condition_fingerprint, tunnel_route_fingerprint
from stf_weather_catalog import all_weather_profiles


class _Map:
    name = "/Game/Carla/Maps/FakeTunnelMap"

    def to_opendrive(self):
        return "<OpenDRIVE><road id='tunnel'/></OpenDRIVE>"


class TunnelLauncherTest(unittest.TestCase):
    def test_repository_yaml_builds_editor_command(self):
        config = load_config()
        command = editor_command(config, "FakeTunnelMap")
        self.assertIn("--scenario", command)
        self.assertIn("2a", command)
        self.assertEqual(
            command[command.index("--route_profile") + 1],
            "tunnel_pass_through")
        self.assertEqual(
            command[command.index("--tunnel_entry_anchor_number") + 1], "2")
        self.assertEqual(
            command[command.index("--tunnel_exit_anchor_number") + 1], "8")
        self.assertEqual(config["scenario_timeout_s"], 60.0)

    def test_behavior_and_tcp_commands_use_separate_roots(self):
        config = copy.copy(load_config())
        batch = Path("/tmp/tunnel-batch")
        behavior = run_command(config, "FakeTunnelMap", "behavior", batch)
        tcp = run_command(config, "FakeTunnelMap", "tcp", batch)
        behavior_root = behavior[behavior.index("--video_dir") + 1]
        tcp_root = tcp[tcp.index("--video_dir") + 1]
        self.assertTrue(behavior_root.endswith("/behavior"))
        self.assertTrue(tcp_root.endswith("/tcp"))
        self.assertIn("--screening", behavior)
        self.assertIn("--screening", tcp)
        self.assertIn("--headless", behavior)
        self.assertIn("--model_path", tcp)

    def test_validation_accepts_exact_120_weather_definitions(self):
        config = copy.copy(load_config())
        config["entry_anchor_number"] = 2
        config["exit_anchor_number"] = 4
        tunnel_map = _Map()
        digest = hashlib.sha256(
            tunnel_map.to_opendrive().encode("utf-8")).hexdigest()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config["input_dir"] = root
            for profile in all_weather_profiles():
                scenario = valid_tunnel_config(
                    profile["id"], copy.deepcopy(profile["parameters"]))
                scenario["tunnel_test"]["map_name"] = "FakeTunnelMap"
                scenario["tunnel_test"]["opendrive_sha256"] = digest
                scenario["tunnel_test"]["route_fingerprint"] = \
                    tunnel_route_fingerprint(scenario)
                scenario["tunnel_test"]["condition_fingerprint"] = \
                    tunnel_condition_fingerprint(scenario)
                document = {"FakeTunnelMap": {"route_01": [scenario]}}
                (root / (scenario["name"] + ".json")).write_text(
                    json.dumps(document), encoding="utf-8")
            paths, summary = validate_screening_definitions(config, {
                "map_name": "FakeTunnelMap",
                "world": types.SimpleNamespace(get_map=lambda: tunnel_map),
            })
            self.assertEqual(len(paths), 120)
            self.assertEqual(summary["weather_count"], 120)

    def test_validation_rejects_current_map_mismatch(self):
        config = copy.copy(load_config())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config["input_dir"] = root
            (root / "scenario_2a_bad.json").write_text(
                json.dumps({"OtherMap": {"route_01": [valid_tunnel_config()]}}),
                encoding="utf-8")
            with self.assertRaisesRegex(ConfigError, "当前CARLA地图"):
                validate_screening_definitions(config, {
                    "map_name": "FakeTunnelMap",
                    "world": types.SimpleNamespace(get_map=lambda: _Map()),
                })


if __name__ == "__main__":
    unittest.main()
