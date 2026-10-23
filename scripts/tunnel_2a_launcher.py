#!/usr/bin/env python3
"""YAML-driven editor and screening launcher for GB/T 41798 2.a."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.roundabout_launcher import (  # noqa: E402
    ConfigError as CarlaConfigError,
    configure_carla_pythonapi,
    print_connection,
    wait_for_carla,
)
from tunnel_2a import (  # noqa: E402
    ROUTE_PROFILE,
    validate_start_distances,
    validate_tunnel_screening_set,
)


DEFAULT_CONFIG = REPO_ROOT / "config" / "tunnel_2a.yaml"


class ConfigError(ValueError):
    """Raised when the unified tunnel screening configuration is incomplete."""


def _mapping(value, path):
    if not isinstance(value, dict):
        raise ConfigError("{} 必须是YAML对象".format(path))
    return value


def _text(value, path, allow_empty=False):
    if not isinstance(value, str):
        raise ConfigError("{} 必须是字符串".format(path))
    value = value.strip()
    if not value and not allow_empty:
        raise ConfigError("{} 不能为空".format(path))
    return value


def _integer(value, path, minimum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError("{} 必须是整数".format(path))
    if minimum is not None and value < minimum:
        raise ConfigError("{} 必须不小于{}".format(path, minimum))
    return value


def _positive_number(value, path):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError("{} 必须是正数".format(path))
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ConfigError("{} 必须大于0".format(path))
    return value


def _bool(value, path):
    if not isinstance(value, bool):
        raise ConfigError("{} 必须是true或false".format(path))
    return value


def _repo_path(value, path, allow_empty=False):
    text = _text(value, path, allow_empty=allow_empty)
    if not text:
        return None
    candidate = Path(text)
    if not candidate.is_absolute():
        candidate = REPO_ROOT / candidate
    return candidate.resolve()


def load_config(path=DEFAULT_CONFIG):
    config_path = Path(path).resolve()
    try:
        raw_bytes = config_path.read_bytes()
    except OSError as exc:
        raise ConfigError("无法读取配置{}：{}".format(config_path, exc)) from exc
    try:
        root = yaml.safe_load(raw_bytes) or {}
    except yaml.YAMLError as exc:
        raise ConfigError("YAML解析失败：{}".format(exc)) from exc
    root = _mapping(root, "配置根节点")
    if root.get("schema_version") != 1:
        raise ConfigError("schema_version必须为1")

    environment = _mapping(root.get("environment"), "environment")
    connection = _mapping(root.get("connection"), "connection")
    editor = _mapping(root.get("editor"), "editor")
    runner = _mapping(root.get("runner"), "runner")
    ads = _mapping(root.get("ads"), "ads")
    behavior = _mapping(ads.get("behavior"), "ads.behavior")
    tcp = _mapping(ads.get("tcp"), "ads.tcp")

    route_profile = _text(
        editor.get("route_profile"), "editor.route_profile")
    if route_profile != ROUTE_PROFILE:
        raise ConfigError(
            "editor.route_profile必须为{}".format(ROUTE_PROFILE))
    if editor.get("weather_profiles") != "all":
        raise ConfigError("editor.weather_profiles必须为all（15×8=120个条件）")
    entry = _integer(
        editor.get("entry_anchor_number"),
        "editor.entry_anchor_number", minimum=2)
    exit_ = _integer(
        editor.get("exit_anchor_number"),
        "editor.exit_anchor_number", minimum=3)
    if entry >= exit_:
        raise ConfigError("入口P编号必须小于出口P编号")

    collision_value = runner.get("collision_config")
    collision_path = None if collision_value is None else _repo_path(
        collision_value, "runner.collision_config", allow_empty=True)
    config = {
        "config_path": config_path,
        "config_sha256": hashlib.sha256(raw_bytes).hexdigest(),
        "conda_env": _text(
            environment.get("conda_env", ""),
            "environment.conda_env", allow_empty=True),
        "carla_root": _repo_path(
            environment.get("carla_root", ""),
            "environment.carla_root", allow_empty=True),
        "host": _text(connection.get("host"), "connection.host"),
        "port": _integer(connection.get("port"), "connection.port", minimum=1),
        "wait_timeout_s": _integer(
            connection.get("wait_timeout_s"),
            "connection.wait_timeout_s", minimum=0),
        "expected_map": _text(
            connection.get("expected_map", ""),
            "connection.expected_map", allow_empty=True),
        "save_dir": _repo_path(editor.get("save_dir"), "editor.save_dir"),
        "route_profile": route_profile,
        "entry_anchor_number": entry,
        "exit_anchor_number": exit_,
        "scenario_timeout_s": _positive_number(
            editor.get("scenario_timeout_s"),
            "editor.scenario_timeout_s"),
        "input_dir": _repo_path(
            runner.get("input_dir"), "runner.input_dir"),
        "route_id": _text(runner.get("route_id"), "runner.route_id"),
        "headless": _bool(runner.get("headless"), "runner.headless"),
        "resume": _bool(runner.get("resume"), "runner.resume"),
        "output_root": _repo_path(
            runner.get("output_root"), "runner.output_root"),
        "collision_config": collision_path,
        "behavior_description": _text(
            behavior.get("description"), "ads.behavior.description"),
        "tcp_model_path": _repo_path(
            tcp.get("model_path", ""), "ads.tcp.model_path", allow_empty=True),
        "tcp_require_cuda": _bool(
            tcp.get("require_cuda"), "ads.tcp.require_cuda"),
        "tcp_camera_timeout_s": _positive_number(
            tcp.get("camera_frame_timeout_s", 2.0), "ads.tcp.camera_frame_timeout_s"),
    }
    if config["port"] > 65535:
        raise ConfigError("connection.port必须不大于65535")
    expansion = _mapping(root.get("expansion", {}), "expansion")
    config["start_distances_m"] = validate_start_distances(
        expansion.get("start_distances_m", list(range(20, 101, 10))))
    config["route_spacing_m"] = _positive_number(
        expansion.get("route_spacing_m", 5.0), "expansion.route_spacing_m")
    if not 1.0 <= config["route_spacing_m"] <= 10.0:
        raise ConfigError("expansion.route_spacing_m必须在1至10米之间")
    config["expansion_mode"] = expansion.get("mode", "entry_distances")
    if config["expansion_mode"] not in ("entry_distances", "spawn_offsets"):
        raise ConfigError("expansion.mode必须为entry_distances或spawn_offsets")
    config["spawn_offsets_m"] = expansion.get("spawn_offsets_m", [-20, -10, 0, 10, 20])
    config["fallback_offsets_m"] = expansion.get("fallback_offsets_m", [-30, -40])
    from tunnel_spawn_offsets import validate_offsets
    validate_offsets(config["spawn_offsets_m"], config["fallback_offsets_m"])
    config["expected_baseline_count"] = _integer(
        expansion.get("expected_baseline_count", 2), "expansion.expected_baseline_count", minimum=1)
    config["opendrive_path"] = _repo_path(expansion.get("opendrive_path", ""),
                                         "expansion.opendrive_path", allow_empty=True)
    config["route_output_dir"] = _repo_path(
        expansion.get("route_output_dir", "./save_scenarios/2a/route_variants"), "expansion.route_output_dir")
    return config


def editor_command(config, map_name):
    return [
        sys.executable,
        str(REPO_ROOT / "waypoints_stf.py"),
        "--host", config["host"],
        "--port", str(config["port"]),
        "--name", map_name,
        "--scenario", "2a",
        "--save_dir", str(config["save_dir"]),
        "--route_profile", config["route_profile"],
        "--tunnel_entry_anchor_number", str(config["entry_anchor_number"]),
        "--tunnel_exit_anchor_number", str(config["exit_anchor_number"]),
        "--tunnel_scenario_timeout_s", str(config["scenario_timeout_s"]),
    ]


def run_command(config, map_name, model, batch_dir=None):
    video_dir = (batch_dir or config["output_root"]) / model
    command = [
        sys.executable,
        str(REPO_ROOT / "run_stf.py"),
        "--host", config["host"],
        "--port", str(config["port"]),
        "--input_dir", str(config["input_dir"]),
        "--town", map_name,
        "--route_id", config["route_id"],
        "--scenario", "2a",
        "--model", model,
        "--video_dir", str(video_dir),
        "--screening",
    ]
    if config["headless"] or model == "behavior":
        command.append("--headless")
    if config["resume"]:
        command.append("--resume")
    if config["collision_config"] is not None:
        command.extend([
            "--collision_config", str(config["collision_config"]),
        ])
    if model == "tcp":
        command.extend(["--model_path", str(config["tcp_model_path"])])
        command.extend(["--tcp_camera_timeout", str(config["tcp_camera_timeout_s"])])
    return command


def _scenario_from_document(document, map_name, route_id, source):
    if not isinstance(document, dict) or len(document) != 1:
        raise ConfigError("{}必须且只能包含一个地图键".format(source))
    document_map = next(iter(document))
    if document_map.lower() != map_name.lower():
        raise ConfigError(
            "{}地图键为{!r}，当前CARLA地图为{!r}".format(
                source, document_map, map_name))
    try:
        configs = document[document_map][route_id]
        scenario = configs[0]
    except (KeyError, IndexError, TypeError) as exc:
        raise ConfigError(
            "{}缺少{} / {} / [0]场景结构".format(
                source, document_map, route_id)) from exc
    if not isinstance(configs, list) or len(configs) != 1 \
            or not isinstance(scenario, dict):
        raise ConfigError("{}每个route必须恰好包含一个场景".format(source))
    return scenario


def validate_screening_definitions(config, map_info):
    paths = sorted(config["input_dir"].glob("scenario_2a_*.json"))
    if not paths:
        raise ConfigError(
            "没有找到scenario_2a_*.json；请先运行editor并按S保存路线")
    actual_map = map_info["map_name"]
    try:
        opendrive = map_info["world"].get_map().to_opendrive()
    except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
        raise ConfigError("无法读取当前地图OpenDRIVE：{}".format(exc)) from exc
    digest = hashlib.sha256(opendrive.encode("utf-8")).hexdigest()
    scenarios = []
    for path in paths:
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ConfigError("无法读取{}：{}".format(path, exc)) from exc
        scenario = _scenario_from_document(
            document, actual_map, config["route_id"], path)
        tunnel = scenario.get("tunnel_test", {})
        if tunnel.get("map_name", "").lower() != actual_map.lower():
            raise ConfigError("{}的tunnel_test.map_name与当前地图不一致".format(path))
        if tunnel.get("opendrive_sha256") != digest:
            raise ConfigError("{}的OpenDRIVE哈希与当前地图不一致".format(path))
        if tunnel.get("scenario_timeout_s") != config["scenario_timeout_s"]:
            raise ConfigError("{}的超时与YAML不一致，请重新保存或扩展".format(path))
        expansion = tunnel.get("expansion")
        anchors = expansion if isinstance(expansion, dict) else tunnel
        prefix = "source_" if isinstance(expansion, dict) else ""
        if anchors.get(prefix + "entry_anchor_number") != config["entry_anchor_number"] \
                or anchors.get(prefix + "exit_anchor_number") != config["exit_anchor_number"]:
            raise ConfigError("{}的入口/出口P编号与YAML不一致".format(path))
        scenarios.append(scenario)
    try:
        summary = validate_tunnel_screening_set(scenarios)
    except ValueError as exc:
        raise ConfigError(str(exc)) from exc
    if "start_distances_m" in summary:
        if summary["start_distances_m"] != config["start_distances_m"]:
            raise ConfigError("场景起点距离集合与YAML不一致，请重新expand")
        if any(item["tunnel_test"]["expansion"].get("route_spacing_m")
               != config["route_spacing_m"] for item in scenarios):
            raise ConfigError("场景路线采样间距与YAML不一致，请重新expand")
    if summary.get("expansion_kind") == "spawn_offsets":
        if summary["baseline_count"] != config["expected_baseline_count"]:
            raise ConfigError("基准路线数量与YAML不一致")
        for scenario in scenarios:
            expansion = scenario["tunnel_test"]["expansion"]
            if (expansion["requested_offsets_m"] != config["spawn_offsets_m"]
                    or expansion["fallback_offsets_m"] != config["fallback_offsets_m"]
                    or expansion["route_spacing_m"] != config["route_spacing_m"]):
                raise ConfigError("场景偏移设置与YAML不一致，请重新expand")
    return paths, summary


def _validate_models(config, models):
    if "tcp" not in models:
        return
    checkpoint = config["tcp_model_path"]
    if checkpoint is None or not checkpoint.is_file():
        raise ConfigError("TCP checkpoint不存在：{}".format(checkpoint))
    if config["tcp_require_cuda"]:
        try:
            import torch
        except ImportError as exc:
            raise ConfigError("YAML要求CUDA，但无法导入torch") from exc
        if not torch.cuda.is_available():
            raise ConfigError("YAML要求CUDA，但torch.cuda.is_available()为False")


def _write_manifest(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8")
    os.replace(temporary, path)


def launch_editor(config, child_env):
    info = wait_for_carla(config)
    print_connection(info)
    config["save_dir"].mkdir(parents=True, exist_ok=True)
    return subprocess.call(
        editor_command(config, info["map_name"]),
        cwd=REPO_ROOT, env=child_env)


def launch_expansion(config, dry_run=False):
    if config.get("expansion_mode") == "spawn_offsets":
        return launch_spawn_expansion(config, dry_run)
    from tunnel_expansion import (
        expand_baseline, load_baseline, read_approach_geometry, write_expansion,
    )

    if config["save_dir"] == config["input_dir"]:
        raise ConfigError("expand要求editor.save_dir与runner.input_dir使用不同目录")
    if not dry_run and config["input_dir"].exists():
        raise ConfigError("扩展输出目录已存在，请为runner.input_dir选择新目录")
    info = wait_for_carla(config)
    print_connection(info)
    carla_map = info["world"].get_map()
    digest = hashlib.sha256(carla_map.to_opendrive().encode("utf-8")).hexdigest()
    baseline, sources = load_baseline(
        config["save_dir"], info["map_name"], digest, config["route_id"])
    section = baseline["tunnel_test"]
    if (section["entry_anchor_number"] != config["entry_anchor_number"]
            or section["exit_anchor_number"] != config["exit_anchor_number"]):
        raise ConfigError("基准路线入口/出口P编号与YAML不一致")
    if section["scenario_timeout_s"] != config["scenario_timeout_s"]:
        raise ConfigError("基准路线超时与YAML不一致，请重新保存基准路线")
    approach = read_approach_geometry(carla_map, baseline)
    configs, summary = expand_baseline(
        baseline, approach, config["start_distances_m"], config["route_spacing_m"])
    print("基准接近段 {:.2f} m；{}个起点 × {}个天气 = {}个场景".format(
        summary["available_approach_m"], summary["start_distance_count"],
        summary["weather_count"], summary["condition_count"]))
    print("起点距入口（米）：{}".format(summary["start_distances_m"]))
    if dry_run:
        print("dry-run检查通过；未写入场景文件，未生成车辆。")
        return 0
    manifest = {
        "schema_version": 1,
        "screening_only": True,
        "physical_field_test_compliance_claimed": False,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "config_sha256": config["config_sha256"],
        "summary": summary,
        "source_files": [{"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                         for path in sources],
    }
    output = write_expansion(configs, config["input_dir"], config["route_id"], manifest)
    print("2.a扩展已生成：{}".format(output))
    return 0


def prepare_spawn_routes(config):
    from tunnel_expansion import load_baselines
    from tunnel_spawn_offsets import build_spawn_route_variants
    if config["save_dir"] == config["input_dir"]:
        raise ConfigError("基准目录和扩展目录必须分开")
    if config["route_output_dir"] in (config["save_dir"], config["input_dir"]):
        raise ConfigError("基准、route输出、天气输出三个目录必须分开")
    if config.get("opendrive_path"):
        import carla
        text = config["opendrive_path"].read_text(encoding="utf-8")
        paths = sorted(config["save_dir"].glob("scenario_2a_*.json"))
        if not paths:
            raise ConfigError("请先绘制并保存顺向基准路线")
        document = json.loads(paths[0].read_text(encoding="utf-8"))
        if len(document) != 1:
            raise ConfigError("基准文件必须只有一个地图键")
        map_name = next(iter(document))
        if config["expected_map"] and config["expected_map"].rsplit("/", 1)[-1].lower() != map_name.lower():
            raise ConfigError("基准地图与expected_map不一致")
        carla_map = carla.Map(map_name, text)
        print("使用本地OpenDRIVE进行几何扩展（不生成车辆）：{}".format(config["opendrive_path"]))
    else:
        info = wait_for_carla(config)
        print_connection(info)
        carla_map, map_name = info["world"].get_map(), info["map_name"]
    digest = hashlib.sha256(carla_map.to_opendrive().encode("utf-8")).hexdigest()
    baselines, sources = load_baselines(config["save_dir"], map_name, digest, config["route_id"])
    if len(baselines) != config["expected_baseline_count"]:
        raise ConfigError("需要{}条顺向基准路线，当前{}条".format(config["expected_baseline_count"], len(baselines)))
    for baseline in baselines:
        section = baseline["tunnel_test"]
        if (section["entry_anchor_number"] != config["entry_anchor_number"]
                or section["exit_anchor_number"] != config["exit_anchor_number"]
                or section["scenario_timeout_s"] != config["scenario_timeout_s"]):
            raise ConfigError("基准路线锚点/超时与YAML不一致")
    variants, summary = build_spawn_route_variants(
        carla_map, baselines, config["spawn_offsets_m"], config["fallback_offsets_m"],
        "require_forward", config["route_spacing_m"])
    print("{}条基准路线，{}条spawn路线（尚未叠加天气）".format(
        summary["baseline_count"], summary["spawn_count"]))
    for report in summary["baseline_reports"]:
        print("路线 {}：{}".format(report["baseline_route_fingerprint"][:12], report["choices"]))
    manifest = dict(schema_version=1, screening_only=True,
                    physical_field_test_compliance_claimed=False,
                    created_at_utc=datetime.now(timezone.utc).isoformat(),
                    config_sha256=config["config_sha256"],
                    baseline_count=summary["baseline_count"],
                    baseline_reports=summary["baseline_reports"],
                    source_files=[dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                                  for path in sources])
    return variants, manifest


def launch_route_expansion(config, dry_run=False):
    from tunnel_route_artifacts import write_route_variants
    if config["expansion_mode"] != "spawn_offsets":
        raise ConfigError("分阶段扩展需要expansion.mode: spawn_offsets")
    if not dry_run and config["route_output_dir"].exists():
        raise ConfigError("route输出目录已存在，请选择新的route_output_dir")
    variants, manifest = prepare_spawn_routes(config)
    if dry_run:
        print("route预检通过；未写文件。")
        return 0
    output = write_route_variants(variants, config["route_output_dir"], config["route_id"], manifest)
    print("第一阶段route已保存：{}".format(output))
    return 0


def launch_weather_expansion(config, dry_run=False):
    from tunnel_route_artifacts import load_route_variants
    from tunnel_expansion import apply_weather_to_variants, write_expansion
    if config["route_output_dir"] == config["input_dir"]:
        raise ConfigError("route输出和天气输出必须分开")
    if not dry_run and config["input_dir"].exists():
        raise ConfigError("天气输出目录已存在，请选择新的runner.input_dir")
    variants, route_summary = load_route_variants(config["route_output_dir"], config["route_id"])
    if route_summary["baseline_count"] != config["expected_baseline_count"]:
        raise ConfigError("route基准数量与配置不一致，请重新扩展route")
    for variant in variants:
        section = variant["tunnel_test"]
        expansion = section["expansion"]
        if (expansion["requested_offsets_m"] != config["spawn_offsets_m"]
                or expansion["fallback_offsets_m"] != config["fallback_offsets_m"]
                or expansion["route_spacing_m"] != config["route_spacing_m"]
                or expansion["source_entry_anchor_number"] != config["entry_anchor_number"]
                or expansion["source_exit_anchor_number"] != config["exit_anchor_number"]
                or section["scenario_timeout_s"] != config["scenario_timeout_s"]):
            raise ConfigError("route参数与配置不一致，请重新扩展route")
    configs, summary = apply_weather_to_variants(variants)
    print("{}条route × {}种天气 = {}个场景".format(
        len(variants), summary["weather_count"], summary["condition_count"]))
    if dry_run:
        print("天气预检通过；未写文件。")
        return 0
    source_manifest = config["route_output_dir"] / "route_manifest.json"
    manifest = dict(schema_version=1, screening_only=True,
                    physical_field_test_compliance_claimed=False,
                    created_at_utc=datetime.now(timezone.utc).isoformat(),
                    config_sha256=config["config_sha256"], summary=summary,
                    route_manifest=str(source_manifest),
                    route_manifest_sha256=hashlib.sha256(source_manifest.read_bytes()).hexdigest())
    output = write_expansion(configs, config["input_dir"], config["route_id"], manifest)
    print("第二阶段天气场景已保存：{}".format(output))
    return 0


def launch_spawn_expansion(config, dry_run=False):
    if dry_run:
        from tunnel_expansion import apply_weather_to_variants
        variants, _ = prepare_spawn_routes(config)
        _, summary = apply_weather_to_variants(variants)
        print("全流程预检通过：{}个场景；未写文件。".format(summary["condition_count"]))
        return 0
    if config["input_dir"].exists():
        raise ConfigError("天气输出目录已存在，请选择新目录")
    launch_route_expansion(config)
    return launch_weather_expansion(config)


def launch_screening(config, child_env, models):
    _validate_models(config, models)
    info = wait_for_carla(config)
    print_connection(info)
    paths, summary = validate_screening_definitions(config, info)
    stamp = datetime.now(timezone.utc).strftime("screening_%Y%m%dT%H%M%S.%fZ")
    batch_dir = config["output_root"] / stamp
    commands = [run_command(config, info["map_name"], model, batch_dir)
                for model in models]
    manifest = {
        "schema_version": 1,
        "scenario_code": "2a",
        "screening_only": True,
        "physical_field_test_compliance_claimed": False,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "config_path": str(config["config_path"]),
        "config_sha256": config["config_sha256"],
        "map_name": info["map_name"],
        "models": list(models),
        "definition_count": len(paths),
        "screening_summary": summary,
        "commands": commands,
        "status": "running",
    }
    manifest_path = batch_dir / "run_manifest.json"
    _write_manifest(manifest_path, manifest)
    for model, command in zip(models, commands):
        print("\n[2.a] 启动{}：{}个场景条件".format(model, len(paths)))
        try:
            subprocess.run(command, cwd=REPO_ROOT, env=child_env, check=True)
        except subprocess.CalledProcessError as exc:
            manifest.update({
                "status": "failed",
                "failed_model": model,
                "returncode": exc.returncode,
                "finished_at_utc": datetime.now(timezone.utc).isoformat(),
            })
            _write_manifest(manifest_path, manifest)
            raise
    manifest.update({
        "status": "completed",
        "finished_at_utc": datetime.now(timezone.utc).isoformat(),
    })
    _write_manifest(manifest_path, manifest)
    print("\n2.a全部天气单次筛选完成：{}".format(batch_dir))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode", choices=("editor", "expand", "expand-routes", "expand-weather", "run", "run-behavior", "run-tcp"))
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--dry-run", action="store_true", help="Validate expansion without writing files")
    args = parser.parse_args(argv)
    if args.dry_run and args.mode not in ("expand", "expand-routes", "expand-weather"):
        parser.error("--dry-run仅用于扩展模式")
    try:
        config = load_config(args.config)
        if (args.mode == "expand-weather" or
                (args.mode in ("expand", "expand-routes") and config.get("opendrive_path")
                 and config["expansion_mode"] == "spawn_offsets")):
            child_env = os.environ.copy()
        else:
            _, child_env = configure_carla_pythonapi(config)
        if args.mode == "editor":
            return launch_editor(config, child_env)
        if args.mode == "expand":
            return launch_expansion(config, args.dry_run)
        if args.mode == "expand-routes":
            return launch_route_expansion(config, args.dry_run)
        if args.mode == "expand-weather":
            return launch_weather_expansion(config, args.dry_run)
        models = {
            "run": ("behavior", "tcp"),
            "run-behavior": ("behavior",),
            "run-tcp": ("tcp",),
        }[args.mode]
        return launch_screening(config, child_env, models)
    except (ConfigError, CarlaConfigError, ValueError, RuntimeError, OSError) as exc:
        print("配置/启动错误：{}".format(exc), file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as exc:
        print("2.a运行失败，退出码{}".format(exc.returncode), file=sys.stderr)
        return exc.returncode or 1
    except KeyboardInterrupt:
        print("用户中止。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
