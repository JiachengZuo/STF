#!/usr/bin/env python3
"""Single-weather route preflight, optionally driving the reference controller.

Unlike the full weather runner, this explicitly reports incomplete baseline
coverage and never labels a partial batch as a complete route check.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import queue
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.tunnel_2a_launcher import load_config
from tunnel_expansion import load_baselines
from tunnel_spawn_offsets import build_spawn_route_variants
from tunnel_2a import canonical_hash, tunnel_route_fingerprint, tunnel_condition_fingerprint, validate_tunnel_screening_set, write_tunnel_result_artifacts
from stf_weather_catalog import all_weather_profiles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(ROOT / "config/tunnel_2a.yaml"))
    parser.add_argument("--weather", default="sunny_07")
    parser.add_argument("--drive", action="store_true")
    parser.add_argument("--model", choices=("behavior", "tcp"), default="behavior")
    parser.add_argument("--max-routes", type=int, help="Limit a smoke check; requires --allow-incomplete")
    parser.add_argument("--reuse-results", type=Path,
                        help="Reuse completed identical physical conditions from a preceding check's 2a_result.json")
    parser.add_argument("--allow-incomplete", action="store_true",
                        help="Check available routes, but report incomplete coverage and exit 2")
    args = parser.parse_args()
    if args.max_routes is not None and (args.max_routes <= 0 or not args.allow_incomplete):
        parser.error("--max-routes must be positive and requires --allow-incomplete")
    config = load_config(args.config)
    profiles = {item["id"]: item for item in all_weather_profiles()}
    if args.weather not in profiles:
        parser.error("Unknown weather ID")
    import carla
    client = carla.Client(config["host"], config["port"])
    client.set_timeout(15.0)
    world = client.get_world()
    carla_map = world.get_map()
    map_name = carla_map.name.rsplit("/", 1)[-1]
    digest = hashlib.sha256(carla_map.to_opendrive().encode()).hexdigest()
    sources, source_paths = load_baselines(config["save_dir"], map_name, digest, config["route_id"])
    complete = len(sources) == config["expected_baseline_count"]
    if not complete and not args.allow_incomplete:
        parser.error("Expected {} baselines; found {}. Save missing routes or explicitly use --allow-incomplete.".format(
            config["expected_baseline_count"], len(sources)))
    variants, route_summary = build_spawn_route_variants(
        carla_map, sources, config["spawn_offsets_m"], config["fallback_offsets_m"], "require_forward",
        config["route_spacing_m"])
    for variant in variants:
        variant["name"] += "_" + args.weather
        variant["weather"] = copy.deepcopy(profiles[args.weather]["parameters"])
        variant["tunnel_test"]["weather_id"] = args.weather
        variant["tunnel_test"]["condition_fingerprint"] = tunnel_condition_fingerprint(variant)
    validate_tunnel_screening_set(variants, require_all_weather=False)
    if args.max_routes is not None and args.max_routes < len(variants):
        variants = variants[:args.max_routes]
        complete = False
    if args.drive and list(world.get_actors().filter("vehicle.*")):
        raise RuntimeError("Existing vehicles in world; route check will not alter or delete them")
    output = ROOT / "runs/tunnel_2a_route_checks" / datetime.now(timezone.utc).strftime("check_%Y%m%dT%H%M%SZ")
    definitions = output / "selected_definitions"
    definitions.mkdir(parents=True)
    paths = []
    for variant in variants:
        path = definitions / (variant["name"] + ".json")
        path.write_text(json.dumps({map_name: {config["route_id"]: [variant]}}, ensure_ascii=False, indent=2))
        paths.append(path)
    metadata = dict(expected_baselines=config["expected_baseline_count"], actual_baselines=len(sources),
                    expected_routes=config["expected_baseline_count"] * len(config["spawn_offsets_m"]),
                    actual_routes=len(variants), coverage_complete=complete, weather_id=args.weather,
                    controller=args.model if args.drive else None, driving_test=args.drive,
                    server_version=client.get_server_version(), client_version=client.get_client_version(),
                    map_name=map_name, opendrive_sha256=digest,
                    source_files=[dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in source_paths])
    metadata["runtime_adapter_sha256"] = hashlib.sha256((ROOT / "tunnel_scene_2a.py").read_bytes()).hexdigest()
    metadata["checkpoint_sha256"] = (hashlib.sha256(config["tcp_model_path"].read_bytes()).hexdigest()
                                      if args.drive and args.model == "tcp" else None)
    reused = {}

    def physical_key(item):
        return canonical_hash(dict(route=tunnel_route_fingerprint(item), weather=item["weather"],
                                   timeout=item["tunnel_test"]["scenario_timeout_s"],
                                   actors=item.get("other_actors"), controller=args.model))

    if args.reuse_results:
        prior = json.loads(args.reuse_results.read_text())
        if not args.drive or prior["metadata"].get("opendrive_sha256") != digest:
            raise ValueError("Reuse requires drive mode and the same OpenDRIVE")
        for key in ("runtime_adapter_sha256", "checkpoint_sha256", "controller"):
            if prior["metadata"].get(key) != metadata[key]:
                raise ValueError("Reuse rejected: changed " + key)
        for record in prior["records"]:
            if record.get("trial_valid") is not True or record.get("controller") != args.model:
                continue
            old_path = Path(record["input_file"])
            old = json.loads(old_path.read_text())[map_name][config["route_id"]][0]
            if old["tunnel_test"]["condition_fingerprint"] != record.get("condition_fingerprint"):
                raise ValueError("Prior input identity differs from its recorded result")
            reused[physical_key(old)] = record
    records = []
    original_settings, original_weather = world.get_settings(), world.get_weather()
    manifest = dict(metadata, status="running", geometry_summary=route_summary)

    def persist():
        (output / "check_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        write_tunnel_result_artifacts(records, str(output), metadata=metadata)

    persist()
    print("CHECK_OUTPUT=" + str(output), flush=True)
    try:
        if args.drive:
            from tunnel_scene_2a import TunnelScene
            import imageio.v2 as imageio
            import numpy as np
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
            world.set_weather(carla.WeatherParameters(**profiles[args.weather]["parameters"]))
        for index, (variant, path) in enumerate(zip(variants, paths), 1):
            section = variant["tunnel_test"]
            if physical_key(variant) in reused:
                record = copy.deepcopy(reused[physical_key(variant)])
                record.update(reused=True, source_result_file=str(args.reuse_results),
                              source_input_file=record["input_file"],
                              source_condition_fingerprint=record["condition_fingerprint"],
                              input_file=str(path), scenario=variant["name"],
                              condition_fingerprint=section["condition_fingerprint"],
                              reuse_basis="identical route/ego/weather/timeout/actors/controller")
                records.append(record)
                persist()
                print("REUSED {}/{} {} pass={}".format(index, len(variants), variant["name"], record["pass"]), flush=True)
                continue
            record = dict(scenario=variant["name"], geometry_valid=True, weather_id=args.weather,
                          spawn_offset_m=section["expansion"]["spawn_offset_m"],
                          baseline_route_fingerprint=section["expansion"]["baseline_route_fingerprint"],
                          input_file=str(path), driving_test=args.drive)
            if not args.drive:
                records.append(record)
                persist()
                continue
            scene, sensors, writer = None, [], None
            print("START {}/{} {}".format(index, len(variants), variant["name"]), flush=True)
            try:
                scene = TunnelScene(client, world, str(path), map_name, config["route_id"], args.model,
                                    str(config["tcp_model_path"]) if args.model == "tcp" else None)
                scene.tcp_frame_timeout_s = config["tcp_camera_timeout_s"]
                scene.spawn()
                collision = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                              carla.Transform(), attach_to=scene.ego)
                sensors.append(collision)
                collision.listen(scene.record_collision)
                camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
                camera_bp.set_attribute("image_size_x", "640")
                camera_bp.set_attribute("image_size_y", "368")
                camera_bp.set_attribute("sensor_tick", "0.05")
                camera = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=-6, z=3),
                                           carla.Rotation(pitch=-12)), attach_to=scene.ego)
                sensors.append(camera)
                images = queue.Queue(maxsize=4)

                def on_image(image):
                    try:
                        images.put_nowait(image)
                    except queue.Full:
                        pass

                camera.listen(on_image)
                video_path = output / (variant["name"] + ".mp4")
                writer = imageio.get_writer(str(video_path), fps=20, codec="libx264")
                deadline = time.monotonic() + 180
                steps = 0
                while True:
                    if time.monotonic() > deadline:
                        raise RuntimeError("wall-clock watchdog reached 180 s")
                    running = scene.tick()
                    world.tick()
                    try:
                        image = images.get(timeout=0.5)
                        rgb = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
                        writer.append_data(rgb[:, :, :3][:, :, ::-1])
                    except queue.Empty:
                        pass
                    steps += 1
                    if steps % 200 == 0:
                        print("PROGRESS {} t={:.1f}s phase={}".format(
                            index, steps * 0.05, scene.tunnel_phase), flush=True)
                    if not running:
                        break
                time.sleep(0.05)  # Drain asynchronous collision/lane callbacks.
                record.update(scene.get_result())
                record["video_path"] = str(video_path)
            except Exception as exc:
                record.update(trial_valid=False, pass_=None, failure_reasons=[str(exc)])
                record["pass"] = record.pop("pass_")
            finally:
                for sensor in sensors:
                    try:
                        sensor.stop()
                        sensor.destroy()
                    except RuntimeError:
                        pass
                if scene is not None:
                    scene.destroy()
                if writer is not None:
                    writer.close()
            records.append(record)
            persist()
            print("RESULT {} pass={} failures={}".format(index, record.get("pass"), record.get("failure_reasons")), flush=True)
        manifest["status"] = "completed" if complete else "incomplete_coverage"
    finally:
        if args.drive:
            world.set_weather(original_weather)
            world.apply_settings(original_settings)
        manifest["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        persist()
    print("FINISHED=" + str(output), flush=True)
    return 0 if complete and all(r.get("pass", True) is True for r in records) else 2


if __name__ == "__main__":
    raise SystemExit(main())
