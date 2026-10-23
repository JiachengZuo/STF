# Local scenario runners alongside the upstream runner

Base: `JiachengZuo/STF` main, commit `f349089d64dc2c3fe953f55c4cb31cf125eec007`.

The only change to an existing upstream file is a six-line entry hook at the beginning of `run.py`. Its original body remains byte-for-byte identical. Existing `scene.py`, `waypoints.py`, `model/tcp.py`, `roundabout_2b.py`, and `weather_catalog.py` remain unchanged.

## Choosing a runner

Existing remote commands keep the existing remote implementation:

```bash
python run.py --scenario 2d --input_dir /path/to/legacy-configs --town roadside_1 --model tcp
```

Explicitly select a local specialized runner with any of these equivalent commands:

```bash
python run.py -2d --input_dir /path/to/save_scenarios_2d --model tcp
python run.py --runner local --scenario 2d --input_dir /path/to/save_scenarios_2d --model tcp
python run_stf.py --scenario 2d --input_dir /path/to/save_scenarios_2d --model tcp
```

Local selectors are `1a`, `2a`, `2d`, `3b_1`, `5a`, `5c`, `5d`, `5e`, and `6c`. Both single- and double-dash selectors are accepted, such as `-2d` and `--2d`. Conflicting selectors fail before loading any scenario. No selector means the existing upstream runner. No automatic routing based on file names or directory contents occurs.

The three supplied standalone runners are included byte-for-byte and called through their own `main()` functions:

| Command | Supplied script | Map loaded by the script |
|---|---|---|
| `python run.py --3b_1 ...` | `scripts/run_3b_1.py` | `STF-3-b-I-t1` |
| `python run.py --5a ...` | `scripts/run_5a.py` | `STF-5-a` |
| `python run.py --5c ...` | `scripts/run_5c.py` | `5c_4` |

They also support `python run_stf.py --scenario 5a ...` and `python run.py --runner local --scenario 5a ...`. Bare `python run.py --scenario 5a ...` retains the upstream generic runner. Options such as `--input_dir`, `--model`, `--model_path`, `--video_dir`, `--host`, and `--port` pass through to the supplied scripts. Their fixed scenario/town checks and map-loading behavior are retained. They continue to use the upstream `scene` and TCP modules, as specified by their existing imports.

These scripts resolve their default input, checkpoint, and video paths relative to their own `scripts/` directory. No scenario matrices or weights are bundled. Specify actual paths when those directories are elsewhere, for example:

```bash
python run.py --3b_1 --input_dir /path/to/save_scenarios_3b --model behavior
python run.py --5a --input_dir /path/to/save_scenarios_5a --model tcp --model_path /path/to/best_model.ckpt
python run.py --5c --input_dir /path/to/save_scenarios_5c --model behavior
```

Use `--help` after any selector to inspect that script's options without connecting to CARLA.

The standalone editor entry for local geometry is `waypoints_stf.py`; remote `waypoints.py` remains unchanged. For example:

```bash
python waypoints_stf.py -2d
python waypoints_stf.py -6c
```

The newly added tunnel launcher invokes `run_stf.py` and `waypoints_stf.py`. Existing remote launcher scripts are not modified. The local `2a` flow is retained in `stf_run_2a.py`; it does not replace upstream `--scenario 2a` behavior.

## Dependency isolation

The original six local runners use `model/stf_tcp.py`, `stf_weather_catalog.py`, `stf_vehicle_control.py`, and `stf_scene.py`. These retain the local TCP loading, weather version 2, action decoding, and scene implementation without changing remote shared modules. `waypoints_stf.py` retains the local editor. Runtime fingerprints identify the actual private source files. Renamed imports and fingerprint paths change runtime hashes, so old result directories cannot be resumed as if the code identity were unchanged; use a new output directory.

Configuration files are included. Generated scenario matrices, simulator maps, checkpoint binaries, recordings, and result ledgers are not included. Use explicit `--input_dir`, `--model_path`, and output paths appropriate to the remote machine. The local `2d` default input path still expands under the current user's home. The original six local runners use the already loaded map; the supplied `3b_1`, `5a`, and `5c` scripts explicitly load their own fixed map before running.

## Applying the reviewed patch

From the intended remote repository checkout, first verify its current state and apply the patch check:

```bash
git apply --check /path/to/compatibility.patch
git apply /path/to/compatibility.patch
```

The patch contains additions and the `run.py` entry hook; it does not remove files. If the checkout has advanced or has conflicting local additions, review those conflicts rather than replacing files wholesale. The accompanying manifest records the exact base, file hashes, and checks.

## Verification

The candidate passed the specialized scenario tests and representative upstream roundabout/traffic-light regressions. The existing 960-condition `2d` matrix passed both Behavior and TCP offline validation. Upstream and candidate `run.py --help` output matched. A source audit found unchanged executable logic in 39 migrated Python modules after accounting for import isolation and fingerprint path changes.

After adding the three supplied runners, 17 focused tests cover legacy pass-through, selector conflicts, real argument parsers, fixed-map checks, preserved map-loading calls (with the client mocked), original scene dependencies, and exit/argument restoration. Each supplied runner was also checked through four actual CLI help forms. The legacy help comparison and existing 960-condition `2d` offline validation were repeated. The supplied scripts themselves are byte-for-byte copies; their SHA-256 identities are included in the package.

No live CARLA replay was performed for this integration. The earlier motion findings describe the local runtime; they do not establish the cause of motion observed under the remote legacy runtime. Verify the selected runner from its `[STF local runner]` startup message before comparing results.
