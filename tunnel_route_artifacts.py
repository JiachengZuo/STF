"""Weather-free route artifacts connecting the two expansion stages."""

import copy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

from tunnel_2a import tunnel_condition_fingerprint, validate_tunnel_screening_set
from stf_weather_catalog import WEATHER_CATALOG_VERSION, all_weather_profiles


def validate_route_variants(variants):
    # A single temporary weather profile exercises the same route/matrix
    # checks. It is not stored in the route artifacts or expanded here.
    profile = all_weather_profiles()[0]
    checked = copy.deepcopy(variants)
    for config in checked:
        config["weather"] = profile["parameters"]
        section = config["tunnel_test"]
        section["weather_id"] = profile["id"]
        section["expansion"]["weather_catalog_version"] = WEATHER_CATALOG_VERSION
        section["condition_fingerprint"] = tunnel_condition_fingerprint(config)
    return validate_tunnel_screening_set(checked, require_all_weather=False)


def write_route_variants(variants, output_dir, route_id, manifest):
    validate_route_variants(variants)
    output = Path(output_dir)
    if output.exists():
        raise ValueError("Route output already exists; choose a new route_output_dir: {}".format(output))
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".tunnel-routes-", dir=output.parent))
    try:
        files = []
        for variant in variants:
            template = copy.deepcopy(variant)
            template.pop("weather", None)
            section = template["tunnel_test"]
            section.pop("weather_id", None)
            section.pop("condition_fingerprint", None)
            section["expansion"].pop("weather_catalog_version", None)
            name = template["name"].removeprefix("scenario_2a_") + ".json"
            document = dict(schema_version=1, artifact_type="tunnel_route_variant",
                            route_id=route_id, route_template=template)
            payload = (json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode()
            if (staging / name).exists():
                raise ValueError("Duplicate route artifact name: " + name)
            (staging / name).write_bytes(payload)
            files.append(dict(file=name, sha256=hashlib.sha256(payload).hexdigest()))
        payload = dict(manifest, artifact_type="tunnel_route_matrix", weather_expanded=False,
                       route_variant_count=len(variants), files=files)
        (staging / "route_manifest.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        staging.rename(output)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return output


def load_route_variants(output_dir, route_id):
    root = Path(output_dir)
    manifest = json.loads((root / "route_manifest.json").read_text(encoding="utf-8"))
    files = manifest["files"]
    expected = {item["file"] for item in files}
    actual = {path.name for path in root.glob("*.json")} - {"route_manifest.json"}
    if len(expected) != len(files) or actual != expected:
        raise ValueError("Route artifact matrix has missing, duplicate or extra files")
    variants = []
    for item in files:
        name = item["file"]
        if Path(name).name != name:
            raise ValueError("Invalid route artifact filename")
        payload = (root / name).read_bytes()
        if hashlib.sha256(payload).hexdigest() != item["sha256"]:
            raise ValueError("Route artifact hash mismatch: " + name)
        document = json.loads(payload)
        if (document.get("artifact_type") != "tunnel_route_variant"
                or document.get("schema_version") != 1 or document.get("route_id") != route_id):
            raise ValueError("Invalid route artifact schema/route_id: " + name)
        variants.append(document["route_template"])
    if len(variants) != manifest["route_variant_count"]:
        raise ValueError("Route artifact count differs from manifest")
    return variants, validate_route_variants(variants)
