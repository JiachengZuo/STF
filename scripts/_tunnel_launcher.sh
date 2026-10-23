#!/usr/bin/env bash
# Shared environment selection for the 2.a entrypoints.
set -euo pipefail
TUNNEL_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TUNNEL_CONFIG_PATH="${TUNNEL_SCRIPT_DIR}/../config/tunnel_2a.yaml"

run_tunnel_launcher() {
    local tunnel_conda_env
    tunnel_conda_env="$(awk '$1 == "conda_env:" {print $2; exit}' "$TUNNEL_CONFIG_PATH" | tr -d "\"'")"
    if [[ -n "$tunnel_conda_env" ]]; then
        command -v conda >/dev/null 2>&1 || { echo "找不到conda，请激活Python 3.10环境。" >&2; exit 2; }
        exec conda run --no-capture-output -n "$tunnel_conda_env" \
            python -u "$TUNNEL_SCRIPT_DIR/tunnel_2a_launcher.py" "$@" --config "$TUNNEL_CONFIG_PATH"
    fi
    if ! command -v python >/dev/null 2>&1 || [[ "$(python -c 'import sys; print(sys.version_info[0])')" != 3 ]]; then
        echo "请先激活兼容的Python 3.10环境，例如 conda activate carla0916。" >&2
        exit 2
    fi
    exec python -u "$TUNNEL_SCRIPT_DIR/tunnel_2a_launcher.py" "$@" --config "$TUNNEL_CONFIG_PATH"
}
