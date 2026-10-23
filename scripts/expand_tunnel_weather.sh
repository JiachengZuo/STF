#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_tunnel_launcher.sh"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "用法：./scripts/expand_tunnel_weather.sh [--dry-run]"
    echo "第二阶段：读取已保存route，叠加120种天气，生成完整天气场景；不需要CARLA Server。"
    exit 0
fi
if (($# > 1)) || [[ $# == 1 && "$1" != "--dry-run" ]]; then
    echo "只接受--dry-run；参数统一编辑config/tunnel_2a.yaml。" >&2
    exit 2
fi
run_tunnel_launcher expand-weather "$@"
