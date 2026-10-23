#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_tunnel_launcher.sh"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "用法：./scripts/run_tunnel_scenarios.sh [behavior|tcp|both]"
    echo "运行已生成场景，默认behavior；both依次运行两个控制器。需要CARLA加载同一地图。"
    exit 0
fi
if (($# > 1)); then
    echo "只接受一个控制器选择：behavior、tcp或both。" >&2
    exit 2
fi
case "${1:-behavior}" in
    behavior) run_tunnel_launcher run-behavior ;;
    tcp) run_tunnel_launcher run-tcp ;;
    both) run_tunnel_launcher run ;;
    *) echo "控制器必须为behavior、tcp或both。" >&2; exit 2 ;;
esac
