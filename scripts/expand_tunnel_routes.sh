#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_tunnel_launcher.sh"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "用法：./scripts/expand_tunnel_routes.sh [--dry-run]"
    echo "第一阶段：按配置扩展各基准的spawn，保存无天气route。参数见config/tunnel_2a.yaml。"
    exit 0
fi
if (($# > 1)) || [[ $# == 1 && "$1" != "--dry-run" ]]; then
    echo "只接受--dry-run；参数统一编辑config/tunnel_2a.yaml。" >&2
    exit 2
fi
run_tunnel_launcher expand-routes "$@"
