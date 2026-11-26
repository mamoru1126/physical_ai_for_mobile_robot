#!/bin/bash
# Orin のベンチを止める.
#   bash jetson/bench/bench_down.sh          ダミーの世界・実行系・安全ゲートを止める（VLM は動かしたまま）
#   bash jetson/bench/bench_down.sh --all    VLM のサーバも止める
set -uo pipefail
cd "$(dirname "$0")/../.."
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
for name in elevator-bench elevator-bench-gate; do
  if "${DOCKER[@]}" ps -a --format '{{.Names}}' | grep -qx "${name}"; then
    "${DOCKER[@]}" rm -f "${name}" > /dev/null && echo "止めました: ${name}"
  fi
done
if [ "${1:-}" = "--all" ]; then
  bash jetson/vlm_down.sh
fi
