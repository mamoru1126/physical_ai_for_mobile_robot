#!/bin/bash
# 実行の記録を分析する（VLM の答えを、シミュレータの真値と突き合わせる）.
#   scripts/analyze.sh                    直近の実行の記録
#   scripts/analyze.sh runs/<名前>        指定した記録
set -e
cd "$(dirname "$0")/.."
if command -v python3 > /dev/null 2>&1; then
  python3 scripts/inside/analyze_run.py "$@"
else
  docker compose run --rm -e ELEVATOR_SKIP_BUILD=1 shell python3 scripts/inside/analyze_run.py "$@"
fi
