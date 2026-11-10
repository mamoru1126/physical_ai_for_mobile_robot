#!/bin/bash
# VLM サーバとのつながりを確かめる（判定、指差しの座標の約束、計画の作成、応答時間）
set -e
cd "$(dirname "$0")/.."
docker compose run --rm -e ELEVATOR_SKIP_BUILD=1 shell python3 scripts/inside/check_vlm.py
