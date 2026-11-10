#!/bin/bash
# VLM サーバ（vLLM）を起動し、応答するまで待つ. 初回はモデル（約 11 GB）のダウンロードがある
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
docker compose up -d vlm
docker compose run --rm -e ELEVATOR_SKIP_BUILD=1 shell python3 scripts/inside/wait_vlm.py
