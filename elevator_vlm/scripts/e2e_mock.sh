#!/bin/bash
# 通し試験（モック VLM、画面なし）. VLM サーバなしで、世界・制御・実行系がつながっているかを確かめる
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
docker compose run --rm shell bash scripts/inside/e2e.sh mock
