#!/bin/bash
# 通し試験（本物の VLM、画面なし）. 計画の作成から3つの指示の実行までを自動で1回行い、結果を表示する
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
docker compose run --rm shell bash scripts/inside/e2e.sh real
