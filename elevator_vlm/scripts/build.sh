#!/bin/bash
# イメージをビルドする（単体テストと、擬似世界での通し試験もこの中で走る）
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
docker compose build sim
