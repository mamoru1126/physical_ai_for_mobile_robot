#!/bin/bash
# 通し試験（モック VLM、画面なし）. VLM サーバなしで、世界・制御・実行系がつながっているかを確かめる.
#   環境変数（指定したものだけ渡す）: PLAN_FILE / DOOR_HOLD_S / DOOR_SENSOR / BOARD_FLOOR / DEST_FLOOR
#   （意味は scripts/e2e_vlm.sh を参照）
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
ARGS=()
for name in PLAN_FILE DOOR_HOLD_S DOOR_SENSOR BOARD_FLOOR DEST_FLOOR; do
  if [ -n "${!name:-}" ]; then
    ARGS+=(-e "${name}=${!name}")
  fi
done
docker compose run --rm "${ARGS[@]}" shell bash scripts/inside/e2e.sh mock
