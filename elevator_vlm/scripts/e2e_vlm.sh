#!/bin/bash
# 通し試験（本物の VLM、画面なし）. 計画の作成から3つの指示の実行までを自動で1回行い、結果と分析を表示する.
#   先に scripts/vlm_up.sh で VLM サーバを起動しておく.
#   環境変数（指定したものだけ渡す）:
#     PLAN_FILE    計画の JSON. 指定すると、VLM に計画を作らせず、この計画で実行する
#                  例: PLAN_FILE=ros2_ws/src/elevator_task/plans/elevator_expected.json bash scripts/e2e_vlm.sh
#     DOOR_HOLD_S / DOOR_SENSOR / BOARD_FLOOR / DEST_FLOOR   エレベーターの設定
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
ARGS=()
for name in PLAN_FILE DOOR_HOLD_S DOOR_SENSOR BOARD_FLOOR DEST_FLOOR; do
  if [ -n "${!name:-}" ]; then
    ARGS+=(-e "${name}=${!name}")
  fi
done
docker compose run --rm "${ARGS[@]}" shell bash scripts/inside/e2e.sh real
