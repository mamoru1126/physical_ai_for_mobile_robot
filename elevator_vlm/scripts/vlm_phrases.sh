#!/bin/bash
# 条件文と目標の言い回しを、正解つきの画像で比べる（VLM サーバが要る. 先に scripts/vlm_up.sh）.
#   scripts/vlm_phrases.sh                 samples/ の画像で比べる
#   scripts/vlm_phrases.sh --latest-run    直近の実行の記録の画像と、そのときの計画の言い回しも足す
# 候補は samples/phrases.json. 結果は runs/phrases_<日時>.txt にも残る
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
docker compose run --rm -e ELEVATOR_SKIP_BUILD=1 shell python3 scripts/inside/eval_phrases.py "$@"
