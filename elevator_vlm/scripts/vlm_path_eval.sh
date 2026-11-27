#!/bin/bash
# 道に沿って走る（follow_path）の机上検証（3090 の PC. VLM サーバが要る. 先に scripts/vlm_up.sh）.
# ATC のコースの実写のフレーム（リポジトリの atc_frames/）と正解（atc_frames/labels.json）で、
# 道の指差し・止まる条件の判定・計画を、実行系と同じ問いかけで測る（ロボットは動かさない）.
#   scripts/vlm_path_eval.sh                  全部
#   scripts/vlm_path_eval.sh --only point     道の指差しだけ（--only check、--only plan も）
#   scripts/vlm_path_eval.sh --path "the corridor"   沿うものの言い回しを変えて試す
# 結果は runs/path_eval_<日時>/（report.md、overlays/ に点を描いた画像）
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
[ -f ../atc_frames/labels.json ] || { echo "NG: ../atc_frames/labels.json がありません" >&2; exit 1; }
docker compose run --rm -e ELEVATOR_SKIP_BUILD=1 -v "$(cd .. && pwd)/atc_frames:/atc_frames:ro" shell \
  python3 scripts/inside/eval_path.py --frames /atc_frames "$@"
