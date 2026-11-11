#!/bin/bash
# VLM サーバ（vLLM）を起動し、応答するまで待つ. 初回はモデル（約 11 GB）のダウンロードがある
# 起動に失敗したら、エラーを表示して終わる
set -u
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
docker compose up -d vlm || exit 1

echo "VLM サーバの起動を待っています（初回は 10 分ほど. 経過は  docker compose logs -f vlm）"
START=$(date +%s)
while true; do
  if curl -sf --noproxy '*' -o /dev/null http://127.0.0.1:8000/v1/models; then
    echo
    echo "起動しました（$(( $(date +%s) - START )) 秒）"
    exit 0
  fi
  if [ -z "$(docker compose ps -q --status running vlm)" ]; then
    echo
    echo "VLM サーバが止まりました. ログの要点:"
    docker compose logs --no-color vlm 2>&1 | grep -E "Error|error:|ERROR.*(Value|Runtime|Memory)" | tail -n 8
    echo "全体のログは  docker compose logs vlm > log/vlm_up.log  で保存できます"
    exit 1
  fi
  if [ $(( $(date +%s) - START )) -gt 3600 ]; then
    echo
    echo "1 時間待っても応答がありません"
    exit 1
  fi
  printf "."
  sleep 5
done
