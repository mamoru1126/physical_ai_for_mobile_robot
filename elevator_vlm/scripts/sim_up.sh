#!/bin/bash
# シミュレーションと実行系を起動する（本物の VLM サーバを使う）. Web UI は http://localhost:8080/
#   モック VLM で動かすときは  scripts/sim_up.sh mock
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
WEB_PORT="$(grep -E '^WEB_PORT=' .env | tail -n 1 | cut -d= -f2)"
# 別のシミュレーションが動いたままだと、ロボットへの指令やオドメトリが混ざって、おかしな動きになる.
# Web UI のポートが使われていたら、始めない
if (exec 3<>"/dev/tcp/127.0.0.1/${WEB_PORT:-8080}") 2> /dev/null; then
  echo "NG: 別のシミュレーションが動いたままです（ポート ${WEB_PORT:-8080} が使われています）."
  echo "    動かしている端末を Ctrl+C で止めるか、bash scripts/down.sh を実行してから、やり直してください."
  exit 3
fi
xhost +local:root > /dev/null 2>&1 || true      # Gazebo の画面を出すため
if [ "${1:-real}" = "mock" ]; then
  docker compose up sim-mock
else
  docker compose up sim
fi
