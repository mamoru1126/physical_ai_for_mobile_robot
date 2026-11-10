#!/bin/bash
# シミュレーションと実行系を起動する（本物の VLM サーバを使う）. Web UI は http://localhost:8080/
#   モック VLM で動かすときは  scripts/sim_up.sh mock
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || cp .env.example .env
xhost +local:root > /dev/null 2>&1 || true      # Gazebo の画面を出すため
if [ "${1:-real}" = "mock" ]; then
  docker compose up sim-mock
else
  docker compose up sim
fi
