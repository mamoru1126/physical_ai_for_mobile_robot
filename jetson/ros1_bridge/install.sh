#!/bin/bash
# ROS 1 との橋渡しのサービスを入れる. コンテナのイメージを作り、電源を入れるたびに自動で起動するようにする.
#   bash jetson/ros1_bridge/install.sh
# 何度実行してもよい（イメージを作り直し、サービスを起動し直す. 設定ファイルは上書きしない）.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
NAME="ros1-bridge"
IMAGE="ros1-bridge:latest"
CONF_DIR="/etc/ros1-bridge"
UNIT="/etc/systemd/system/${NAME}.service"

DOCKER_BIN="$(command -v docker || true)"
[ -n "${DOCKER_BIN}" ] || { echo "NG: Docker がありません（jetson/README.md の「Docker」）" >&2; exit 1; }
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
sudo systemctl enable docker > /dev/null 2>&1 || true

echo "1. コンテナのイメージを作ります（初回は ros1_bridge を作るので数十分かかります. ネットが要ります）"
"${DOCKER[@]}" build -t "${IMAGE}" "${HERE}"

echo "2. 設定ファイルを置きます: ${CONF_DIR}/bridge.env"
sudo mkdir -p "${CONF_DIR}"
if [ -f "${CONF_DIR}/bridge.env" ]; then
  echo "   すでにあるので、そのまま使います"
else
  sudo cp "${HERE}/bridge.env" "${CONF_DIR}/bridge.env"
fi
master="$(sudo sed -n 's/^ROS_MASTER_URI=//p' "${CONF_DIR}/bridge.env" | tail -n 1 || true)"
if [ "${master:-auto}" = "auto" ]; then
  echo "   ROS 1 の roscore: 同じネットワークから自動で探します"
else
  echo "   ROS 1 の roscore: ${master}（違っていたら sudo nano ${CONF_DIR}/bridge.env で直す）"
fi

echo "3. 自動で起動するサービスを入れます: ${UNIT}"
sudo tee "${UNIT}" > /dev/null <<UNIT_EOF
[Unit]
Description=ROS 1 <-> ROS 2 bridge (odom, cloud, cmd_vel)
After=docker.service network-online.target
Wants=network-online.target
Requires=docker.service

[Service]
Restart=always
RestartSec=5
ExecStartPre=-${DOCKER_BIN} rm -f ${NAME}
ExecStart=${DOCKER_BIN} run --rm --name ${NAME} --network host --ipc host \\
  --env-file ${CONF_DIR}/bridge.env \\
  ${IMAGE}
ExecStop=${DOCKER_BIN} stop -t 5 ${NAME}

[Install]
WantedBy=multi-user.target
UNIT_EOF
sudo systemctl daemon-reload
sudo systemctl enable "${NAME}" > /dev/null
sudo systemctl restart "${NAME}"

sleep 5
echo
if systemctl is-active --quiet "${NAME}"; then
  echo "OK: サービスが動いています（電源を入れるたびに自動で起動します）"
else
  echo "NG: サービスが動いていません. ログ: journalctl -u ${NAME} -n 50"
fi
echo "roscore が見つかるまで待ちます（ログに「roscore を探しています」と出ます）"
echo "ログを見る: journalctl -u ${NAME} -f"
