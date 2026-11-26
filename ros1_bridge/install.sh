#!/bin/bash
# ロボットの PC（ROS 1 が動いている PC）に、橋渡しのサービスを入れる. ネットは要らない.
#   bash ros1-bridge-bundle/install.sh   （手元の PC の build_image.sh で作った一式を展開したところで）
# 何度実行してもよい（イメージを入れ直し、サービスを起動し直す. 設定ファイルは上書きしない）.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
NAME="ros1-bridge"
IMAGE="ros1-bridge:latest"
CONF_DIR="/etc/ros1-bridge"
UNIT="/etc/systemd/system/${NAME}.service"
TARBALL="${HERE}/ros1-bridge-image.tar.gz"

DOCKER_BIN="$(command -v docker || true)"
[ -n "${DOCKER_BIN}" ] || { echo "NG: Docker がありません（README の「Docker がないとき」）" >&2; exit 1; }
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
sudo systemctl enable docker > /dev/null 2>&1 || true

echo "1. コンテナのイメージを入れます"
if [ -f "${TARBALL}" ]; then
  if [ -f "${TARBALL}.sha256" ]; then
    (cd "${HERE}" && sha256sum -c --quiet "$(basename "${TARBALL}").sha256") \
      || { echo "NG: イメージのファイルが壊れています. 運び直してください" >&2; exit 1; }
  fi
  "${DOCKER[@]}" load -i "${TARBALL}"
elif [ -f "${HERE}/Dockerfile" ] && [ "${BUILD_HERE:-0}" = "1" ]; then
  echo "   この PC で作ります（ネットが要ります）"
  "${DOCKER[@]}" build -t "${IMAGE}" "${HERE}"
elif "${DOCKER[@]}" image inspect "${IMAGE}" > /dev/null 2>&1; then
  echo "   すでに入っているイメージを使います"
else
  echo "NG: イメージがありません. 手元の PC で bash ros1_bridge/build_image.sh を実行し、" >&2
  echo "    できた ros1-bridge-bundle.tar を運んで、その中の install.sh を実行してください" >&2
  exit 1
fi

echo "2. 設定ファイルを置きます: ${CONF_DIR}/bridge.env"
sudo mkdir -p "${CONF_DIR}"
if [ -f "${CONF_DIR}/bridge.env" ]; then
  echo "   すでにあるので、そのまま使います"
else
  sudo cp "${HERE}/bridge.env" "${CONF_DIR}/bridge.env"
fi
master="$(sudo sed -n 's/^ROS_MASTER_URI=//p' "${CONF_DIR}/bridge.env" | tail -n 1 || true)"
echo "   ROS 1 の roscore: ${master:-http://localhost:11311}"

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
echo "roscore が動くまで待ちます（ログに「roscore を待っています」と出ます）"
echo "ログを見る: journalctl -u ${NAME} -f"
