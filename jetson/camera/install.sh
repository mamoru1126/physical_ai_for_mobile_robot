#!/bin/bash
# USB カメラのサービスを入れる. コンテナのイメージを作り、電源を入れるたびに自動で起動するようにする.
#   bash jetson/camera/install.sh
# 何度実行してもよい（イメージを作り直し、サービスを起動し直す. 設定ファイルは上書きしない）.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
NAME="usb-camera"
IMAGE="usb-camera:latest"
CONF_DIR="/etc/usb-camera"
UNIT="/etc/systemd/system/${NAME}.service"

DOCKER_BIN="$(command -v docker || true)"
[ -n "${DOCKER_BIN}" ] || { echo "NG: Docker がありません（jetson/README.md の「Docker」）" >&2; exit 1; }
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
# プロキシの下では、イメージを作るコンテナ（apt-get）にもプロキシの設定を渡す.
# プロキシがこの PC の上（127.0.0.1 など）にあっても届くよう、作るときもホストのネットワークを使う
BUILD_ARGS=(--network host)
for var in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do
  [ -n "${!var:-}" ] && BUILD_ARGS+=(--build-arg "${var}=${!var}")
done
sudo systemctl enable docker > /dev/null 2>&1 || true

echo "1. コンテナのイメージを作ります（初回はネットから取ってくるので数分かかります）"
"${DOCKER[@]}" build "${BUILD_ARGS[@]}" -t "${IMAGE}" "${HERE}"

echo "2. 設定ファイルを置きます: ${CONF_DIR}/camera.env"
sudo mkdir -p "${CONF_DIR}"
if [ -f "${CONF_DIR}/camera.env" ]; then
  echo "   すでにあるので、そのまま使います"
else
  sudo cp "${HERE}/camera.env" "${CONF_DIR}/camera.env"
fi
port="$(sudo sed -n 's/^CAMERA_WEB_PORT=//p' "${CONF_DIR}/camera.env" | tail -n 1 || true)"
port="${port:-9000}"

echo "3. 自動で起動するサービスを入れます: ${UNIT}"
# /dev をまるごと見せ、映像の機器（主番号 81）と USB（189）の読み書きを許す. カメラを挿し直しても見える
sudo tee "${UNIT}" > /dev/null <<UNIT_EOF
[Unit]
Description=USB camera to ROS 2 topic, viewable on port ${port}
After=docker.service network-online.target
Wants=network-online.target
Requires=docker.service

[Service]
Restart=always
RestartSec=5
ExecStartPre=-${DOCKER_BIN} rm -f ${NAME}
ExecStart=${DOCKER_BIN} run --rm --name ${NAME} --network host --ipc host \\
  --env-file ${CONF_DIR}/camera.env \\
  -v /dev:/dev "--device-cgroup-rule=c 81:* rmw" "--device-cgroup-rule=c 189:* rmw" \\
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
ip="$(hostname -I 2> /dev/null | awk '{print $1}')"
echo "ブラウザで開く: http://${ip:-<このPCのIP>}:${port}/"
echo "ログを見る: journalctl -u ${NAME} -f"
