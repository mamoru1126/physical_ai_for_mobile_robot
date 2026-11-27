#!/bin/bash
# 安全ゲートのサービスを入れる. 電源を入れるたびに自動で起動する.
#   ネットにつながる PC（Orin など）:     bash safety_gate/install.sh          （この PC でイメージを作る）
#   ネットにつながらない PC（ロボットの PC）: bash safety-gate-bundle/install.sh （運んだイメージを入れる）
# 何度実行してもよい（イメージを入れ直し、サービスを起動し直す. 設定ファイルは上書きしない）.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
NAME="safety-gate"
IMAGE="safety-gate:latest"
CONF_DIR="/etc/safety-gate"
UNIT="/etc/systemd/system/${NAME}.service"
TARBALL="${HERE}/safety-gate-image.tar.gz"

DOCKER_BIN="$(command -v docker || true)"
[ -n "${DOCKER_BIN}" ] || { echo "NG: Docker がありません" >&2; exit 1; }
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
# プロキシの下では、イメージを作るコンテナ（apt-get、pip）にもプロキシの設定を渡す.
# プロキシがこの PC の上（127.0.0.1 など）にあっても届くよう、作るときもホストのネットワークを使う
BUILD_ARGS=(--network host)
for var in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do
  [ -n "${!var:-}" ] && BUILD_ARGS+=(--build-arg "${var}=${!var}")
done
sudo systemctl enable docker > /dev/null 2>&1 || true

echo "1. コンテナのイメージを入れます"
if [ -f "${TARBALL}" ]; then
  if [ -f "${TARBALL}.sha256" ]; then
    (cd "${HERE}" && sha256sum -c --quiet "$(basename "${TARBALL}").sha256") \
      || { echo "NG: イメージのファイルが壊れています. 運び直してください" >&2; exit 1; }
  fi
  "${DOCKER[@]}" load -i "${TARBALL}"
elif [ -f "${HERE}/Dockerfile" ]; then
  echo "   この PC で作ります（初回は数分〜十数分. ネットが要ります）"
  "${DOCKER[@]}" build "${BUILD_ARGS[@]}" -t "${IMAGE}" "${HERE}"
elif "${DOCKER[@]}" image inspect "${IMAGE}" > /dev/null 2>&1; then
  echo "   すでに入っているイメージを使います"
else
  echo "NG: イメージがありません（README の「ネットにつながらない PC に入れる」）" >&2
  exit 1
fi

echo "2. 設定ファイルを置きます: ${CONF_DIR}/config.yaml"
conf_src="${HERE}/config.yaml"
[ -f "${conf_src}" ] || conf_src="${HERE}/ros2_ws/src/safety_gate/config/config.yaml"
sudo mkdir -p "${CONF_DIR}"
if [ -f "${CONF_DIR}/config.yaml" ]; then
  echo "   すでにあるので、そのまま使います"
else
  sudo cp "${conf_src}" "${CONF_DIR}/config.yaml"
fi
port="$(sudo sed -n 's/^  port: *\([0-9]*\).*/\1/p' "${CONF_DIR}/config.yaml" | head -n 1 || true)"
port="${port:-9001}"

echo "3. 自動で起動するサービスを入れます: ${UNIT}"
# 設定ファイルのフォルダを、コンテナの /config に読み取り専用で見せる
sudo tee "${UNIT}" > /dev/null <<UNIT_EOF
[Unit]
Description=Safety gate for velocity commands (WebUI on port ${port})
After=docker.service network-online.target
Wants=network-online.target
Requires=docker.service

[Service]
Restart=always
RestartSec=5
ExecStartPre=-${DOCKER_BIN} rm -f ${NAME}
ExecStart=${DOCKER_BIN} run --rm --name ${NAME} --network host --ipc host \\
  -v ${CONF_DIR}:/config:ro \\
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
echo "WebUI: http://${ip:-<このPCのIP>}:${port}/"
echo "設定を変える: sudo nano ${CONF_DIR}/config.yaml のあと sudo systemctl restart ${NAME}"
echo "ログを見る: journalctl -u ${NAME} -f"
