#!/bin/bash
# 手元の PC（x86、ネットにつながっている）で橋渡しのイメージを作り、ロボットの PC に運ぶ一式をまとめる.
#   bash ros1_bridge/build_image.sh
# できるもの: ros1_bridge/dist/ros1-bridge-bundle.tar（USB メモリなどで、ロボットの PC に運ぶ）
# 初回は ros1_bridge を作るので、数十分かかる.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
IMAGE="ros1-bridge:latest"
PLATFORM="${PLATFORM:-linux/amd64}"   # ロボットの PC が x86 なら、このまま
OUT="${HERE}/dist"
BUNDLE="ros1-bridge-bundle"

command -v docker > /dev/null || { echo "NG: Docker がありません" >&2; exit 1; }
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi

# 手元の PC と同じ種類の CPU 向けなら、ふつうに作る. 違うなら --platform を付ける（buildx と qemu が要る）
native="linux/amd64"
case "$(uname -m)" in
  aarch64 | arm64) native="linux/arm64" ;;
esac
platform_args=()
[ "${PLATFORM}" = "${native}" ] || platform_args=(--platform "${PLATFORM}")

echo "1. イメージを作ります（${PLATFORM}）"
"${DOCKER[@]}" build "${platform_args[@]}" -t "${IMAGE}" "${HERE}"

echo "2. ロボットの PC に運ぶ一式をまとめます"
rm -rf "${OUT:?}/${BUNDLE}" "${OUT}/${BUNDLE}.tar"
mkdir -p "${OUT}/${BUNDLE}"
"${DOCKER[@]}" save "${IMAGE}" | gzip > "${OUT}/${BUNDLE}/ros1-bridge-image.tar.gz"
cp install.sh uninstall.sh bridge.env README.md "${OUT}/${BUNDLE}/"
{
  echo "作った日時: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "リポジトリのコミット: $(git -C "${HERE}" rev-parse --short HEAD 2> /dev/null || echo 不明)"
  echo "CPU の種類: ${PLATFORM}"
  echo "イメージ: ${IMAGE} $("${DOCKER[@]}" image inspect -f '{{.Id}}' "${IMAGE}")"
} > "${OUT}/${BUNDLE}/VERSION.txt"
(cd "${OUT}/${BUNDLE}" && sha256sum ros1-bridge-image.tar.gz > ros1-bridge-image.tar.gz.sha256)
tar -C "${OUT}" -cf "${OUT}/${BUNDLE}.tar" "${BUNDLE}"

echo
echo "OK: ${OUT}/${BUNDLE}.tar（$(du -h "${OUT}/${BUNDLE}.tar" | cut -f1)）"
echo "これをロボットの PC に運び、次を実行します:"
echo "  tar -xf ${BUNDLE}.tar"
echo "  bash ${BUNDLE}/install.sh"
