#!/bin/bash
# 手元の PC（ネットにつながっている）で安全ゲートのイメージを作り、ネットにつながらない PC（ロボットの PC など）に運ぶ一式をまとめる.
#   bash safety_gate/build_image.sh
# できるもの: safety_gate/dist/safety-gate-bundle.tar（USB メモリなどで、ロボットの PC に運ぶ）
# 初回は数分〜十数分かかる.
set -euo pipefail
cd "$(dirname "$0")"
HERE="$(pwd)"
IMAGE="safety-gate:latest"
PLATFORM="${PLATFORM:-linux/amd64}"   # ロボットの PC が x86 なら、このまま
OUT="${HERE}/dist"
BUNDLE="safety-gate-bundle"

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
"${DOCKER[@]}" save "${IMAGE}" | gzip > "${OUT}/${BUNDLE}/safety-gate-image.tar.gz"
cp install.sh uninstall.sh README.md "${OUT}/${BUNDLE}/"
cp ros2_ws/src/safety_gate/config/config.yaml "${OUT}/${BUNDLE}/config.yaml"
{
  echo "作った日時: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "リポジトリのコミット: $(git -C "${HERE}" rev-parse --short HEAD 2> /dev/null || echo 不明)"
  echo "CPU の種類: ${PLATFORM}"
  echo "イメージ: ${IMAGE} $("${DOCKER[@]}" image inspect -f '{{.Id}}' "${IMAGE}")"
} > "${OUT}/${BUNDLE}/VERSION.txt"
(cd "${OUT}/${BUNDLE}" && sha256sum safety-gate-image.tar.gz > safety-gate-image.tar.gz.sha256)
tar -C "${OUT}" -cf "${OUT}/${BUNDLE}.tar" "${BUNDLE}"

echo
echo "OK: ${OUT}/${BUNDLE}.tar（$(du -h "${OUT}/${BUNDLE}.tar" | cut -f1)）"
echo "これをロボットの PC に運び、次を実行します:"
echo "  tar -xf ${BUNDLE}.tar"
echo "  bash ${BUNDLE}/install.sh"
