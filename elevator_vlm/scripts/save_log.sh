#!/bin/bash
# 直近の実行の記録を log/ にコピーする（リポジトリにプッシュして共有するため）.
# VLM に渡した画像は数が多いので、通し試験の要所の画像だけを含める.
set -e
cd "$(dirname "$0")/.."
NAME="$(date +%Y%m%d_%H%M%S)"
DEST="log/${NAME}"
mkdir -p "${DEST}"
LATEST_E2E="$(ls -d runs/e2e_* 2>/dev/null | sort | tail -n 1 || true)"
if [ -n "${LATEST_E2E}" ]; then
  cp -r "${LATEST_E2E}" "${DEST}/"
fi
LATEST_RUN="$(ls -d runs/20* 2>/dev/null | sort | tail -n 1 || true)"
if [ -n "${LATEST_RUN}" ]; then
  mkdir -p "${DEST}/$(basename "${LATEST_RUN}")"
  cp "${LATEST_RUN}/events.jsonl" "${DEST}/$(basename "${LATEST_RUN}")/"
fi
docker compose logs --no-color --tail 300 vlm > "${DEST}/vlm.log" 2>&1 || true
echo "コピーしました: ${DEST}"
