#!/bin/bash
# 直近の実行の記録を log/ にコピーする（リポジトリにプッシュして共有するため）.
#   - 直近の通し試験（runs/e2e_*）の結果
#   - 直近の実行（runs/20*）の記録、その分析（analysis.txt）、VLM に渡した画像のうち要所のもの
#   - 直近の言い回しの比較（runs/phrases_*）の結果
#   - VLM サーバのログの末尾
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
if [ -n "${LATEST_RUN}" ] && [ -f "${LATEST_RUN}/events.jsonl" ]; then
  RUN_DEST="${DEST}/$(basename "${LATEST_RUN}")"
  mkdir -p "${RUN_DEST}"
  cp "${LATEST_RUN}/events.jsonl" "${RUN_DEST}/"
  if command -v python3 > /dev/null 2>&1; then
    python3 scripts/inside/analyze_run.py "${LATEST_RUN}" --key-images "${RUN_DEST}" > "${RUN_DEST}/analysis.txt" 2>&1 || true
  elif [ -d "${LATEST_RUN}/images" ]; then
    cp -r "${LATEST_RUN}/images" "${RUN_DEST}/"
  fi
fi
for f in $(ls runs/phrases_*.txt 2>/dev/null | sort | tail -n 1) $(ls runs/phrases_*.json 2>/dev/null | sort | tail -n 1); do
  cp "${f}" "${DEST}/"
done
docker compose logs --no-color --tail 300 vlm > "${DEST}/vlm.log" 2>&1 || true
echo "コピーしました: ${DEST}"
