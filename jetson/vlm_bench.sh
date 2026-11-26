#!/bin/bash
# Orin で動いている VLM のサーバ（jetson/vlm_up.sh で起動したもの）の、応答時間と正解率を測る.
#   bash jetson/vlm_bench.sh          設定の確認と、今の既定の言い回しでの机上評価（数分）
#   bash jetson/vlm_bench.sh --full   上に加えて、言い回しの候補をすべて比べる（モデル1つにつき 10〜20 分の見込み）
#
# 問いかけは 1 回ずつ順に行う（同時には問いかけない）. 出てくる応答時間が、実機で 1 回待つ時間の目安になる.
# 画像は elevator_vlm/samples/ の、Gazebo で撮った正解つきの画像を使う.
# 結果は jetson/log/<日時>_<モデル>/ に残す（プッシュして共有するため）.
set -uo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"
PORT="${VLM_PORT:-8000}"
BASE_URL="http://127.0.0.1:${PORT}/v1"
NAME="elevator-vlm"

full=0
case "${1:-}" in
  "") ;;
  --full) full=1 ;;
  *) echo "使い方: bash jetson/vlm_bench.sh [--full]" >&2; exit 2 ;;
esac

models="$(curl -sf --noproxy '*' --max-time 5 "${BASE_URL}/models" || true)"
if [ -z "${models}" ]; then
  echo "NG: VLM のサーバが動いていません（bash jetson/vlm_up.sh で起動）" >&2
  exit 1
fi
if ! python3 -c 'import PIL, numpy, yaml' 2> /dev/null; then
  echo "NG: 測るための Python の部品が足りません. 次で入れてください:" >&2
  echo "  sudo apt install -y python3-pil python3-numpy python3-yaml" >&2
  exit 1
fi
root="$(printf '%s' "${models}" | python3 -c 'import json, sys; d = json.load(sys.stdin)["data"][0]; print(d.get("root") or d["id"])')"
short="$(basename "${root}" | sed 's/[^A-Za-z0-9._-]/_/g')"
OUT="${REPO}/jetson/log/$(date +%Y%m%d_%H%M%S)_${short}"
mkdir -p "${OUT}"

if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo -n docker)
fi

# ---- 測ったときの状態 --------------------------------------------------------------
power="$(nvpmodel -q 2> /dev/null || sudo -n nvpmodel -q 2> /dev/null || true)"
power_name="$(printf '%s\n' "${power}" | grep -m 1 'Power Mode' | sed 's/.*Power Mode: *//')"
vllm_version="$(curl -sf --noproxy '*' --max-time 5 "http://127.0.0.1:${PORT}/version" 2> /dev/null || true)"
image="$("${DOCKER[@]}" inspect -f '{{.Config.Image}}' "${NAME}" 2> /dev/null | head -n 1 || true)"
run_args="$("${DOCKER[@]}" inspect -f '{{json .Args}}' "${NAME}" 2> /dev/null | head -n 1 || true)"
{
  echo "日時: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "Jetson Linux: $(head -n 1 /etc/nv_tegra_release 2> /dev/null || echo 不明)"
  echo "メモリ: $(awk '/^MemTotal:/ { printf "%.1f GiB", $2 / 1048576 }' /proc/meminfo)"
  echo "電源モード: ${power_name:-不明}"
  echo "モデル: ${root}"
  echo "vLLM: ${vllm_version:-不明}"
  echo "イメージ: ${image:-不明}"
  echo "起動の引数: ${run_args:-不明}"
} > "${OUT}/env.txt"
cat "${OUT}/env.txt"
echo

# 測っている間の、GPU の使用率・周波数・温度・電力（1 秒ごと）
tegra_pid=""
if command -v tegrastats > /dev/null 2>&1; then
  tegrastats --interval 1000 --logfile "${OUT}/tegrastats.log" > /dev/null 2>&1 &
  tegra_pid=$!
fi
stop_tegrastats() {
  if [ -n "${tegra_pid}" ]; then
    kill "${tegra_pid}" 2> /dev/null || true
    wait "${tegra_pid}" 2> /dev/null || true
    tegra_pid=""
  fi
}
trap stop_tegrastats EXIT

export ELEVATOR_VLM_ROOT="${REPO}/elevator_vlm"
export VLM_BASE_URL="${BASE_URL}"
export VLM_MODEL="vlm"
export VLM_EVAL_WORKERS=1

# ---- 1. 設定の確認と、既定の言い回しでの机上評価 ------------------------------------------------
echo "==== 設定の確認と机上評価（check_vlm.py）===="
python3 "${REPO}/elevator_vlm/scripts/inside/check_vlm.py" 2>&1 | tee "${OUT}/check_vlm.txt"
check_status=${PIPESTATUS[0]}

# ---- 2. 言い回しの候補をすべて比べる（--full のとき）-------------------------------------------
if [ "${full}" -eq 1 ]; then
  echo
  echo "==== 言い回しの比較（eval_phrases.py）===="
  python3 "${REPO}/elevator_vlm/scripts/inside/eval_phrases.py" --workers 1 --out "${OUT}" 2>&1 \
    | tee "${OUT}/phrases_console.txt"
fi

stop_tegrastats
"${DOCKER[@]}" logs --tail 300 "${NAME}" > "${OUT}/vlm.log" 2>&1 || true

echo
echo "==== まとめ ===="
grep -h -A2 '^応答時間（1回あたり）' "${OUT}/check_vlm.txt" 2> /dev/null | sed 's/^/  /'
if [ -f "${OUT}/tegrastats.log" ]; then
  # GR3D_FREQ（GPU の使用率）の最大. 0% のままなら GPU を使っていない
  gpu_max="$(grep -o 'GR3D_FREQ [0-9]*%' "${OUT}/tegrastats.log" | tr -dc '0-9\n' | sort -n | tail -n 1)"
  [ -n "${gpu_max}" ] && echo "  GPU の使用率の最大: ${gpu_max}%（tegrastats）"
fi
echo
echo "結果: ${OUT}"
if [ "${check_status}" -ne 0 ]; then
  echo "注意: 設定の確認（[1]〜[3]）に NG があります. check_vlm.txt を見てください."
fi
echo "共有するときは、jetson/log/ をコミットしてプッシュしてください（jetson/vlm.md の「6. 結果を送る」）."
exit "${check_status}"
