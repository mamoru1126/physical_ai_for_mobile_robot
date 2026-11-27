#!/bin/bash
# 道に沿って走る（follow_path）の机上検証を Orin で行う. ロボットは動かさない.
# ATC のコースの実写のフレーム（atc_frames/）と正解（atc_frames/labels.json）で、本物の VLM が
#   1. 道の上の次の点を、走れる範囲の中に指せるか（人やコーンを避けているか）
#   2. 止まる条件（エレベーターの扉が目の前、など）を正しく判定できるか
#   3. 「道に沿って走って」などの指示から、正しい計画を作れるか
# と、応答時間・消費電力を測る.
#   bash jetson/bench/path_eval.sh                  全部
#   bash jetson/bench/path_eval.sh --only point     道の指差しだけ（--only check、--only plan も）
#   bash jetson/bench/path_eval.sh --limit 5        枚数を減らして試す
# VLM のサーバが動いていなければ起動する（jetson/vlm_up.sh. 既定のモデル）.
# 電源モードが MAXN なら始めない（ベンチと同じ. BENCH_ALLOW_MAXN=1 で始める）.
# 結果は jetson/log/path_eval_<日時>_<モデル>/ に残す（report.md、overlays/ に点を描いた画像）.
set -uo pipefail
cd "$(dirname "$0")/../.."
ROOT="$(pwd)"
RUNTIME_IMAGE="elevator-runtime:latest"
VLM_PORT="${VLM_PORT:-8000}"

say() { printf '%s\n' "$*"; }
fail() { printf 'NG: %s\n' "$*" >&2; exit 1; }

command -v docker > /dev/null 2>&1 || fail "Docker がありません（jetson/README.md の「Docker」）"
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
[ -f "${ROOT}/atc_frames/labels.json" ] || fail "正解のファイルがありません: ${ROOT}/atc_frames/labels.json"

# ---- 電源モード（MAXN では始めない）-------------------------------------------------
power="$(nvpmodel -q 2> /dev/null || sudo -n nvpmodel -q 2> /dev/null || true)"
power_name="$(printf '%s\n' "${power}" | grep -m 1 'Power Mode' | sed 's/.*Power Mode: *//')"
if [[ "${power_name}" == MAXN* ]] && [ "${BENCH_ALLOW_MAXN:-0}" != "1" ]; then
  fail "電源モードが ${power_name} です. この電源では電流が足りずに落ちるので始めません（sudo nvpmodel -m 2 で MODE_30W）"
fi

# ---- イメージと VLM ------------------------------------------------------------------
if ! "${DOCKER[@]}" image inspect "${RUNTIME_IMAGE}" > /dev/null 2>&1; then
  say "イメージを作ります: ${RUNTIME_IMAGE}（初回は数分〜十数分）"
  "${DOCKER[@]}" build "${BUILD_ARGS[@]}" -t "${RUNTIME_IMAGE}" -f "${ROOT}/elevator_vlm/docker/Dockerfile.runtime" \
    --label "elevator.src=$(git -C "${ROOT}" rev-parse HEAD:elevator_vlm 2> /dev/null || echo unknown)" "${ROOT}/elevator_vlm" \
    || fail "イメージを作れません"
fi
vlm_ready() { curl -sf --noproxy '*' --max-time 3 "http://127.0.0.1:${VLM_PORT}/v1/models" > /dev/null 2>&1; }
if ! vlm_ready; then
  if "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx elevator-vlm; then
    say "VLM のサーバが起動の途中です. 答えられるようになるまで待ちます"
  else
    say "VLM のサーバを起動します（jetson/vlm_up.sh）"
    bash "${ROOT}/jetson/vlm_up.sh" || fail "VLM のサーバを起動できません"
  fi
  for _ in $(seq 1 480); do
    vlm_ready && break
    sleep 5
  done
  vlm_ready || fail "VLM のサーバが答えません（ログ: ${DOCKER[*]} logs elevator-vlm）"
fi
model="$(curl -sf --noproxy '*' --max-time 5 "http://127.0.0.1:${VLM_PORT}/v1/models" \
  | python3 -c 'import json, sys; d = json.load(sys.stdin)["data"][0]; print(d.get("root") or d["id"])' 2> /dev/null || echo unknown)"
short="$(basename "${model}" | sed 's/[^A-Za-z0-9._-]/_/g')"
stamp="$(date +%Y%m%d_%H%M%S)"
OUT="${ROOT}/jetson/log/path_eval_${stamp}_${short}"
mkdir -p "${OUT}"
{
  echo "日時: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "Jetson Linux: $(head -n 1 /etc/nv_tegra_release 2> /dev/null || echo 不明)"
  echo "電源モード: ${power_name:-不明}"
  echo "モデル: ${model}"
  echo "リポジトリ: $(git -C "${ROOT}" rev-parse --short HEAD 2> /dev/null || echo 不明)"
} > "${OUT}/env.txt"
cat "${OUT}/env.txt"

# 測っている間の、電力・GPU の使用率（1 秒ごと）
tegra_pid=""
if command -v tegrastats > /dev/null 2>&1; then
  tegrastats --interval 1000 --logfile "${OUT}/tegrastats.log" > /dev/null 2>&1 &
  tegra_pid=$!
fi
trap '[ -n "${tegra_pid}" ] && kill "${tegra_pid}" 2> /dev/null' EXIT

# ---- 測る（リポジトリのスクリプトと正解を、そのまま使う）--------------------------------------
"${DOCKER[@]}" run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -e VLM_BASE_URL="http://127.0.0.1:${VLM_PORT}/v1" -e VLM_MODEL=vlm \
  -v "${ROOT}/elevator_vlm:/workspace" -v "${ROOT}/atc_frames:/atc_frames:ro" -v "${OUT}:/out" \
  --entrypoint python3 "${RUNTIME_IMAGE}" /workspace/scripts/inside/eval_path.py \
  --frames /atc_frames --out /out --config /workspace/ros2_ws/src/elevator_sim/config/task_bench_path.yaml "$@" \
  2>&1 | tee "${OUT}/eval.txt"
code=${PIPESTATUS[0]}
if [ -n "${tegra_pid}" ]; then
  kill "${tegra_pid}" 2> /dev/null || true
  wait "${tegra_pid}" 2> /dev/null || true
  tegra_pid=""
fi
# 結果のディレクトリ（path_eval_<日時>）の中身を、1 つ上に移す
inner="$(ls -d "${OUT}"/path_eval_* 2> /dev/null | head -n 1)"
if [ -n "${inner}" ]; then
  mv "${inner}"/* "${OUT}/"
  rmdir "${inner}"
fi
if [ -f "${OUT}/tegrastats.log" ]; then
  python3 - "${OUT}/tegrastats.log" > "${OUT}/power.txt" << 'PY'
import re
import sys

totals, gpu = [], []
for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
    found = re.findall(r"([A-Z][A-Z0-9_]+) (\d+)mW/(\d+)mW", line)
    if found:
        totals.append(sum(int(now) for _n, now, _a in found))
    m = re.search(r"GR3D_FREQ (\d+)%", line)
    if m:
        gpu.append(int(m.group(1)))
if totals:
    print(f"レールの合計（目安）: 最大 {max(totals) / 1000:.1f} W、平均 {sum(totals) / len(totals) / 1000:.1f} W")
if gpu:
    print(f"GPU の使用率: 最大 {max(gpu)}%、平均 {sum(gpu) / len(gpu):.0f}%")
PY
  sed 's/^/  /' "${OUT}/power.txt"
fi
say ""
say "結果: ${OUT}（report.md、overlays/）"
say "共有するときは、jetson/log/ をコミットしてプッシュしてください."
exit "${code}"
