#!/bin/bash
# Orin のベンチで、エレベーターのタスクを自動で1回通す（人が画面で押すのと同じ操作を、WebUI の API で行う）.
#   bash jetson/bench/bench_e2e.sh
# ベンチが動いていなければ、先に bash jetson/bench/bench_up.sh を実行する（このスクリプトからも起動する）.
#
# 計画の作成 → 承認して実行 → エレベーターを呼ぶ → 終わるまで見守り、真値で成否を判定する.
# 通している間の消費電力（tegrastats）も記録する.
# 結果は jetson/log/bench_<日時>_<モデル>/ に残す（プッシュして共有するため）.
#
# 環境変数:
#   BENCH_PLAN_FILE   VLM に計画を作らせず、この計画（JSON）を読み込んで実行する
#                     （例 elevator_vlm/ros2_ws/src/elevator_task/plans/elevator_expected.json）
#   BENCH_TIMEOUT_S   タスクの時間の上限 [秒]（既定 900）
set -uo pipefail
cd "$(dirname "$0")/../.."
ROOT="$(pwd)"
DOMAIN="${BENCH_DOMAIN_ID:-43}"

if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi

if ! "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx elevator-bench; then
  echo "ベンチが動いていないので、起動します"
  bash "${ROOT}/jetson/bench/bench_up.sh" || exit 1
fi
for name in elevator-bench elevator-bench-gate; do
  "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx "${name}" || { echo "NG: ${name} が動いていません" >&2; exit 1; }
done

stamp="$(date +%Y%m%d_%H%M%S)"
model="$(curl -sf --noproxy '*' --max-time 5 "http://127.0.0.1:${VLM_PORT:-8000}/v1/models" \
  | python3 -c 'import json, sys; d = json.load(sys.stdin)["data"][0]; print(d.get("root") or d["id"])' 2> /dev/null || echo unknown)"
short="$(basename "${model}" | sed 's/[^A-Za-z0-9._-]/_/g')"
OUT="${ROOT}/jetson/log/bench_${stamp}_${short}"
RUN_NAME="bench_e2e_${stamp}"
mkdir -p "${OUT}" "${ROOT}/elevator_vlm/runs"

# ---- 通したときの状態 ----------------------------------------------------------------
power="$(nvpmodel -q 2> /dev/null || sudo -n nvpmodel -q 2> /dev/null || true)"
power_name="$(printf '%s\n' "${power}" | grep -m 1 'Power Mode' | sed 's/.*Power Mode: *//')"
{
  echo "日時: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "Jetson Linux: $(head -n 1 /etc/nv_tegra_release 2> /dev/null || echo 不明)"
  echo "メモリ: $(awk '/^MemTotal:/ { printf "%.1f GiB", $2 / 1048576 }' /proc/meminfo)"
  echo "電源モード: ${power_name:-不明}"
  echo "モデル: ${model}"
  echo "リポジトリ: $(git -C "${ROOT}" rev-parse --short HEAD 2> /dev/null || echo 不明)"
  for name in elevator-vlm elevator-bench elevator-bench-gate; do
    echo "${name}: $("${DOCKER[@]}" inspect -f '{{.Config.Image}} {{.Image}}' "${name}" 2> /dev/null | cut -c1-90 || echo なし)"
  done
} > "${OUT}/env.txt"
cat "${OUT}/env.txt"
echo

# 通している間の、電力・GPU の使用率・温度（1 秒ごと）
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

# ---- 通す ------------------------------------------------------------------------
# ROS 2 のパラメータは型を見るので、時間は小数の形で渡す（900 ではなく 900.0）
timeout_s="$(awk -v t="${BENCH_TIMEOUT_S:-900}" 'BEGIN { printf "%.1f", t }')"
runner_args=(-p out_dir:="/runs/${RUN_NAME}" -p task_timeout_s:="${timeout_s}")
if [ -n "${BENCH_PLAN_FILE:-}" ]; then
  [ -f "${BENCH_PLAN_FILE}" ] || { echo "NG: 計画のファイルがありません: ${BENCH_PLAN_FILE}" >&2; exit 1; }
  cp "${BENCH_PLAN_FILE}" "${ROOT}/elevator_vlm/runs/${RUN_NAME}_plan.json"
  runner_args+=(-p plan_file:="/runs/${RUN_NAME}_plan.json")
fi
echo "==== 通します（計画 → 承認して実行 → エレベーターを呼ぶ）===="
"${DOCKER[@]}" exec -e ROS_DOMAIN_ID="${DOMAIN}" -e ROS_LOCALHOST_ONLY=1 elevator-bench /entrypoint.sh \
  ros2 run elevator_sim e2e_runner --ros-args "${runner_args[@]}" 2>&1 | tee "${OUT}/e2e.txt"
code=${PIPESTATUS[0]}
stop_tegrastats

# ---- 記録を集める ---------------------------------------------------------------------
src="${ROOT}/elevator_vlm/runs/${RUN_NAME}"
cp "${src}/summary.json" "${OUT}/" 2> /dev/null || true
cp "${src}"/*.jpg "${OUT}/" 2> /dev/null || true
run_dir="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1])).get("run_dir") or "")' "${OUT}/summary.json" 2> /dev/null || true)"
if [ -n "${run_dir}" ]; then
  cp "${ROOT}/elevator_vlm/runs/${run_dir#/runs/}/events.jsonl" "${OUT}/" 2> /dev/null || true
  "${DOCKER[@]}" exec elevator-bench /entrypoint.sh python3 /workspace/scripts/inside/analyze_run.py "${run_dir}" \
    > "${OUT}/analysis.txt" 2>&1 || true
fi
"${DOCKER[@]}" logs --tail 400 elevator-bench > "${OUT}/bench.log" 2>&1 || true
"${DOCKER[@]}" logs --tail 200 elevator-bench-gate > "${OUT}/safety_gate.log" 2>&1 || true
"${DOCKER[@]}" logs --tail 200 elevator-vlm > "${OUT}/vlm.log" 2>&1 || true

# 消費電力の最大（tegrastats の「名前 今mW/平均mW」を、レールごとに集める）
if [ -f "${OUT}/tegrastats.log" ]; then
  python3 - "${OUT}/tegrastats.log" > "${OUT}/power.txt" << 'PY'
import re
import sys

rails = {}
totals = []
gpu = []
for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
    found = re.findall(r"([A-Z][A-Z0-9_]+) (\d+)mW/(\d+)mW", line)
    for name, now, _avg in found:
        rails.setdefault(name, []).append(int(now))
    if found:
        totals.append(sum(int(now) for _n, now, _a in found))
    m = re.search(r"GR3D_FREQ (\d+)%", line)
    if m:
        gpu.append(int(m.group(1)))
print(f"記録した秒数: {len(totals)}")
for name, values in rails.items():
    print(f"{name}: 最大 {max(values) / 1000:.1f} W、平均 {sum(values) / len(values) / 1000:.1f} W")
if totals:
    print(f"レールの合計（目安）: 最大 {max(totals) / 1000:.1f} W、平均 {sum(totals) / len(totals) / 1000:.1f} W")
if gpu:
    print(f"GPU の使用率: 最大 {max(gpu)}%、平均 {sum(gpu) / len(gpu):.0f}%")
PY
fi

echo
echo "==== まとめ ===="
python3 - "${OUT}/summary.json" << 'PY' 2> /dev/null || echo "  まとめ（summary.json）がありません. e2e.txt を見てください"
import json
import sys

s = json.load(open(sys.argv[1], encoding="utf-8"))
print(f"  結果: {s.get('result')}（{s.get('outcome')}）")
print(f"  実行系: {s.get('task_state')} {s.get('task_message') or ''}")
ref = s.get("referee") or {}
print(f"  真値による判定: {ref.get('summary', '不明')}")
for i, st in enumerate(s.get("steps") or []):
    print(f"  手順 {i + 1}: {st.get('phase')}（判定 {st.get('checks')} 回）{st.get('message') or ''}")
PY
[ -f "${OUT}/power.txt" ] && sed 's/^/  /' "${OUT}/power.txt"
grep -h -E "^判定 [0-9]+ 回" "${OUT}/analysis.txt" 2> /dev/null | head -n 1 | sed 's/^/  VLM の応答: /'
echo
echo "結果: ${OUT}"
echo "共有するときは、jetson/log/ をコミットしてプッシュしてください（jetson/bench/README.md の「結果を送る」）."
exit "${code}"
