#!/bin/bash
# Orin から VLM の走行を操作する（実行系の WebUI のボタンと同じことを、コマンドで）. 先に robot_up.sh で起動しておく.
#   bash jetson/robot/drive.sh plans                    用意済みの計画の一覧
#   bash jetson/robot/drive.sh load path_10m.json       用意済みの計画を読み込む
#   bash jetson/robot/drive.sh plan "道に沿って走って"    指示文から計画を作る（VLM が作る）
#   bash jetson/robot/drive.sh arm                      計画を承認して、開始の合図を待つ（手順を選ぶなら arm 2 3）
#   bash jetson/robot/drive.sh start                    走り出す（arm した計画. ROS1システムからの start と同じ）
#   bash jetson/robot/drive.sh finish                   ここまで（その場で止めて、成功として終える）
#   bash jetson/robot/drive.sh stop                     停止（失敗として止める. reset まで動かない）
#   bash jetson/robot/drive.sh reset                    停止を解除する
#   bash jetson/robot/drive.sh state                    状態を出し続ける（Ctrl+C で終わる）
# ロボット全体を止めるのは、台車の非常停止. stop は VLM の側を止めるだけ.
set -uo pipefail
URL="${DRIVE_URL:-http://127.0.0.1:8080}"

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; }

read -r -d '' PY_RESULT <<'PY'
import json, sys
r = json.load(sys.stdin)
print(("OK: " if r.get("ok") else "NG: ") + str(r.get("message", "")))
sys.exit(0 if r.get("ok") else 3)
PY

read -r -d '' PY_PLAN <<'PY'
import json, sys
s = json.load(sys.stdin)
steps = (s.get("plan") or {}).get("steps") or []
cfg = s.get("follow_path_cfg") or {}
if not steps:
    print("計画: なし（%s）" % (s.get("plan_error") or s.get("plan_state")))
    sys.exit(3)
print("計画:")
for i, st in enumerate(steps, 1):
    do = st.get("do") or {}
    extra = [do.get("skill", "")]
    if do.get("skill") == "follow_path":
        d = do.get("distance_m") or 0
        extra.append("%g m" % d if d else "距離を決めない（上限 %g m）" % cfg.get("max_distance_m", 100))
        if do.get("stop_when"):
            extra.append("止まる条件: " + do["stop_when"])
    print("  %d. %s（%s）" % (i, st.get("instruction", ""), "、".join(extra)))
print("答え 1 回で最大 %g m 進みます. 走らせる: arm → start" % cfg.get("step_m", 1.0))
PY

read -r -d '' PY_STATE <<'PY'
import json, sys
s = json.load(sys.stdin)
d = s.get("drive") or {}
out = ["状態 %s" % s.get("task_state"), "走っている" if d.get("active") else "止まっている"]
if d.get("armed"):
    out.append("承認 手順 %s〜%s" % tuple(d["armed"]))
if d.get("active") and d.get("step"):
    out.append("手順 %s %s" % (d.get("step"), d.get("skill", "")))
if d.get("traveled_m") is not None:
    out.append("%.1f / %s m" % (d["traveled_m"], d.get("limit_m")))
if d.get("result"):
    out.append("前回の終わり方 %s" % d["result"])
if s.get("stopped"):
    out.append("停止中（reset で解除）")
if s.get("task_message"):
    out.append(str(s["task_message"])[:60])
print("、".join(out))
PY

# 状態（JSON）を STATE に取る
fetch_state() {
  STATE="$(curl -sf --noproxy '*' --max-time 5 "${URL}/api/state?since=999999999")" \
    || { echo "NG: 実行系に接続できません（bash jetson/robot/robot_up.sh で起動）"; exit 3; }
}

# post <パス> [JSON]: 結果（OK / NG と理由）を出す
post() {
  local body="${2:-}" out
  [ -n "${body}" ] || body='{}'
  if ! out="$(curl -sf --noproxy '*' --max-time 30 -X POST -H 'Content-Type: application/json' -d "${body}" "${URL}$1")"; then
    echo "NG: 実行系に接続できません（bash jetson/robot/robot_up.sh で起動）"
    return 3
  fi
  printf '%s' "${out}" | python3 -c "${PY_RESULT}"
}

show_plan() {
  fetch_state
  printf '%s' "${STATE}" | python3 -c "${PY_PLAN}"
}

range_body() {  # range_body <合図> [始め] [終わり]
  if [ -n "${2:-}" ]; then
    printf '{"command": "%s", "from": %d, "to": %d}' "$1" "$2" "${3:-$2}"
  else
    printf '{"command": "%s"}' "$1"
  fi
}

cmd="${1:-}"
shift || true
case "${cmd}" in
  plans)
    fetch_state
    printf '%s' "${STATE}" | python3 -c 'import json, sys; print("\n".join(json.load(sys.stdin).get("presets") or ["（なし）"]))'
    ;;
  load)
    [ -n "${1:-}" ] || { echo "NG: 計画の名前を付けてください（一覧: bash jetson/robot/drive.sh plans）"; exit 2; }
    post /api/load_preset "$(python3 -c 'import json, sys; print(json.dumps({"name": sys.argv[1]}))' "$1")" || exit 3
    show_plan
    ;;
  plan)
    [ -n "${1:-}" ] || { echo "NG: 指示文を付けてください（例 \"道に沿って走って\"）"; exit 2; }
    post /api/plan "$(python3 -c 'import json, sys; print(json.dumps({"instructions": sys.argv[1]}))' "$*")" || exit 3
    echo "VLM が計画を作っています"
    for _ in $(seq 1 90); do
      fetch_state
      [ "$(printf '%s' "${STATE}" | python3 -c 'import json, sys; print(json.load(sys.stdin).get("plan_state"))')" = "planning" ] || break
      sleep 1
    done
    show_plan
    ;;
  arm)
    if [ -n "${1:-}" ]; then
      post /api/arm "$(printf '{"from": %d, "to": %d}' "$1" "${2:-$1}")"
    else
      post /api/arm
    fi
    ;;
  start)
    post /api/command "$(range_body start "$@")" || { echo "  → 計画を読み込んで（load）、承認（arm）してから start"; exit 3; }
    ;;
  finish|stop|ping)
    post /api/command "$(range_body "${cmd}")"
    ;;
  reset)
    post /api/reset
    ;;
  state)
    last=""
    while true; do
      fetch_state
      line="$(printf '%s' "${STATE}" | python3 -c "${PY_STATE}")"
      if [ "${line}" != "${last}" ]; then
        printf '%s %s\n' "$(date +%H:%M:%S)" "${line}"
        last="${line}"
      fi
      sleep 1
    done
    ;;
  ""|-h|--help|help)
    usage
    ;;
  *)
    echo "NG: 知らないコマンドです: ${cmd}"
    usage
    exit 2
    ;;
esac
