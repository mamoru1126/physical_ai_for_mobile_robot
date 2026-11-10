#!/bin/bash
# コンテナの中で実行する. Gazebo（画面なし）と実行系を起動し、通し試験を1回行う.
#   使い方: e2e.sh <mock|real>
#   環境変数:
#     RENDER        egl（NVIDIA の GPU で描画. 既定）か xvfb（GPU なし. ソフトウェア描画）
#     PRINT_IMAGES  true にすると、要所の画像を小さくして base64 でログに出す（CI 用）
#     PRINT_SAMPLES true にすると、正解つきの画像の組（机上評価用）を作って base64 でログに出す（CI 用）
#     DOOR_HOLD_S / DOOR_SENSOR / BOARD_FLOOR / DEST_FLOOR   エレベーターの設定
set -u
MODE="${1:-mock}"
OUT="/runs/e2e_$(date +%Y%m%d_%H%M%S)_${MODE}"
mkdir -p "${OUT}"

LAUNCH_ARGS="vlm:=${MODE} gui:=false log_dir:=/runs"
LAUNCH_ARGS="${LAUNCH_ARGS} door_hold_s:=${DOOR_HOLD_S:-6.0} door_sensor:=${DOOR_SENSOR:-true}"
LAUNCH_ARGS="${LAUNCH_ARGS} board_floor:=${BOARD_FLOOR:-1} dest_floor:=${DEST_FLOOR:-3}"

if [ "${RENDER:-egl}" = "xvfb" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  xvfb-run -a -s "-screen 0 1280x720x24" \
    ros2 launch elevator_sim demo.launch.py ${LAUNCH_ARGS} headless_rendering:=false > "${OUT}/launch.log" 2>&1 &
else
  ros2 launch elevator_sim demo.launch.py ${LAUNCH_ARGS} headless_rendering:=true > "${OUT}/launch.log" 2>&1 &
fi
LAUNCH_PID=$!

ros2 run elevator_sim e2e_runner --ros-args -p out_dir:="${OUT}" -p print_images:="${PRINT_IMAGES:-false}"
CODE=$?

kill -INT "${LAUNCH_PID}" 2>/dev/null
for _ in $(seq 1 20); do
  kill -0 "${LAUNCH_PID}" 2>/dev/null || break
  sleep 0.5
done
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null

if [ "${PRINT_SAMPLES:-false}" = "true" ] && [ -f "${OUT}/summary.json" ]; then
  RUN_DIR="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('run_dir') or '')" "${OUT}/summary.json")"
  if [ -n "${RUN_DIR}" ] && python3 /workspace/scripts/inside/export_samples.py "${RUN_DIR}" "${OUT}/samples"; then
    for f in "${OUT}"/samples/*; do
      echo "SAMPLE_BEGIN $(basename "${f}")"
      base64 -w 0 "${f}"
      echo
      echo "SAMPLE_END"
    done
  fi
fi

echo "---- launch.log の警告とエラー（重複はまとめる）----"
grep -i -E "error|warn|exception|traceback|\\[err\\]" "${OUT}/launch.log" | sed -E 's/[0-9]+\.[0-9]+/#/g' | sort | uniq -c | sort -rn | head -n 40
echo "---- launch.log（末尾）----"
tail -n "${LAUNCH_LOG_LINES:-80}" "${OUT}/launch.log"
echo "記録: ${OUT}（ホストでは runs/ の下）"
exit "${CODE}"
