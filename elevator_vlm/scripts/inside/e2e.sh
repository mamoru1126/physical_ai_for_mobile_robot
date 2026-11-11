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

if [ "${CODE}" != "0" ]; then
  # 失敗したら、トピックが届いているかを記録する（シミュレータを止める前に）
  {
    echo "==== ROS のトピックの周波数（各 4 秒）===="
    for t in /clock /camera/image_raw /camera/camera_info /odom /ground_truth/odom /elevator/joint_states; do
      echo "--- ${t}"
      timeout 4 ros2 topic hz "${t}" 2>&1 | tail -n 3
    done
    echo "==== Gazebo のトピック ===="
    timeout 5 gz topic -l 2>&1 | head -n 60
    echo "==== 環境 ===="
    env | grep -E "^(GZ_|ROS_|RMW_|http_proxy|https_proxy|no_proxy|HTTP_PROXY|NO_PROXY)" | sed -E 's#(://)[^@/]*@#\1***@#'
  } > "${OUT}/diagnostics.txt" 2>&1
  cp -r "$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('run_dir') or '/nonexistent')" "${OUT}/summary.json" 2>/dev/null)/events.jsonl" "${OUT}/" 2>/dev/null
  echo "---- 診断（${OUT}/diagnostics.txt）----"
  cat "${OUT}/diagnostics.txt"
fi

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
