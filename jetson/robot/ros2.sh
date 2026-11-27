#!/bin/bash
# Orin で ros2 コマンドを使う. Orin には ROS 2 が入っていないので、実行系のイメージの中で動かす.
# ドメインは実行系と同じ（/etc/elevator-task/robot.env の ROS_DOMAIN_ID. なければ 42）.
#   bash jetson/robot/ros2.sh topic list
#   bash jetson/robot/ros2.sh topic hz /camera/compressed        （Ctrl+C で止める）
#   bash jetson/robot/ros2.sh topic echo /safety_gate/state
#   bash jetson/robot/ros2.sh topic info /cmd_vel --verbose
#   bash jetson/robot/ros2.sh node list
# 1 回目は道具のコンテナ（elevator-ros2）を起動し、相手を見つけるまで数秒待つ. 2 回目からはすぐ.
# 道具のコンテナは bash jetson/robot/robot_down.sh で消える（残しておいても害はない）.
set -euo pipefail
source "$(dirname "$0")/lib.sh"
if [ "$#" -eq 0 ]; then
  sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
  exit 0
fi
NAME=elevator-ros2
"${DOCKER[@]}" image inspect "${RUNTIME_IMAGE}" > /dev/null 2>&1 \
  || fail "実行系のイメージ（${RUNTIME_IMAGE}）がありません（bash jetson/robot/robot_up.sh を一度、ネットのあるところで）"
mapfile -t ros_env < <(ros_env_args)
want="$(printf '%s ' "${ros_env[@]}") $("${DOCKER[@]}" image inspect -f '{{.Id}}' "${RUNTIME_IMAGE}")"
have=""
if [ "$("${DOCKER[@]}" inspect -f '{{.State.Running}}' "${NAME}" 2> /dev/null || true)" = "true" ]; then
  have="$("${DOCKER[@]}" inspect -f '{{ index .Config.Labels "elevator.ros2" }}' "${NAME}" 2> /dev/null || true)"
fi
# ドメインかイメージが変わったら、道具のコンテナを作り直す
if [ "${have}" != "${want}" ]; then
  "${DOCKER[@]}" rm -f "${NAME}" > /dev/null 2>&1 || true
  "${DOCKER[@]}" run -d --name "${NAME}" --label "elevator.ros2=${want}" --network host --ipc host \
    "${ros_env[@]}" "${RUNTIME_IMAGE}" sleep infinity > /dev/null
  # ros2 のデーモンを起動して、相手を見つけておく
  "${DOCKER[@]}" exec "${NAME}" /entrypoint.sh ros2 daemon start > /dev/null 2>&1 || true
  sleep 3
fi
tty=()
[ -t 0 ] && [ -t 1 ] && tty=(-it)
exec "${DOCKER[@]}" exec ${tty[@]+"${tty[@]}"} "${NAME}" /entrypoint.sh ros2 "$@"
