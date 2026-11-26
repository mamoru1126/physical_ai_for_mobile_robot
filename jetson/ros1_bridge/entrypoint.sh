#!/bin/bash
# ROS 1 の roscore を待ってから、決めたトピックだけを橋渡しする.
# roscore が止まったら、コンテナを終わらせる（サービスが起動し直し、また roscore を待つ）.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/ros1_bridge/fastdds_udp.xml

: "${ROS_MASTER_URI:?ROS_MASTER_URI を /etc/ros1-bridge/bridge.env に書いてください}"
master_host="$(echo "${ROS_MASTER_URI}" | sed -E 's#^[a-z]+://([^:/]+).*#\1#')"

for pair in $(echo "${ROS1_HOSTS:-}" | tr -d '"'); do
  echo "${pair#*:} ${pair%%:*}" >> /etc/hosts
done

if [ -z "${ROS1_IP:-}" ] || [ "${ROS1_IP}" = "auto" ]; then
  master_ip="$(getent ahostsv4 "${master_host}" | awk 'NR==1{print $1}')"
  ROS1_IP="$(ip -4 route get "${master_ip:-${master_host}}" 2> /dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')"
fi
[ -n "${ROS1_IP}" ] || { echo "NG: この PC の IP が決められません. ROS1_IP を書いてください" >&2; exit 1; }
export ROS_IP="${ROS1_IP}"
unset ROS_HOSTNAME

echo "ROS 1: roscore ${ROS_MASTER_URI}、この PC の IP ${ROS_IP}"
echo "ROS 2: ドメイン ${ROS_DOMAIN_ID:-0}"
echo "ROS 1 -> ROS 2: ${ODOM_TOPIC} (nav_msgs/Odometry)、${CLOUD_TOPIC} (sensor_msgs/PointCloud2)"
echo "ROS 2 -> ROS 1: ${CMD_VEL_TOPIC} (geometry_msgs/Twist)"

master_up() { timeout 5 rosparam get /rosdistro > /dev/null 2>&1; }
until master_up; do
  echo "roscore を待っています（${ROS_MASTER_URI}）"
  sleep 3
done
echo "roscore につながりました"

# 橋渡しするトピックの一覧. parameter_bridge は ROS 1 のパラメーター topics を読む
cat > /tmp/bridge.yaml <<YAML
topics:
  - {topic: ${ODOM_TOPIC}, type: nav_msgs/msg/Odometry, queue_size: 10}
  - {topic: ${CLOUD_TOPIC}, type: sensor_msgs/msg/PointCloud2, queue_size: 2}
  - {topic: ${CMD_VEL_TOPIC}, type: geometry_msgs/msg/Twist, queue_size: 1}
YAML
rosparam load /tmp/bridge.yaml

ros2 run ros1_bridge parameter_bridge &
bridge_pid=$!
trap 'kill ${bridge_pid} 2> /dev/null || true' TERM INT

# 橋渡しが止まるか、roscore が 3 回続けて見えなくなったら終わる
miss=0
while kill -0 "${bridge_pid}" 2> /dev/null; do
  sleep 5
  if master_up; then
    miss=0
  else
    miss=$((miss + 1))
    echo "roscore が見えません（${miss}/3）"
    if [ "${miss}" -ge 3 ]; then
      echo "roscore が止まったので、起動し直します"
      kill "${bridge_pid}" 2> /dev/null || true
      wait "${bridge_pid}" || true
      exit 1
    fi
  fi
done
wait "${bridge_pid}"
