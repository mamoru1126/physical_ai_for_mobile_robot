#!/bin/bash
# ROS 1 の roscore を見つけてから、決めたトピックだけを橋渡しする.
# roscore が止まったら、コンテナを終わらせる（サービスが起動し直し、また roscore を探す）.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/ros1_bridge_ws/install/local_setup.bash
TOOL=/opt/ros1_bridge/ros1_master.py

want_master="${ROS_MASTER_URI:-http://localhost:11311}"
unset ROS_MASTER_URI ROS_HOSTNAME
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
# 途中から・ここまで（VLM の指示で走る区間の開始と終了の合図、状態）. 空なら橋渡ししない
VLM_COMMAND_TOPIC="${VLM_COMMAND_TOPIC-/vlm_drive/command}"
VLM_STATE_TOPIC="${VLM_STATE_TOPIC-/vlm_drive/state}"
VLM_ACTIVE_TOPIC="${VLM_ACTIVE_TOPIC-/vlm_drive/active}"
extra=()
[ -n "${VLM_COMMAND_TOPIC}" ] && extra+=("${VLM_COMMAND_TOPIC}=std_msgs/msg/String")
[ -n "${VLM_STATE_TOPIC}" ] && extra+=("${VLM_STATE_TOPIC}=std_msgs/msg/String")
[ -n "${VLM_ACTIVE_TOPIC}" ] && extra+=("${VLM_ACTIVE_TOPIC}=std_msgs/msg/Bool")

# 名前と IP の組（例: robot-pc:192.168.0.100）を /etc/hosts に足す. auto は下で使う
fix_hosts=0
for word in $(echo "${ROS1_HOSTS:-auto}" | tr -d '"'); do
  if [ "${word}" = "auto" ]; then
    fix_hosts=1
  else
    echo "${word#*:} ${word%%:*}" >> /etc/hosts
  fi
done

# ROS 2 の相手の IP が書いてあれば、Fast DDS に教える（自動で見つからないとき用）
if [ -n "${ROS2_PEERS:-}" ]; then
  {
    echo '<?xml version="1.0" encoding="UTF-8"?>'
    echo '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">'
    echo '  <participant profile_name="peers" is_default_profile="true"><rtps><builtin><initialPeersList>'
    for peer in $(echo "${ROS2_PEERS}" | tr -d '"'); do
      echo "    <locator><udpv4><address>${peer}</address></udpv4></locator>"
    done
    echo '    <locator><udpv4><address>127.0.0.1</address></udpv4></locator>'
    echo '  </initialPeersList></builtin></rtps></participant>'
    echo '</profiles>'
  } > /tmp/fastdds_peers.xml
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_peers.xml
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  echo "ROS 2 の相手: ${ROS2_PEERS}"
fi

echo "ROS 2: Humble、ドメイン ${ROS_DOMAIN_ID:-0}"
echo "ROS 1 -> ROS 2: ${ODOM_TOPIC} (nav_msgs/Odometry)、${CLOUD_TOPIC} (sensor_msgs/PointCloud2)"
echo "ROS 2 -> ROS 1: ${CMD_VEL_TOPIC} (geometry_msgs/Twist)"
if [ "${BRIDGE_TF:-true}" = "true" ]; then
  echo "ROS 1 <-> ROS 2: /tf、/tf_static (tf2_msgs/TFMessage)"
fi
[ -n "${VLM_COMMAND_TOPIC}" ] && echo "ROS 1 -> ROS 2: ${VLM_COMMAND_TOPIC} (std_msgs/String: start / finish / stop)"
[ -n "${VLM_STATE_TOPIC}${VLM_ACTIVE_TOPIC}" ] && echo "ROS 2 -> ROS 1: ${VLM_STATE_TOPIC} (std_msgs/String)、${VLM_ACTIVE_TOPIC} (std_msgs/Bool)"

until uri="$(python3 "${TOOL}" find "${want_master}")"; do
  if [ "${want_master}" = "auto" ]; then
    echo "roscore を探しています（同じネットワークのポート 11311）"
  else
    echo "roscore を待っています（${want_master}）"
  fi
  sleep 3
done
export ROS_MASTER_URI="${uri}"

# この PC の IP. roscore の PC へ届く IP を選ぶ（向こうのノードが、この IP でつないでくる）
if [ -z "${ROS1_IP:-}" ] || [ "${ROS1_IP}" = "auto" ]; then
  master_host="$(echo "${ROS_MASTER_URI}" | sed -E 's#^[a-z]+://([^:/]+).*#\1#')"
  master_ip="$(getent ahostsv4 "${master_host}" | awk 'NR==1{print $1}')"
  ROS1_IP="$(ip -4 route get "${master_ip:-${master_host}}" 2> /dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')"
fi
[ -n "${ROS1_IP}" ] || { echo "NG: この PC の IP が決められません. ROS1_IP を書いてください" >&2; exit 1; }
export ROS_IP="${ROS1_IP}"
echo "roscore につながりました: ${ROS_MASTER_URI}（この PC の IP ${ROS_IP}）"

add_hosts() {
  [ "${fix_hosts}" = "1" ] || return 0
  local names
  names="$(python3 "${TOOL}" fix-hosts "${ROS_MASTER_URI}" 2> /dev/null || true)"
  [ -z "${names}" ] || echo "名前が引けないので、roscore と同じ PC とみなしました: $(echo ${names})"
}
add_hosts

python3 "${TOOL}" set-topics "${ROS_MASTER_URI}" "${ODOM_TOPIC}" "${CLOUD_TOPIC}" "${CMD_VEL_TOPIC}" "${BRIDGE_TF:-true}" "${extra[@]}"
ros2 run ros1_bridge parameter_bridge &
bridge_pid=$!
trap 'kill ${bridge_pid} 2> /dev/null || true' TERM INT

# 橋渡しが止まるか、roscore が 3 回続けて見えなくなったら終わる.
# あとから起動した向こうのノードの名前も、ここで見ておく
miss=0
while kill -0 "${bridge_pid}" 2> /dev/null; do
  sleep 5
  if python3 "${TOOL}" check "${ROS_MASTER_URI}"; then
    miss=0
    add_hosts
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
