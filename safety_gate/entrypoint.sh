#!/bin/bash
# 安全ゲートを起動する. 設定ファイルは、コンテナの外から /config/config.yaml に差し込む.
#   引数なし:   安全ゲートと WebUI を起動する
#   selftest:   にせの点群と指令で、通しの確認をする（CI 用）
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/safety_gate_ws/install/setup.bash

CONFIG="${SAFETY_GATE_CONFIG:-/config/config.yaml}"
if [ ! -f "${CONFIG}" ]; then
  echo "注意: 設定ファイル ${CONFIG} がないので、既定の設定で動かします"
  CONFIG="/opt/safety_gate_ws/install/safety_gate/share/safety_gate/config/config.yaml"
fi

# ROS 2 のドメインは、設定ファイルの ros_domain_id にそろえる
domain="$(python3 -c 'import sys, yaml; c = yaml.safe_load(open(sys.argv[1])) or {}; print(c.get("ros_domain_id", ""))' "${CONFIG}")"
if [ -n "${domain}" ]; then
  export ROS_DOMAIN_ID="${domain}"
fi
echo "設定ファイル: ${CONFIG}、ROS 2 のドメイン: ${ROS_DOMAIN_ID:-0}"

if [ "${1:-}" = "selftest" ]; then
  exec python3 /opt/safety_gate/selftest.py "${CONFIG}"
fi
exec ros2 launch safety_gate safety_gate.launch.py config:="${CONFIG}"
