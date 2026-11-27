#!/bin/bash
# ROS 2 のドメインを、Orin の中の 3 か所でまとめて変える（ロボットの PC の橋渡しと同じ番号にする）.
#   bash jetson/robot/set_domain.sh          今の番号を見る
#   bash jetson/robot/set_domain.sh 0        0 にそろえる（ROS_DOMAIN_ID を決めていない ROS 2 は 0）
#
# 変えるところ:
#   /etc/elevator-task/robot.env    ROS_DOMAIN_ID=     （実行系）
#   /etc/safety-gate/config.yaml    ros_domain_id:     （安全ゲート）
#   /etc/usb-camera/camera.env      ROS_DOMAIN_ID=     （カメラ）
# 変えたあと、カメラと安全ゲートのサービスを起動し直す. 実行系が動いていれば、それも起動し直す.
set -euo pipefail
source "$(dirname "$0")/lib.sh"

show() {
  say "  実行系     ${ROBOT_ENV}: $(env_get "${ROBOT_ENV}" ROS_DOMAIN_ID)"
  say "  安全ゲート ${GATE_CONF}: $(yaml_get "${GATE_CONF}" ros_domain_id)"
  say "  カメラ     ${CAMERA_ENV}: $(env_get "${CAMERA_ENV}" ROS_DOMAIN_ID)"
}

if [ "$#" -eq 0 ]; then
  say "今の ROS 2 のドメイン:"
  show
  exit 0
fi
new="$1"
[[ "${new}" =~ ^[0-9]+$ ]] && [ "$((10#${new}))" -le 101 ] || fail "ドメインは 0〜101 の数にしてください: ${new}"
new="$((10#${new}))"

ensure_task_etc
say "変える前:"
show

# KEY=VALUE のファイル: 行があれば書き換え、なければ足す
set_env() {
  local file="$1" key="$2" value="$3"
  [ -f "${file}" ] || { warn "${file} がありません（飛ばします）"; return 0; }
  if sudo grep -q "^${key}=" "${file}"; then
    sudo sed -i "s/^${key}=.*/${key}=${value}/" "${file}"
  else
    printf '%s=%s\n' "${key}" "${value}" | sudo tee -a "${file}" > /dev/null
  fi
}
set_env "${ROBOT_ENV}" ROS_DOMAIN_ID "${new}"
set_env "${CAMERA_ENV}" ROS_DOMAIN_ID "${new}"
if [ -f "${GATE_CONF}" ]; then
  if sudo grep -q '^ros_domain_id:' "${GATE_CONF}"; then
    sudo sed -i "s/^ros_domain_id:.*/ros_domain_id: ${new}/" "${GATE_CONF}"
  else
    sudo sed -i "1i ros_domain_id: ${new}" "${GATE_CONF}"
  fi
else
  warn "${GATE_CONF} がありません（飛ばします）"
fi

say "変えた後:"
show
for svc in usb-camera safety-gate; do
  if systemctl cat "${svc}.service" > /dev/null 2>&1; then
    sudo systemctl restart "${svc}"
    say "起動し直しました: ${svc}"
  fi
done
"${DOCKER[@]}" rm -f elevator-ros2 > /dev/null 2>&1 || true
if "${DOCKER[@]}" ps --format '{{.Names}}' 2> /dev/null | grep -qx elevator-task; then
  say "実行系を起動し直します"
  bash "${ROBOT_DIR}/robot_up.sh"
fi
say ""
say "ロボットの PC の橋渡しも、同じ番号（${new}）になっているかを確かめてください（jetson/robot/link_check.md の 2）"
