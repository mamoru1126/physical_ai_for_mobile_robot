#!/bin/bash
# Orin から、つながりを確かめる（設定、サービス、カメラ、ロボットの PC の橋渡し、安全ゲート、実行系）.
# ロボットは動かさない（--zero を付けたときだけ、速度 0 の指令を 3 秒流す）. 手順は jetson/robot/link_check.md.
#   bash jetson/robot/check_link.sh            15 秒ほど受けて、結果を出す
#   bash jetson/robot/check_link.sh --zero     最後に、安全ゲートの入力へ速度 0 を 3 秒流す（安全ゲート → 橋渡し →
#                                              ロボットの PC で rostopic echo /cmd_vel に 0 が出れば、速度の通り道がつながっている）
#   bash jetson/robot/check_link.sh --save     結果を jetson/log/link_check_<日時>.txt にも残す
# 終わりのコード: 0 は NG なし（注意はあってもよい）、3 は NG あり.
set -uo pipefail
source "$(dirname "$0")/lib.sh"
cd "${ROOT}"

zero=()
save=0
for a in "$@"; do
  case "${a}" in
    --zero) zero=(--zero) ;;
    --save) save=1 ;;
    *) fail "知らない引数です: ${a}（使えるもの: --zero、--save）" ;;
  esac
done
if [ "${save}" = "1" ] && [ -z "${CHECK_LINK_LOG:-}" ]; then
  mkdir -p "${ROOT}/jetson/log"
  export CHECK_LINK_LOG="${ROOT}/jetson/log/link_check_$(date +%Y%m%d_%H%M%S).txt"
  bash "${ROBOT_DIR}/check_link.sh" "${zero[@]+"${zero[@]}"}" 2>&1 | tee "${CHECK_LINK_LOG}"
  rc="${PIPESTATUS[0]}"
  say "記録しました: ${CHECK_LINK_LOG#"${ROOT}"/}"
  exit "${rc}"
fi

DOMAIN="$(robot_domain)"
ODOM_TOPIC="$(robot_odom_topic)"
TASK_FILE="${ROBOT_CONFIG:-${TASK_CONF}}"
[ -f "${TASK_FILE}" ] || TASK_FILE="${ROBOT_DIR}/task_robot.yaml"
GATE_FILE="${GATE_CONF}"
[ -n "$(read_file "${GATE_FILE}")" ] || GATE_FILE="${ROOT}/safety_gate/ros2_ws/src/safety_gate/config/config.yaml"
ng=0

say "== つながりの確かめ（$(date '+%Y-%m-%d %H:%M')）=="
say ""
say "== 設定（Orin の中）=="
for pair in "${ROBOT_ENV}:bash jetson/robot/robot_up.sh" "${TASK_CONF}:bash jetson/robot/robot_up.sh" \
            "${GATE_CONF}:bash safety_gate/install.sh" "${CAMERA_ENV}:bash jetson/camera/install.sh"; do
  f="${pair%%:*}"
  if [ -n "$(read_file "${f}")" ]; then
    say "[OK]   ${f}"
  else
    say "[注意] ${f} がありません（${pair#*:} で作る）"
  fi
done
[ "${TASK_FILE}" = "${TASK_CONF}" ] || say "       実行系の設定は ${TASK_FILE#"${ROOT}"/} を使って確かめます"
[ "${GATE_FILE}" = "${GATE_CONF}" ] || say "       安全ゲートの設定は ${GATE_FILE#"${ROOT}"/} を使って確かめます"
gate_domain="$(yaml_get "${GATE_FILE}" ros_domain_id)"
cam_domain="$(env_get "${CAMERA_ENV}" ROS_DOMAIN_ID)"
if [ "${gate_domain:-0}" = "${DOMAIN}" ] && { [ -z "${cam_domain}" ] || [ "${cam_domain}" = "${DOMAIN}" ]; }; then
  say "[OK]   ROS 2 のドメイン ${DOMAIN}（実行系、安全ゲート、カメラ）"
else
  say "[NG]   ROS 2 のドメインがそろっていません: 実行系 ${DOMAIN}、安全ゲート ${gate_domain:-0}、カメラ ${cam_domain:-?}"
  say "         → bash jetson/robot/set_domain.sh <番号>（ロボットの PC の橋渡しと同じ番号にする）"
  ng=1
fi
say "       オドメトリのトピック: ${ODOM_TOPIC}（${ROBOT_ENV} の ODOM_TOPIC）"
say "       ⚠ 最終の速度指令のトピック: $(yaml_get "${GATE_FILE}" topics.cmd_out)（${GATE_CONF} の topics.cmd_out）"
say ""
say "== サービスとコンテナ（Orin の中）=="
for svc in usb-camera safety-gate; do
  if ! systemctl cat "${svc}.service" > /dev/null 2>&1; then
    say "[注意] ${svc} のサービスが入っていません"
  elif systemctl is-active --quiet "${svc}"; then
    say "[OK]   ${svc}（動いている. 電源を入れると自動で起動: $(systemctl is-enabled "${svc}" 2> /dev/null)）"
  else
    say "[NG]   ${svc} が動いていません → sudo systemctl restart ${svc}、ログ: journalctl -u ${svc} -n 30"
    ng=1
  fi
done
for c in elevator-vlm elevator-task; do
  if "${DOCKER[@]}" ps --format '{{.Names}}' 2> /dev/null | grep -qx "${c}"; then
    say "[OK]   ${c}（動いている）"
  else
    say "[--]   ${c} は動いていません（bash jetson/robot/robot_up.sh で起動）"
  fi
done
if curl -sf --noproxy '*' --max-time 3 http://127.0.0.1:8000/v1/models > /dev/null 2>&1; then
  say "[OK]   VLM が答える（ポート 8000）"
fi
say ""

"${DOCKER[@]}" image inspect "${RUNTIME_IMAGE}" > /dev/null 2>&1 \
  || fail "実行系のイメージ（${RUNTIME_IMAGE}）がありません（bash jetson/robot/robot_up.sh を一度、ネットのあるところで）"
mapfile -t ros_env < <(ros_env_args)
"${DOCKER[@]}" run --rm --network host --ipc host "${ros_env[@]}" \
  -v "${ROBOT_DIR}/check_link.py:/check_link.py:ro" \
  -v "${TASK_FILE}:/cfg/task.yaml:ro" -v "${GATE_FILE}:/cfg/gate.yaml:ro" \
  "${RUNTIME_IMAGE}" python3 /check_link.py --task /cfg/task.yaml --gate /cfg/gate.yaml \
  --odom "${ODOM_TOPIC}" "${zero[@]+"${zero[@]}"}"
rc=$?
if [ "${rc}" != "0" ] && [ "${rc}" != "3" ]; then
  fail "確かめのプログラムが途中で止まりました（コード ${rc}）"
fi
[ "${rc}" = "3" ] && ng=1
say ""
say "ロボットの PC（ROS 1）の側の確かめは jetson/robot/link_check.md の 6."
[ "${ng}" = "0" ] || exit 3
exit 0
