#!/bin/bash
# 実機（ロボットに載せた Orin）で、VLM の指示で走る実行系を起動する. ネットにつながっていなくてよい.
#   bash jetson/robot/robot_up.sh              VLM が動いていなければ、既定のモデルで起動する
#   bash jetson/robot/robot_up.sh <モデル>      VLM を、そのモデルで起動する（動いていないとき）
#
# 起動するもの（一度起動すると、Orin の電源を入れ直しても自動で起動する. robot_down.sh で止めるまで）:
#   elevator-vlm    VLM のサーバ（jetson/vlm_up.sh. ポート 8000）
#   elevator-task   実行系（WebUI はポート 8080）
# 設定（初めて実行したときに、jetson/robot/ の雛形から作る）:
#   /etc/elevator-task/robot.env   ROS 2 のドメイン、オドメトリのトピック
#   /etc/elevator-task/task.yaml   カメラの取り付け、走る速さ、合図のトピックなど
# 前もって入れておくもの（ネットがあるうちに）:
#   カメラ        bash jetson/camera/install.sh（サービス usb-camera. 設定 /etc/usb-camera/camera.env）
#   安全ゲート    bash safety_gate/install.sh（サービス safety-gate. 設定 /etc/safety-gate/config.yaml）
#   イメージ      このスクリプトを一度ネットのあるところで実行すると、実行系のイメージを作る
#   VLM の重み    同じく、一度ネットのあるところで実行すると落としておける
#
# 環境変数（設定ファイルより優先する）:
#   ROBOT_DOMAIN_ID   ROS 2 のドメイン
#   ROBOT_ODOM_TOPIC  オドメトリのトピック
#   ROBOT_CONFIG      実行系の設定ファイル（既定 /etc/elevator-task/task.yaml）
#   ROBOT_REBUILD=1   実行系のイメージを作り直す（リポジトリを更新したとき. ネットが要る）
set -euo pipefail
source "$(dirname "$0")/lib.sh"
cd "${ROOT}"
VLM_PORT="${VLM_PORT:-8000}"

command -v docker > /dev/null 2>&1 || fail "Docker がありません（jetson/README.md の「Docker」）"
if [ -n "${ROBOT_CONFIG:-}" ]; then
  CONFIG="${ROBOT_CONFIG}"
else
  ensure_task_etc
  CONFIG="${TASK_CONF}"
fi
[ -f "${CONFIG}" ] || fail "設定がありません: ${CONFIG}"
DOMAIN="$(robot_domain)"
ODOM_TOPIC="$(robot_odom_topic)"
# プロキシの下では、イメージを作るコンテナ（apt-get、pip）にもプロキシの設定を渡す.
# プロキシがこの PC の上（127.0.0.1 など）にあっても届くよう、作るときもホストのネットワークを使う
BUILD_ARGS=(--network host)
for var in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do
  [ -n "${!var:-}" ] && BUILD_ARGS+=(--build-arg "${var}=${!var}")
done
online=0
curl -sf --max-time 5 -o /dev/null https://huggingface.co 2> /dev/null && online=1

# ---- 1. 実行系のイメージ ----------------------------------------------------------------
src="$(source_stamp)"
if [ "${ROBOT_REBUILD:-0}" = "1" ] || ! "${DOCKER[@]}" image inspect "${RUNTIME_IMAGE}" > /dev/null 2>&1; then
  [ "${online}" = "1" ] || fail "実行系のイメージ（${RUNTIME_IMAGE}）がなく、ネットもありません. ネットのあるところで一度実行してください"
  say "イメージを作ります: ${RUNTIME_IMAGE}（初回は数分〜十数分. 中で単体テストも通します）"
  "${DOCKER[@]}" build "${BUILD_ARGS[@]}" --label "elevator.src=${src}" -t "${RUNTIME_IMAGE}" \
    -f "${ROOT}/elevator_vlm/docker/Dockerfile.runtime" "${ROOT}/elevator_vlm"
elif [ "$(image_stamp)" != "${src}" ]; then
  warn "実行系のイメージが、リポジトリの今の中身から作ったものではありません（git pull の後に作り直していない）."
  warn "  ネットのあるところで作り直す: ROBOT_REBUILD=1 bash jetson/robot/robot_up.sh"
fi

# ---- 2. VLM のサーバ --------------------------------------------------------------------
vlm_ready() { curl -sf --noproxy '*' --max-time 3 "http://127.0.0.1:${VLM_PORT}/v1/models" > /dev/null 2>&1; }
if ! vlm_ready; then
  if "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx elevator-vlm; then
    say "VLM のサーバが起動の途中です. 答えられるようになるまで待ちます"
    for _ in $(seq 1 480); do
      vlm_ready && break
      sleep 5
    done
    vlm_ready || fail "VLM のサーバが答えません（ログ: ${DOCKER[*]} logs elevator-vlm）"
  else
    # 電源を入れ直しても、自動で起動するようにする（robot_down.sh --all で止めるまで）
    if [ "${online}" = "1" ]; then
      say "VLM のサーバを起動します（jetson/vlm_up.sh）"
      VLM_RESTART=unless-stopped bash "${ROOT}/jetson/vlm_up.sh" "$@"
    else
      say "VLM のサーバを、ネットなしで起動します（落としてある重みを使う）"
      VLM_RESTART=unless-stopped VLM_OFFLINE=1 bash "${ROOT}/jetson/vlm_up.sh" "$@"
    fi
  fi
fi

# ---- 3. 安全ゲート（点群を見るもの）とカメラの設定を確かめる --------------------------------
cmd_out=""
# パイプで grep -q に渡すと、pipefail の下では systemctl が SIGPIPE で終わり「入っていない」と誤る
if systemctl cat safety-gate.service > /dev/null 2>&1; then
  systemctl is-active --quiet safety-gate || warn "安全ゲート（safety-gate）が動いていません: sudo systemctl start safety-gate"
  if [ -z "$(read_file "${GATE_CONF}")" ]; then
    warn "安全ゲートの設定（${GATE_CONF}）が読めません"
  else
    cmd_out="$(yaml_get "${GATE_CONF}" topics.cmd_out)"
    gate_in="$(yaml_get "${GATE_CONF}" topics.cmd_in)"
    gate_domain="$(yaml_get "${GATE_CONF}" ros_domain_id)"
    if [ "$(yaml_get "${GATE_CONF}" safety.publish_when_idle)" != "false" ]; then
      warn "安全ゲートの設定（${GATE_CONF}）に publish_when_idle: false がありません."
      warn "  このままだと、VLM が走らせていない間も安全ゲートが 0 を出し続け、ROS1システムの指令とぶつかります."
      warn "  safety: の下に「publish_when_idle: false」を足して、sudo systemctl restart safety-gate"
    fi
    if [ "${gate_in:-/vlm_cmd_vel}" != "${RELAY_TOPIC}" ]; then
      warn "安全ゲートの入力（topics.cmd_in: ${gate_in}）が、実行系の出力（${RELAY_TOPIC}）と違います. 安全ゲートを通らず、ロボットは動きません"
      warn "  ${GATE_CONF} の topics.cmd_in を ${RELAY_TOPIC} にして、sudo systemctl restart safety-gate"
    fi
    if [ "${gate_domain:-0}" != "${DOMAIN}" ]; then
      warn "ROS 2 のドメインが違います（実行系 ${DOMAIN}、安全ゲート ${gate_domain:-0}）. そろえる: bash jetson/robot/set_domain.sh <番号>"
    fi
  fi
else
  warn "安全ゲート（safety-gate）のサービスが入っていません. 点群を見ないまま走ることになります（bash safety_gate/install.sh）"
fi
if systemctl cat usb-camera.service > /dev/null 2>&1; then
  cam_domain="$(env_get "${CAMERA_ENV}" ROS_DOMAIN_ID)"
  if [ -n "${cam_domain}" ] && [ "${cam_domain}" != "${DOMAIN}" ]; then
    warn "ROS 2 のドメインが違います（実行系 ${DOMAIN}、カメラ ${cam_domain}）. そろえる: bash jetson/robot/set_domain.sh <番号>"
  fi
else
  warn "カメラ（usb-camera）のサービスが入っていません（bash jetson/camera/install.sh）"
fi

# ---- 4. 実行系 ---------------------------------------------------------------------------
if "${DOCKER[@]}" ps -a --format '{{.Names}}' | grep -qx elevator-task; then
  "${DOCKER[@]}" rm -f elevator-task > /dev/null
  sleep 1
fi
(exec 3<> /dev/tcp/127.0.0.1/8080) 2> /dev/null && fail "ポート 8080 を別のプログラムが使っています（ベンチなら bash jetson/bench/bench_down.sh）"
mkdir -p "${ROOT}/elevator_vlm/runs"
"${DOCKER[@]}" run -d --name elevator-task --restart unless-stopped --network host --ipc host \
  -e ROS_DOMAIN_ID="${DOMAIN}" \
  -e VLM_BASE_URL="http://127.0.0.1:${VLM_PORT}/v1" -e VLM_MODEL=vlm \
  -v "${CONFIG}:/config/task.yaml:ro" -v "${ROOT}/elevator_vlm/runs:/runs" \
  "${RUNTIME_IMAGE}" ros2 launch elevator_task task.launch.py config_file:=/config/task.yaml \
  use_sim_time:=false cmd_vel_topic:="${RELAY_TOPIC}" odom_topic:="${ODOM_TOPIC}" \
  log_dir:=/runs web_port:=8080 > /dev/null

say "実行系の準備ができるまで待ちます（カメラ、オドメトリ、VLM）"
status=""
for _ in $(seq 1 45); do
  status="$(curl -sf --noproxy '*' --max-time 3 http://127.0.0.1:8080/api/state 2> /dev/null \
    | python3 -c 'import json, sys; s = json.load(sys.stdin); print("ok" if not s["robot_status"] and s["vlm"]["ok"] else "、".join(s["robot_status"]) or s["vlm"]["info"])' 2> /dev/null || true)"
  [ "${status}" = "ok" ] && break
  if ! "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx elevator-task; then
    "${DOCKER[@]}" logs --tail 40 elevator-task 2>&1 | sed 's/^/  /'
    fail "実行系（elevator-task）が止まりました"
  fi
  sleep 2
done
[ "${status}" = "ok" ] || warn "まだ準備ができていません（${status:-Web UI に接続できません}）. 確かめる: bash jetson/robot/check_link.sh"

ip="$(hostname -I 2> /dev/null | awk '{print $1}')"
say ""
say "OK: 実行系を起動しました"
say "  ROS 2 のドメイン:     ${DOMAIN}"
say "  オドメトリ:           ${ODOM_TOPIC}（${ROBOT_ENV} の ODOM_TOPIC）"
say "  カメラの取り付けなど: ${CONFIG}"
say "  ⚠ 最終の速度指令:    ${cmd_out:-（安全ゲートの設定が読めません）}（${GATE_CONF} の topics.cmd_out. ロボットが受けるトピックと同じ名前か）"
say "  実行系の WebUI:       http://${ip:-<OrinのIP>}:8080/"
say "  安全ゲートの WebUI:   http://${ip:-<OrinのIP>}:9001/"
say "VLM で走らせる（Orin から）:"
say "  bash jetson/robot/drive.sh load path_10m.json"
say "  bash jetson/robot/drive.sh arm"
say "  bash jetson/robot/drive.sh start      （ROS1システムからなら rostopic pub -1 /vlm_drive/command std_msgs/String \"data: 'start'\"）"
say "  bash jetson/robot/drive.sh finish"
say "止めるとき: bash jetson/robot/robot_down.sh"
