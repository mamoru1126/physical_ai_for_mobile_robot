#!/bin/bash
# 実機（ロボットに載せた Orin）で、VLM の指示で走る実行系を起動する. ネットにつながっていなくてよい.
#   bash jetson/robot/robot_up.sh              VLM が動いていなければ、既定のモデルで起動する
#   bash jetson/robot/robot_up.sh <モデル>      VLM を、そのモデルで起動する（動いていないとき）
#
# 起動するもの:
#   elevator-vlm    VLM のサーバ（jetson/vlm_up.sh. ポート 8000）
#   elevator-task   実行系（Web UI はポート 8080）. 設定は jetson/robot/task_robot.yaml
# 前もって入れておくもの（ネットがあるうちに）:
#   カメラ        bash jetson/camera/install.sh（サービス usb-camera）
#   安全ゲート    bash safety_gate/install.sh（サービス safety-gate. 点群を見る. WebUI はポート 9001）
#   橋渡し        ロボットの PC に ros1_bridge/（README のとおり）
#   イメージ      このスクリプトを一度ネットのあるところで実行すると、実行系のイメージを作る
#   VLM の重み    bash jetson/vlm_up.sh を一度ネットのあるところで実行すると、落としておける
#
# 環境変数:
#   ROBOT_DOMAIN_ID   ROS 2 のドメイン（既定 42）
#   ROBOT_CONFIG      実行系の設定（既定 jetson/robot/task_robot.yaml）
#   ROBOT_REBUILD=1   実行系のイメージを作り直す（リポジトリを更新したとき. ネットが要る）
set -euo pipefail
cd "$(dirname "$0")/../.."
ROOT="$(pwd)"
DOMAIN="${ROBOT_DOMAIN_ID:-42}"
CONFIG="${ROBOT_CONFIG:-${ROOT}/jetson/robot/task_robot.yaml}"
RUNTIME_IMAGE="elevator-runtime:latest"
VLM_PORT="${VLM_PORT:-8000}"

say() { printf '%s\n' "$*"; }
warn() { printf '注意: %s\n' "$*"; }
fail() { printf 'NG: %s\n' "$*" >&2; exit 1; }

command -v docker > /dev/null 2>&1 || fail "Docker がありません（jetson/README.md の「Docker」）"
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
# プロキシの下では、イメージを作るコンテナ（apt-get、pip）にもプロキシの設定を渡す.
# プロキシがこの PC の上（127.0.0.1 など）にあっても届くよう、作るときもホストのネットワークを使う
BUILD_ARGS=(--network host)
for var in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do
  [ -n "${!var:-}" ] && BUILD_ARGS+=(--build-arg "${var}=${!var}")
done
[ -f "${CONFIG}" ] || fail "設定がありません: ${CONFIG}"
online=0
curl -sf --max-time 5 -o /dev/null https://huggingface.co 2> /dev/null && online=1

# ---- 1. 実行系のイメージ ----------------------------------------------------------------
if [ "${ROBOT_REBUILD:-0}" = "1" ] || ! "${DOCKER[@]}" image inspect "${RUNTIME_IMAGE}" > /dev/null 2>&1; then
  [ "${online}" = "1" ] || fail "実行系のイメージ（${RUNTIME_IMAGE}）がなく、ネットもありません. ネットのあるところで一度実行してください"
  say "イメージを作ります: ${RUNTIME_IMAGE}（初回は数分〜十数分. 中で単体テストも通します）"
  "${DOCKER[@]}" build "${BUILD_ARGS[@]}" -t "${RUNTIME_IMAGE}" -f "${ROOT}/elevator_vlm/docker/Dockerfile.runtime" "${ROOT}/elevator_vlm"
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
    if [ "${online}" = "1" ]; then
      say "VLM のサーバを起動します（jetson/vlm_up.sh）"
      bash "${ROOT}/jetson/vlm_up.sh" "$@"
    else
      say "VLM のサーバを、ネットなしで起動します（落としてある重みを使う）"
      VLM_OFFLINE=1 bash "${ROOT}/jetson/vlm_up.sh" "$@"
    fi
  fi
fi

# ---- 3. 安全ゲート（点群を見るもの）を確かめる ------------------------------------------------
# パイプで grep -q に渡すと、pipefail の下では systemctl が SIGPIPE で終わり「入っていない」と誤る
if systemctl cat safety-gate.service > /dev/null 2>&1; then
  systemctl is-active --quiet safety-gate || warn "安全ゲート（safety-gate）が動いていません: sudo systemctl start safety-gate"
  gate_conf=/etc/safety-gate/config.yaml
  if ! sudo -n grep -Eq '^[[:space:]]*publish_when_idle:[[:space:]]*false' "${gate_conf}" 2> /dev/null \
     && ! grep -Eq '^[[:space:]]*publish_when_idle:[[:space:]]*false' "${gate_conf}" 2> /dev/null; then
    warn "安全ゲートの設定（${gate_conf}）に publish_when_idle: false がありません."
    warn "  このままだと、VLM が走らせていない間も安全ゲートが 0 を出し続け、元のシステムの指令とぶつかります."
    warn "  safety: の下に「publish_when_idle: false」を足して、sudo systemctl restart safety-gate"
  fi
else
  warn "安全ゲート（safety-gate）のサービスが入っていません. 点群を見ないまま走ることになります（bash safety_gate/install.sh）"
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
  use_sim_time:=false cmd_vel_topic:=/vlm_cmd_vel log_dir:=/runs web_port:=8080 > /dev/null

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
[ "${status}" = "ok" ] || warn "まだ準備ができていません（${status:-Web UI に接続できません}）. カメラのサービスと、ロボットの PC の橋渡し（/odom）を確かめてください"

ip="$(hostname -I 2> /dev/null | awk '{print $1}')"
say ""
say "OK: 実行系を起動しました（ROS 2 のドメイン ${DOMAIN}）"
say "  実行系の WebUI:   http://${ip:-<OrinのIP>}:8080/"
say "  安全ゲートの WebUI: http://${ip:-<OrinのIP>}:9001/"
say "VLM の指示で走る区間（途中から・ここまで）:"
say "  1. 画面で計画を作る（例「道に沿って走って」）か、用意済みの計画を読み込む"
say "  2. 「承認して開始の合図を待つ」を押す（すぐ走らせるなら「承認して実行」）"
say "  3. 元のシステム（ROS 1）から: rostopic pub -1 /vlm_drive/command std_msgs/String \"data: 'start'\""
say "     終えるとき:              rostopic pub -1 /vlm_drive/command std_msgs/String \"data: 'finish'\""
say "     走っているか:            rostopic echo /vlm_drive/active"
say "止めるとき: bash jetson/robot/robot_down.sh"
