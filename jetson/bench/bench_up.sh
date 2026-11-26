#!/bin/bash
# Orin のベンチを起動する. 実機（ロボットとエレベーター）の代わりにダミーの世界をつなぎ、
# 本物の VLM・実行系・安全ゲートでエレベーターのタスクを動かせるようにする.
#   bash jetson/bench/bench_up.sh              VLM が動いていなければ、既定のモデルで起動する
#   bash jetson/bench/bench_up.sh <モデル>      VLM を、そのモデルで起動する（動いていないとき）
#
# 起動するもの（どれも Docker のコンテナ. 電源を入れ直しても自動では起動しない）:
#   elevator-vlm         VLM のサーバ（jetson/vlm_up.sh と同じもの. ポート 8000）
#   elevator-bench-gate  安全ゲート（点群を見るもの. safety_gate/. WebUI はポート 9001）
#   elevator-bench       ダミーの世界と実行系（WebUI はポート 8080）
#
# ベンチの ROS 2 は、ドメインを本番（42）と分け、この PC の中だけで通信する. 同じネットワークに
# ロボットの橋渡しがあっても、ベンチの /cmd_vel がロボットに届かないようにするため.
#
# 環境変数（指定したときだけ使う）:
#   BENCH_DOOR_HOLD_S    ダミーのエレベーターが扉を全開で保持する秒数（既定 15）
#   BENCH_DOMAIN_ID      ベンチの ROS 2 のドメイン（既定 43）
#   BENCH_REBUILD=1      実行系と安全ゲートのイメージを作り直す（リポジトリを更新したとき）
#   BENCH_ALLOW_MAXN=1   電源モードが MAXN でも始める（既定は始めない）
set -euo pipefail
cd "$(dirname "$0")/../.."
ROOT="$(pwd)"
DOMAIN="${BENCH_DOMAIN_ID:-43}"
HOLD="${BENCH_DOOR_HOLD_S:-15}"
RUNTIME_IMAGE="elevator-runtime:latest"
GATE_IMAGE="safety-gate:latest"
VLM_PORT="${VLM_PORT:-8000}"

say() { printf '%s\n' "$*"; }
fail() { printf 'NG: %s\n' "$*" >&2; exit 1; }

command -v docker > /dev/null 2>&1 || fail "Docker がありません（jetson/README.md の「Docker」）"
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
[ "${DOMAIN}" = "42" ] && fail "BENCH_DOMAIN_ID を本番と同じ 42 にはできません（ロボットに指令が届くおそれがある）"

# ---- 電源モード（MAXN では始めない）-------------------------------------------------
if [ -f /etc/nv_tegra_release ]; then
  power="$(nvpmodel -q 2> /dev/null || sudo -n nvpmodel -q 2> /dev/null || true)"
  power_name="$(printf '%s\n' "${power}" | grep -m 1 'Power Mode' | sed 's/.*Power Mode: *//')"
  if [[ "${power_name}" == MAXN* ]] && [ "${BENCH_ALLOW_MAXN:-0}" != "1" ]; then
    say "NG: 電源モードが ${power_name} です. この電源では電流が足りずに落ちるので、ベンチは始めません." >&2
    say "    モードの一覧を見て（sudo nvpmodel -q --verbose）、MAXN ではないモードに切り替えてください." >&2
    say "    AGX Orin 開発キットの JetPack 7.2 なら、既定の MODE_30W はモード 2 です: sudo nvpmodel -m 2" >&2
    exit 1
  fi
  say "電源モード: ${power_name:-不明}（MAXN にはしません. jetson_clocks も使いません）"
fi

# ---- 前に起動したベンチを片づけ、ポートが空いているか確かめる ---------------------------
for name in elevator-bench elevator-bench-gate; do
  if "${DOCKER[@]}" ps -a --format '{{.Names}}' | grep -qx "${name}"; then
    "${DOCKER[@]}" rm -f "${name}" > /dev/null
  fi
done
sleep 1
for port in 8080 9001; do
  if (exec 3<> "/dev/tcp/127.0.0.1/${port}") 2> /dev/null; then
    hint=""
    [ "${port}" = "9001" ] && hint=" 実機用の安全ゲートのサービスなら: sudo systemctl stop safety-gate"
    fail "ポート ${port} を別のプログラムが使っています. 止めてから、もう一度実行してください.${hint}"
  fi
done

# ---- 1. イメージ（なければ作る. ネットが要る）---------------------------------------------
build() {
  local image="$1" context="$2"
  shift 2
  if [ "${BENCH_REBUILD:-0}" = "1" ] || ! "${DOCKER[@]}" image inspect "${image}" > /dev/null 2>&1; then
    say "イメージを作ります: ${image}（初回は数分〜十数分. 中で単体テストも通します）"
    "${DOCKER[@]}" build -t "${image}" "$@" "${context}"
  fi
}
build "${RUNTIME_IMAGE}" "${ROOT}/elevator_vlm" -f "${ROOT}/elevator_vlm/docker/Dockerfile.runtime"
build "${GATE_IMAGE}" "${ROOT}/safety_gate"

# ---- 2. VLM のサーバ --------------------------------------------------------------------
vlm_ready() { curl -sf --noproxy '*' --max-time 3 "http://127.0.0.1:${VLM_PORT}/v1/models" > /dev/null 2>&1; }
vlm_listening() { (exec 3<> "/dev/tcp/127.0.0.1/${VLM_PORT}") 2> /dev/null; }
if ! vlm_ready; then
  if vlm_listening && ! "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx elevator-vlm; then
    # 別の VLM のサーバ（モックなど）がポートで待ち受けている. 準備ができたかは、実行系の画面で確かめる
    say "ポート ${VLM_PORT} の VLM のサーバを使います（まだ準備中. このまま進みます）"
  elif "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx elevator-vlm; then
    say "VLM のサーバが起動の途中です. 答えられるようになるまで待ちます"
    for _ in $(seq 1 480); do
      vlm_ready && break
      sleep 5
    done
    vlm_ready || fail "VLM のサーバが答えません（ログ: ${DOCKER[*]} logs elevator-vlm）"
  else
    say "VLM のサーバを起動します（jetson/vlm_up.sh）"
    bash "${ROOT}/jetson/vlm_up.sh" "$@"
  fi
fi
model="$(curl -sf --noproxy '*' --max-time 5 "http://127.0.0.1:${VLM_PORT}/v1/models" \
  | python3 -c 'import json, sys; d = json.load(sys.stdin)["data"][0]; print(d.get("root") or d["id"])' 2> /dev/null || echo 不明)"
say "VLM: ${model}"

# ---- 3. 安全ゲート（点群を見るもの）--------------------------------------------------------
mkdir -p "${ROOT}/elevator_vlm/runs/bench_gate"
sed "s/^ros_domain_id: .*/ros_domain_id: ${DOMAIN}/" "${ROOT}/jetson/bench/safety_gate/config.yaml" \
  > "${ROOT}/elevator_vlm/runs/bench_gate/config.yaml"
"${DOCKER[@]}" run -d --name elevator-bench-gate --network host --ipc host \
  -e ROS_LOCALHOST_ONLY=1 \
  -v "${ROOT}/elevator_vlm/runs/bench_gate:/config:ro" \
  "${GATE_IMAGE}" > /dev/null

# ---- 4. ダミーの世界と実行系 ---------------------------------------------------------------
"${DOCKER[@]}" run -d --name elevator-bench --network host --ipc host \
  -e ROS_DOMAIN_ID="${DOMAIN}" -e ROS_LOCALHOST_ONLY=1 \
  -e VLM_BASE_URL="http://127.0.0.1:${VLM_PORT}/v1" -e VLM_MODEL=vlm \
  -v "${ROOT}/elevator_vlm/runs:/runs" \
  "${RUNTIME_IMAGE}" ros2 launch elevator_sim bench.launch.py door_hold_s:="${HOLD}" > /dev/null

say "実行系の準備ができるまで待ちます（カメラ、オドメトリ、VLM）"
ready=0
for _ in $(seq 1 90); do
  status="$(curl -sf --noproxy '*' --max-time 3 http://127.0.0.1:8080/api/state 2> /dev/null \
    | python3 -c 'import json, sys; s = json.load(sys.stdin); print("ok" if not s["robot_status"] and s["vlm"]["ok"] else "、".join(s["robot_status"]) or s["vlm"]["info"])' 2> /dev/null || true)"
  if [ "${status}" = "ok" ]; then
    ready=1
    break
  fi
  for name in elevator-bench elevator-bench-gate; do
    if ! "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx "${name}"; then
      say "NG: ${name} が止まりました. ログの最後:"
      "${DOCKER[@]}" logs --tail 40 "${name}" 2>&1 | sed 's/^/  /'
      exit 1
    fi
  done
  sleep 2
done
if [ "${ready}" != "1" ]; then
  say "注意: まだ準備ができていません（${status:-Web UI に接続できません}）. ログ: ${DOCKER[*]} logs elevator-bench"
fi

ip="$(hostname -I 2> /dev/null | awk '{print $1}')"
say ""
say "OK: ベンチを起動しました"
say "  実行系の WebUI:   http://${ip:-<OrinのIP>}:8080/"
say "  安全ゲートの WebUI: http://${ip:-<OrinのIP>}:9001/"
say "画面で通すとき: 「計画を作る」→「承認して実行」→「エレベーターを呼ぶ」の順に押す"
say "自動で通すとき: bash jetson/bench/bench_e2e.sh"
say "止めるとき:     bash jetson/bench/bench_down.sh"
