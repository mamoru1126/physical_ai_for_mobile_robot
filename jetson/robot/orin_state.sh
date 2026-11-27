#!/bin/bash
# この Orin の今の状態を出す（何が入っていて、何が自動で起動し、設定がどうなっているか）. 何も変えない.
#   bash jetson/robot/orin_state.sh           画面に出す
#   bash jetson/robot/orin_state.sh --save    jetson/log/orin_state_<日時>.md にも残す（受け渡す前などに）
# IP アドレスやトークンなど、ネットワークの中の情報は書かない.
set -uo pipefail
source "$(dirname "$0")/lib.sh"
cd "${ROOT}"

if [ "${1:-}" = "--save" ] && [ -z "${ORIN_STATE_LOG:-}" ]; then
  mkdir -p "${ROOT}/jetson/log"
  export ORIN_STATE_LOG="${ROOT}/jetson/log/orin_state_$(date +%Y%m%d_%H%M%S).md"
  bash "${ROBOT_DIR}/orin_state.sh" > "${ORIN_STATE_LOG}" 2>&1
  cat "${ORIN_STATE_LOG}"
  say ""
  say "記録しました: ${ORIN_STATE_LOG#"${ROOT}"/}"
  exit 0
fi

block() {
  local body
  body="$(cat)"
  printf '\n```text\n%s\n```\n' "${body:-（なし）}"
}
first_line() { head -n 1 | sed 's/[[:space:]]*$//'; }

say "# Orin の状態（$(date '+%Y-%m-%d %H:%M')）"
say ""
say "## 基本"
say ""
say "| 項目 | 値 |"
say "| --- | --- |"
say "| OS | $( (. /etc/os-release 2> /dev/null && printf '%s' "${PRETTY_NAME:-?}") ) |"
say "| L4T（JetPack） | $(cat /etc/nv_tegra_release 2> /dev/null | first_line | sed 's/^# //')$(dpkg-query -W -f=' / nvidia-jetpack ${Version}' nvidia-jetpack 2> /dev/null) |"
say "| 電源モード | $( (nvpmodel -q 2> /dev/null || sudo -n nvpmodel -q 2> /dev/null) | grep -m 1 'Power Mode' | sed 's/.*Power Mode: *//') |"
say "| メモリ | $(free -h 2> /dev/null | awk '/^Mem:/ {print $2 "（空き " $7 "）"}') |"
say "| ディスク（/） | $(df -h / 2> /dev/null | awk 'NR == 2 {print $2 "（空き " $4 "）"}') |"
say "| Docker | $("${DOCKER[@]}" version --format '{{.Server.Version}}' 2> /dev/null) |"
say "| リポジトリ | ${ROOT/#"${HOME}"/\~}（$(git -C "${ROOT}" rev-parse --abbrev-ref HEAD 2> /dev/null) $(git -C "${ROOT}" log -1 --format='%h %ad' --date=short 2> /dev/null)） |"
changed="$(git -C "${ROOT}" status --short 2> /dev/null | grep -v '^?? jetson/log/' | head -n 20)"
[ -n "${changed}" ] && { say ""; say "リポジトリの、コミットしていない変更:"; printf '%s\n' "${changed}" | block; }

say ""
say "## 自動で起動するもの"
say ""
say "| 名前 | 種類 | 電源を入れると | 今 | 中身 |"
say "| --- | --- | --- | --- | --- |"
for svc in usb-camera safety-gate ros1-bridge; do
  systemctl cat "${svc}.service" > /dev/null 2>&1 || continue
  case "${svc}" in
    usb-camera) what="カメラ（WebUI 9000）" ;;
    safety-gate) what="点群を見る安全ゲート（WebUI 9001）" ;;
    *) what="ROS 1 との橋渡し" ;;
  esac
  say "| ${svc} | サービス（systemd） | $(systemctl is-enabled "${svc}" 2> /dev/null) | $(systemctl is-active "${svc}" 2> /dev/null) | ${what} |"
done
while IFS=$'\t' read -r name image status; do
  [ -n "${name}" ] || continue
  policy="$("${DOCKER[@]}" inspect -f '{{.HostConfig.RestartPolicy.Name}}' "${name}" 2> /dev/null)"
  case "${policy}" in
    unless-stopped|always) boot="起動する（${policy}）" ;;
    *) boot="起動しない" ;;
  esac
  say "| ${name} | コンテナ（${image}） | ${boot} | ${status} | |"
done < <("${DOCKER[@]}" ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}' 2> /dev/null | grep -v '^usb-camera\b\|^safety-gate\b')
say ""
say "usb-camera と safety-gate はサービスが起動するコンテナなので、上の表のサービスの行で見る."

say ""
say "## イメージ"
"${DOCKER[@]}" images --format '{{.Repository}}:{{.Tag}}\t{{.Size}}\t{{.CreatedSince}}' 2> /dev/null \
  | grep -E '^(elevator-runtime|safety-gate|usb-camera|vllm/|ros1-bridge)' | block
stamp="$(image_stamp)"
if [ -z "${stamp}" ] || [ "${stamp}" = "<no value>" ]; then
  say "実行系のイメージ（${RUNTIME_IMAGE}）: いつのリポジトリから作ったか分からない（ROBOT_REBUILD=1 で作り直すと分かる）"
elif [ "${stamp}" = "$(source_stamp)" ]; then
  say "実行系のイメージ（${RUNTIME_IMAGE}）: リポジトリの今の中身から作ったもの"
else
  say "実行系のイメージ（${RUNTIME_IMAGE}）: リポジトリより古い（ROBOT_REBUILD=1 bash jetson/robot/robot_up.sh で作り直す. ネットはなくてよい）"
fi

say ""
say "## VLM の重み（~/.cache/huggingface）"
ls -d "${HOME}"/.cache/huggingface/hub/models--* 2> /dev/null | while read -r d; do
  printf '%s\t%s\n' "$(basename "${d}" | sed 's/^models--//; s/--/\//')" "$(du -sh "${d}" 2> /dev/null | cut -f1)"
done | block

say ""
say "## USB カメラ"
{ ls -l /dev/v4l/by-id/ 2> /dev/null | sed -n 's/.* \([^ ]*\) -> .*\/\(video[0-9]*\)$/\1 → \/dev\/\2/p'; ls /dev/video* 2> /dev/null; } | block

say ""
say "## 設定"
say ""
say "### ⚠ 最終の速度指令のトピックと、ロボットの足形（${GATE_CONF}）"
{
  printf 'topics.cmd_out（最終の速度指令）: %s\n' "$(yaml_get "${GATE_CONF}" topics.cmd_out)"
  printf 'topics.cmd_in（実行系からの入力）: %s\n' "$(yaml_get "${GATE_CONF}" topics.cmd_in)"
  printf 'topics.cloud（点群）: %s\n' "$(yaml_get "${GATE_CONF}" topics.cloud)"
  printf 'frames.base / frames.cloud: %s / %s\n' "$(yaml_get "${GATE_CONF}" frames.base)" "$(yaml_get "${GATE_CONF}" frames.cloud)"
  printf 'tf.publish_static: %s\n' "$(yaml_get "${GATE_CONF}" tf.publish_static)"
  printf 'safety.publish_when_idle: %s\n' "$(yaml_get "${GATE_CONF}" safety.publish_when_idle)"
  printf 'ros_domain_id: %s\n' "$(yaml_get "${GATE_CONF}" ros_domain_id)"
  yaml_block "${GATE_CONF}" footprint
} | block
gate_default="${ROOT}/safety_gate/ros2_ws/src/safety_gate/config/config.yaml"
if [ -n "$(read_file "${GATE_CONF}")" ]; then
  d="$(diff <(read_file "${GATE_CONF}") "${gate_default}" 2> /dev/null)"
  if [ -z "${d}" ]; then
    say "リポジトリの既定（safety_gate/ros2_ws/src/safety_gate/config/config.yaml）と同じ."
  else
    say "リポジトリの既定との違い（< がこの Orin、> が既定）:"
    printf '%s\n' "${d}" | block
  fi
else
  say "（${GATE_CONF} がありません）"
fi

for pair in "${ROBOT_ENV}:${ROBOT_DIR}/robot.env" "${TASK_CONF}:${ROBOT_DIR}/task_robot.yaml" \
            "${CAMERA_ENV}:${ROOT}/jetson/camera/camera.env"; do
  file="${pair%%:*}"
  tmpl="${pair#*:}"
  say ""
  say "### ${file}"
  if [ -z "$(read_file "${file}")" ]; then
    say ""
    say "（ありません）"
    continue
  fi
  case "${file}" in
    *.env) read_file "${file}" | grep -Ev '^[[:space:]]*(#|$)' | block ;;
    *) yaml_block "${file}" camera | grep -Ev '^[[:space:]]*#' | block ;;
  esac
  d="$(diff <(read_file "${file}") "${tmpl}" 2> /dev/null)"
  if [ -z "${d}" ]; then
    say "雛形（${tmpl#"${ROOT}"/}）と同じ."
  else
    say "雛形（${tmpl#"${ROOT}"/}）との違い（< がこの Orin、> が雛形）:"
    printf '%s\n' "${d}" | block
  fi
done

say ""
say "## 待ち受けているポート"
{
  for p in 8000:VLM 8080:実行系のWebUI 9000:カメラのWebUI 9001:安全ゲートのWebUI; do
    if (exec 3<> "/dev/tcp/127.0.0.1/${p%%:*}") 2> /dev/null; then
      printf '%s %s: 待ち受けている\n' "${p%%:*}" "${p#*:}"
    else
      printf '%s %s: いない\n' "${p%%:*}" "${p#*:}"
    fi
  done
} | block
