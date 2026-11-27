# jetson/robot/ のスクリプトが共通に使うもの（source して使う. これだけを実行はしない）.
#
# 実機の設定は、この Orin の /etc の下の 3 つのフォルダにある（リポジトリを更新しても上書きされない）:
#   /etc/elevator-task/robot.env    実行系: ROS 2 のドメイン、オドメトリのトピック
#   /etc/elevator-task/task.yaml    実行系: カメラの取り付け、走る速さ、合図のトピックなど
#   /etc/safety-gate/config.yaml    安全ゲート: 最終の速度指令のトピック、ロボットの足形、点群のトピックなど
#   /etc/usb-camera/camera.env      カメラ: 機器、解像度、トピック
# 前の 2 つは、robot_up.sh を初めて実行したときに jetson/robot/ の雛形から作る.

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${ROBOT_DIR}/../.." && pwd)"
RUNTIME_IMAGE="elevator-runtime:latest"
TASK_ETC=/etc/elevator-task
ROBOT_ENV="${TASK_ETC}/robot.env"
TASK_CONF="${TASK_ETC}/task.yaml"
GATE_CONF="${GATE_CONFIG:-/etc/safety-gate/config.yaml}"
CAMERA_ENV=/etc/usb-camera/camera.env
# 実行系から安全ゲートへの中継のトピック. 安全ゲートの topics.cmd_in と同じでなければならない
RELAY_TOPIC=/vlm_cmd_vel

say() { printf '%s\n' "$*"; }
warn() { printf '注意: %s\n' "$*"; }
fail() { printf 'NG: %s\n' "$*" >&2; exit 1; }

if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi

# ファイルを読む（root しか読めなくても、sudo で読めれば読む）
read_file() {
  cat "$1" 2> /dev/null || sudo -n cat "$1" 2> /dev/null || true
}

# KEY=VALUE の形のファイルから値を取る: env_get <ファイル> <KEY>
env_get() {
  read_file "$1" | sed -n "s/^[[:space:]]*$2=\(.*\)$/\1/p" | tail -n 1 \
    | sed -e 's/[[:space:]]*#.*$//' -e 's/^"\(.*\)"$/\1/' -e "s/^'\(.*\)'$/\1/"
}

# 2 段までの YAML から値を取る: yaml_get <ファイル> <キー> か <段.キー>（例 topics.cmd_out）
yaml_get() {
  read_file "$1" | awk -v want="$2" '
    /^[[:space:]]*#/ { next }
    {
      line = $0
      sub(/[[:space:]]+#.*$/, "", line)
      if (match(line, /^[A-Za-z_][A-Za-z0-9_]*:/)) {
        key = substr(line, 1, RLENGTH - 1); val = substr(line, RLENGTH + 1); sect = key
      } else if (match(line, /^  [A-Za-z_][A-Za-z0-9_]*:/)) {
        key = sect "." substr(line, 3, RLENGTH - 3); val = substr(line, RLENGTH + 1)
      } else {
        next
      }
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", val)
      gsub(/^"|"$/, "", val)
      if (key == want) { print val; exit }
    }'
}

# YAML の 1 つの段（例 footprint）を、そのまま取り出す: yaml_block <ファイル> <段>
yaml_block() {
  read_file "$1" | awk -v want="$2" '
    $0 ~ "^" want ":" { on = 1; print; next }
    on && !/^[[:space:]]+[^[:space:]]/ { exit }
    on { print }'
}

# ROS 2 のコンテナに渡す環境（ROS_LOCALHOST_ONLY は、ベンチで試すときだけ外から渡す）
ros_env_args() {
  printf '%s\n' -e "ROS_DOMAIN_ID=$(robot_domain)"
  [ -n "${ROS_LOCALHOST_ONLY:-}" ] && printf '%s\n' -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  return 0
}

# 実行系の設定（環境変数があれば、そちらを使う）
robot_domain() {
  local v="${ROBOT_DOMAIN_ID:-}"
  [ -n "${v}" ] || v="$(env_get "${ROBOT_ENV}" ROS_DOMAIN_ID)"
  printf '%s\n' "${v:-42}"
}
robot_odom_topic() {
  local v="${ROBOT_ODOM_TOPIC:-}"
  [ -n "${v}" ] || v="$(env_get "${ROBOT_ENV}" ODOM_TOPIC)"
  printf '%s\n' "${v:-/odom}"
}

# /etc/elevator-task/ がなければ、雛形から作る
ensure_task_etc() {
  if [ ! -f "${ROBOT_ENV}" ] || [ ! -f "${TASK_CONF}" ]; then
    sudo mkdir -p "${TASK_ETC}"
    if [ ! -f "${ROBOT_ENV}" ]; then
      sudo cp "${ROBOT_DIR}/robot.env" "${ROBOT_ENV}"
      say "実行系の設定を作りました: ${ROBOT_ENV}（雛形は jetson/robot/robot.env）"
    fi
    if [ ! -f "${TASK_CONF}" ]; then
      sudo cp "${ROBOT_DIR}/task_robot.yaml" "${TASK_CONF}"
      say "実行系の設定を作りました: ${TASK_CONF}（雛形は jetson/robot/task_robot.yaml）"
    fi
  fi
}

# 実行系のイメージが、リポジトリの今の中身から作ったものか（作るときに付けた印と比べる）
source_stamp() {
  git -C "${ROOT}" rev-parse HEAD:elevator_vlm 2> /dev/null || echo unknown
}
image_stamp() {
  "${DOCKER[@]}" image inspect -f '{{ index .Config.Labels "elevator.src" }}' "${RUNTIME_IMAGE}" 2> /dev/null || true
}
