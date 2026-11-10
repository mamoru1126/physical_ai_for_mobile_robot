#!/bin/bash
# ROS 2 と本リポジトリの環境を読み込んでからコマンドを実行する
set -e
source /opt/ros/humble/setup.bash

export ELEVATOR_VLM_ROOT="${ELEVATOR_VLM_ROOT:-/workspace}"
WS=/opt/elevator_ws

# マウントされたソースで ROS パッケージを（再）ビルドする.
# symlink-install なので、Python・launch・世界の編集はコンテナを起動し直すだけで反映される
if [ "${ELEVATOR_SKIP_BUILD:-0}" != "1" ] && [ -d "${ELEVATOR_VLM_ROOT}/ros2_ws/src" ]; then
  mkdir -p "${WS}"
  if ! colcon --log-base "${WS}/log" build --symlink-install \
        --base-paths "${ELEVATOR_VLM_ROOT}/ros2_ws/src" \
        --build-base "${WS}/build" --install-base "${WS}/install" > "${WS}/build.log" 2>&1; then
    echo "[entrypoint] colcon build に失敗しました:" >&2
    tail -n 40 "${WS}/build.log" >&2
  fi
fi
if [ -f "${WS}/install/setup.bash" ]; then
  source "${WS}/install/setup.bash"
fi

exec "$@"
