#!/bin/bash
# 安全ゲートのサービスを止めて、自動で起動しないようにする（設定ファイルとイメージは残す）.
#   bash uninstall.sh
set -euo pipefail
NAME="safety-gate"
sudo systemctl disable --now "${NAME}" 2> /dev/null || true
sudo rm -f "/etc/systemd/system/${NAME}.service"
sudo systemctl daemon-reload
echo "サービスを外しました（設定 /etc/safety-gate/config.yaml とイメージ safety-gate:latest は残っています）"
echo "止まったあとは、安全ゲートからの /cmd_vel は出ません"
