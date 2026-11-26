#!/bin/bash
# コンテナの入口. カメラを探し（つながっていなければ、つながるまで待つ）、カメラと配信を起動する.
set -eo pipefail
source /opt/ros/humble/setup.bash

find_camera() {
  local dev
  # USB のカメラは /dev/v4l/by-id/ に名前で出る. 映像の本体は video-index0 の方
  for dev in /dev/v4l/by-id/*-video-index0; do
    [ -e "${dev}" ] && { readlink -f "${dev}"; return 0; }
  done
  for dev in /dev/video*; do
    [ -e "${dev}" ] && { echo "${dev}"; return 0; }
  done
  return 1
}

want="${CAMERA_DEVICE:-auto}"
waited=0
while true; do
  if [ "${want}" = "auto" ]; then
    device="$(find_camera || true)"
  else
    device="$( [ -e "${want}" ] && readlink -f "${want}" || true)"
  fi
  [ -n "${device}" ] && break
  if [ $((waited % 30)) -eq 0 ]; then
    echo "カメラがつながるのを待っています（CAMERA_DEVICE=${want}）"
  fi
  sleep 2
  waited=$((waited + 2))
done

echo "カメラ: ${device}"
v4l2-ctl --device "${device}" --list-formats-ext 2> /dev/null | head -n 40 || true
echo "トピック: /${CAMERA_NAMESPACE:-usb_cam}/image_raw（圧縮: /${CAMERA_NAMESPACE:-usb_cam}/image_raw/compressed）"
echo "ブラウザ: http://<このPCのIP>:${CAMERA_WEB_PORT:-9000}/"

exec ros2 launch /opt/usb_camera/camera.launch.py \
  video_device:="${device}" \
  image_width:="${CAMERA_WIDTH:-640}" \
  image_height:="${CAMERA_HEIGHT:-480}" \
  framerate:="${CAMERA_FPS:-30.0}" \
  pixel_format:="${CAMERA_PIXEL_FORMAT:-mjpeg2rgb}" \
  namespace:="${CAMERA_NAMESPACE:-usb_cam}" \
  frame_id:="${CAMERA_FRAME_ID:-usb_cam}" \
  camera_info_url:="${CAMERA_INFO_URL:-}" \
  port:="${CAMERA_WEB_PORT:-9000}"
