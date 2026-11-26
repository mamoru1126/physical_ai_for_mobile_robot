# USB カメラの映像を ROS 2 のトピックに流し、ブラウザでも見られるようにするコンテナ.
# GPU は使わない. Orin（arm64）でも x86 の PC でも作れる.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-usb-cam \
        ros-humble-web-video-server \
        ros-humble-image-transport-plugins \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

COPY camera.launch.py entrypoint.sh /opt/usb_camera/
RUN chmod +x /opt/usb_camera/entrypoint.sh
ENTRYPOINT ["/opt/usb_camera/entrypoint.sh"]
