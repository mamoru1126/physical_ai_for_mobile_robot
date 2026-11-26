# 安全ゲートのコンテナ（ROS 2 Humble）. GPU は使わない. x86 でも Orin（arm64）でも作れる.
# 作るときに単体テストも通す（通らなければ、ここで止まる）.
ARG BASE=ros:humble-ros-base-jammy
FROM ${BASE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        libyaml-cpp-dev \
        python3-yaml \
        ros-humble-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src /opt/safety_gate_ws/src
RUN source /opt/ros/humble/setup.bash \
    && cd /opt/safety_gate_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ \
    && colcon test --packages-select safety_gate --event-handlers console_direct+ \
    && colcon test-result --verbose \
    && rm -rf build log

COPY entrypoint.sh selftest.py /opt/safety_gate/
RUN chmod +x /opt/safety_gate/entrypoint.sh
ENTRYPOINT ["/opt/safety_gate/entrypoint.sh"]
