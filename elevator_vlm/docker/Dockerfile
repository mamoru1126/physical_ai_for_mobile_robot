# syntax=docker/dockerfile:1
# =============================================================================
# エレベーター課題のシミュレーション環境
#   Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic (gz-sim 8)
# 土台は omnivla_gazebo の Dockerfile と同じ（PyTorch と OmniVLA は入れない）.
# VLM は別のコンテナ（vLLM）で動かし、HTTP でつなぐ. このイメージに GPU 用のライブラリは要らない.
# GPU ドライバはホストのものを NVIDIA Container Toolkit 経由で使う（Gazebo のカメラの描画に使う）.
# =============================================================================
ARG BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1 \
    ELEVATOR_VLM_ROOT=/workspace \
    GZ_IP=127.0.0.1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---- 基本の道具 -----------------------------------------------------------------
# xvfb と Mesa は、GPU のない計算機（CI など）でカメラを描画するときに使う
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates \
        python3-pip python3-colcon-common-extensions \
        python3-numpy python3-pil python3-yaml python3-pytest fonts-dejavu-core \
        libgl1 libegl1 libglib2.0-0 libgl1-mesa-dri libglx-mesa0 libegl-mesa0 mesa-utils \
        xvfb xauth x11-apps less nano \
    && rm -rf /var/lib/apt/lists/*

# ---- Gazebo Harmonic + ros_gz（Humble 向けのバイナリ. packages.osrfoundation.org）----
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-rqt-image-view \
        ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# ---- 本リポジトリ ----------------------------------------------------------------
# 実行時は docker compose で /workspace にマウントする（イメージ単体でも動くようにコピーもしておく）
COPY . /workspace
WORKDIR /workspace
RUN source /opt/ros/humble/setup.bash \
    && mkdir -p /opt/elevator_ws \
    && colcon --log-base /opt/elevator_ws/log build --symlink-install \
        --base-paths /workspace/ros2_ws/src \
        --build-base /opt/elevator_ws/build --install-base /opt/elevator_ws/install

# ---- 動作確認 --------------------------------------------------------------------
# 1. ROS に依存しない部分の単体テストと、擬似世界での通し試験
# 2. ノードが import できるか、launch ファイルの引数が解釈できるか
RUN source /opt/ros/humble/setup.bash && source /opt/elevator_ws/install/setup.bash \
    && python3 -m pytest -q /workspace/ros2_ws/src/elevator_task/test \
    && python3 -m pytest -q /workspace/ros2_ws/src/elevator_sim/test \
    && python3 -c "import elevator_task.task_node, elevator_task.safety_gate_node, elevator_sim.elevator_controller_node, elevator_sim.mock_vlm_node, elevator_sim.e2e_runner; print('nodes import ok')" \
    && ros2 launch elevator_sim demo.launch.py --show-args > /dev/null \
    && ros2 launch elevator_task task.launch.py --show-args > /dev/null \
    && gz sim --versions

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
