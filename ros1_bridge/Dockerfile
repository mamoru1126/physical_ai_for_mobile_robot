# ROS 1 と ROS 2 の橋渡し（ros1_bridge）のコンテナ. ROS 2 は Humble（ほかのコンテナと同じ）.
#
# Humble の Ubuntu 22.04 には ROS 1 の公式のパッケージがない. そこで、Ubuntu 自身が配っている
# ROS 1 のパッケージ（ros-core-dev など. 中身は Noetic と同じ世代で、Noetic とそのままやりとりできる）
# を入れ、ros1_bridge をこの中で作る. やり方は ROS 2 の公式の手引き
# 「Using ros1_bridge with upstream ROS on Ubuntu 22.04」と、ros-humble-ros1-bridge-builder にならった.
# 初回は ros1_bridge を作るので、数十分かかる.
ARG BASE=ros:humble-ros-base-jammy
FROM ${BASE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG BRIDGE_REPO=https://github.com/ros2/ros1_bridge.git
ARG BRIDGE_REF=master

# 1. ROS 1 を Ubuntu のパッケージから入れる.
#    ROS 2 の apt の一覧があると、catkin がぶつかって入らないので、いったん外す
RUN mkdir -p /root/ros2-apt \
    && mv /etc/apt/sources.list.d/ros2* /root/ros2-apt/ \
    && apt-get update \
    && sed -i -e 's|^Conflicts: catkin|#Conflicts: catkin|' /var/lib/dpkg/status \
    && apt-get install -f -y \
    && cd /tmp \
    && apt-get download python3-catkin-pkg python3-rospkg python3-rosdistro \
    && dpkg --force-overwrite -i python3-catkin-pkg*.deb python3-rospkg*.deb python3-rosdistro*.deb \
    && apt-get install -f -y \
    && apt-get install -y ros-core-dev libgeometry-msgs-dev libsensor-msgs-dev libnav-msgs-dev libtf2-msgs-dev git iproute2 \
    && if [ "$(uname -m)" = "aarch64" ] && [ -d /usr/lib/x86_64-linux-gnu/pkgconfig ]; then \
         cp -n /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/; \
       fi \
    && mv /root/ros2-apt/* /etc/apt/sources.list.d/ \
    && rm -rf /var/lib/apt/lists/* /tmp/*.deb

# 1b. Ubuntu の libtf2-msgs-dev には C++ のヘッダしかなく、ros1_bridge が ROS 1 の tf2_msgs を見つけられない.
#     足りないもの（メッセージの定義、package.xml、pkg-config）だけを足す. 中身は Noetic の tf2_msgs と同じ
COPY ros1_tf2_msgs/ /tmp/ros1_tf2_msgs/
RUN echo "libtf2-msgs-dev の中身:" && dpkg -L libtf2-msgs-dev | grep -vE "^/usr/include/tf2_msgs/.+" | head -40 \
    && test -f /usr/include/tf2_msgs/TFMessage.h \
    && mkdir -p /usr/share/tf2_msgs/msg \
    && for f in TFMessage.msg TF2Error.msg; do \
         [ -f "/usr/share/tf2_msgs/msg/${f}" ] || cp "/tmp/ros1_tf2_msgs/msg/${f}" /usr/share/tf2_msgs/msg/; \
       done \
    && { [ -f /usr/share/tf2_msgs/package.xml ] || cp /tmp/ros1_tf2_msgs/package.xml /usr/share/tf2_msgs/; } \
    && pc_dir="/usr/lib/$(uname -m)-linux-gnu/pkgconfig" \
    && mkdir -p "${pc_dir}" \
    && { pkg-config --exists tf2_msgs || cp /tmp/ros1_tf2_msgs/tf2_msgs.pc "${pc_dir}/"; } \
    && pkg-config --modversion tf2_msgs \
    && ls /usr/share/tf2_msgs /usr/share/tf2_msgs/msg \
    && rm -rf /tmp/ros1_tf2_msgs

# 2. ros1_bridge を作る. メモリ 3 GB あたり 1 つずつ並べて作る（足りないと止まるため）
RUN source /opt/ros/humble/setup.bash \
    && export ROS_PACKAGE_PATH=/usr/share \
    && git clone --depth 1 -b "${BRIDGE_REF}" "${BRIDGE_REPO}" /opt/ros1_bridge_ws/src/ros1_bridge \
    && cd /opt/ros1_bridge_ws \
    && mem_gb=$(awk '/MemTotal/{print int($2/1024/1024)}' /proc/meminfo) \
    && jobs=$(( mem_gb / 3 )) \
    && if [ "${jobs}" -lt 1 ]; then jobs=1; fi \
    && if [ "${jobs}" -gt "$(nproc)" ]; then jobs=$(nproc); fi \
    && echo "ros1_bridge を ${jobs} 並列で作ります" \
    && MAKEFLAGS="-j${jobs}" colcon build --packages-select ros1_bridge \
         --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ \
    && rm -rf build log

# 3. 使うメッセージの組（TF も）が作られたかを確かめる（なければ、ここで止める）
RUN source /opt/ros/humble/setup.bash \
    && source /opt/ros1_bridge_ws/install/local_setup.bash \
    && timeout 60 ros2 run ros1_bridge dynamic_bridge --print-pairs > /tmp/pairs.txt \
    && for t in nav_msgs/msg/Odometry sensor_msgs/msg/PointCloud2 geometry_msgs/msg/Twist tf2_msgs/msg/TFMessage; do \
         grep -q "${t}" /tmp/pairs.txt || { echo "NG: ${t} を橋渡しできません"; exit 1; }; \
       done \
    && echo "OK: Odometry、PointCloud2、Twist、TFMessage を橋渡しできます"

COPY entrypoint.sh ros1_master.py /opt/ros1_bridge/
RUN chmod +x /opt/ros1_bridge/entrypoint.sh
ENTRYPOINT ["/opt/ros1_bridge/entrypoint.sh"]
