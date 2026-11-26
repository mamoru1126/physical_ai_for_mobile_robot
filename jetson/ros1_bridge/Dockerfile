# ROS 1 と ROS 2 の橋渡し（ros1_bridge）のコンテナ.
# ROS 1（Noetic）と ROS 2 を両方入れられるのは Ubuntu 20.04 だけなので、公式の
# ros:foxy-ros1-bridge（Noetic + Foxy、arm64 もある）を使う.
# Foxy と、Orin の Humble は、同じ DDS の上でふつうのメッセージをやりとりできる.
ARG BASE=ros:foxy-ros1-bridge
FROM ${BASE}

RUN apt-get update && apt-get install -y --no-install-recommends iproute2 \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh fastdds_udp.xml /opt/ros1_bridge/
RUN chmod +x /opt/ros1_bridge/entrypoint.sh
ENTRYPOINT ["/opt/ros1_bridge/entrypoint.sh"]
