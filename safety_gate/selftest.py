#!/usr/bin/env python3
"""安全ゲートの通しの確認（コンテナの中で動かす. CI 用）.

にせの点群（cloud_link 座標系. base_link から見て前に 0.1 m、高さ 0.5 m、後ろ向きに付いている）と
にせの指令を流し、出てくる /cmd_vel と状態、WebUI の応答を確かめる.
  python3 selftest.py <設定ファイル>
"""
import json
import math
import re
import signal
import struct
import subprocess
import sys
import time
import urllib.request

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

TF_XYZ = (0.1, 0.0, 0.5)
TF_YAW = math.pi


def make_config(src):
    with open(src, encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    config["tf"] = {"publish_static": True, "parent": "base_link", "child": "cloud_link",
                    "xyz": list(TF_XYZ), "rpy": [0.0, 0.0, TF_YAW]}
    config.setdefault("frames", {})["cloud"] = ""
    config.setdefault("web", {})["port"] = 9001
    path = "/tmp/selftest_config.yaml"
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, allow_unicode=True)
    return path, config


def to_cloud_frame(points_base):
    """base_link の点を cloud_link の点にする（p_cloud = R^T (p_base - t)）."""
    c, s = math.cos(TF_YAW), math.sin(TF_YAW)
    out = []
    for x, y, z in points_base:
        dx, dy, dz = x - TF_XYZ[0], y - TF_XYZ[1], z - TF_XYZ[2]
        out.append((c * dx + s * dy, -s * dx + c * dy, dz))
    return out


def wall_at(x, half_width=0.4):
    pts = []
    y = -half_width
    while y <= half_width + 1e-9:
        for z in (0.2, 0.5, 0.8):
            pts.append((x, y, z))
        y += 0.03
    return pts


class Tester(Node):
    def __init__(self, topics):
        super().__init__("safety_gate_selftest")
        self.cloud_pub = self.create_publisher(PointCloud2, topics["cloud"], 10)
        self.cmd_pub = self.create_publisher(Twist, topics["cmd_in"], 10)
        self.outs = []
        self.states = []
        self.create_subscription(Twist, topics["cmd_out"], lambda m: self.outs.append(
            (time.time(), m.linear.x, m.angular.z)), 10)
        self.create_subscription(String, topics["state"], lambda m: self.states.append(
            (time.time(), m.data)), 10)

    def cloud(self, points_base):
        pts = to_cloud_frame(points_base)
        msg = PointCloud2()
        msg.header.frame_id = "/cloud_link"  # ROS 1 から来ると先頭に / が付くことがある
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
        self.cloud_pub.publish(msg)

    def cmd(self, vx, wz=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_pub.publish(msg)


def run(node, seconds, cloud=None, cmd=None):
    start = time.time()
    while time.time() - start < seconds:
        if cloud is not None:
            node.cloud(cloud)
        if cmd is not None:
            node.cmd(*cmd)
        end = time.time() + 0.1
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
    since = time.time() - 0.4
    outs = [o for o in node.outs if o[0] >= since]
    states = [s[1] for s in node.states if s[0] >= since]
    return outs, states


def main():
    path, config = make_config(sys.argv[1])
    topics = config["topics"]
    launch = subprocess.Popen(["ros2", "launch", "safety_gate", "safety_gate.launch.py",
                               f"config:={path}"])
    failures = []

    def check(name, ok, detail):
        print(("OK  " if ok else "NG  ") + name + "  " + detail, flush=True)
        if not ok:
            failures.append(name)

    try:
        rclpy.init()
        node = Tester(topics)
        time.sleep(3.0)
        run(node, 1.0)

        outs, states = run(node, 1.0)
        check("指令なし → IDLE で 0", bool(outs) and all(o[1] == 0.0 for o in outs) and
              bool(states) and states[-1].startswith("IDLE"), f"{states[-1:] or '状態なし'}")

        near = wall_at(0.5)
        outs, states = run(node, 1.5, cloud=near, cmd=(0.3,))
        check("前 0.5 m に壁、前へ → STOP", bool(outs) and all(o[1] == 0.0 for o in outs) and
              states and states[-1].startswith("STOP"), f"{states[-1:]}")

        outs, states = run(node, 1.5, cloud=near, cmd=(-0.2,))
        check("前 0.5 m に壁、後ろへ → PASS", bool(outs) and abs(outs[-1][1] + 0.2) < 1e-6 and
              states[-1].startswith("PASS"), f"{states[-1:]} out={outs[-1][1] if outs else None}")

        outs, states = run(node, 1.5, cloud=wall_at(4.0), cmd=(0.3,))
        check("遠くに壁、前へ → PASS", bool(outs) and abs(outs[-1][1] - 0.3) < 1e-6 and
              states[-1].startswith("PASS"), f"{states[-1:]}")

        outs, states = run(node, 1.5, cloud=wall_at(0.9), cmd=(0.3,))
        check("前 0.9 m に壁、前へ → SLOW", bool(outs) and 0.0 < outs[-1][1] < 0.3 and
              states[-1].startswith("SLOW"), f"{states[-1:]} out={outs[-1][1] if outs else None}")

        # WebUI
        port = config["web"]["port"]
        page = urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5).read().decode()
        check("WebUI の画面", "安全ゲート" in page, f"{len(page)} 文字")
        raw = urllib.request.urlopen(f"http://127.0.0.1:{port}/latest", timeout=5).read().decode()
        data = json.loads(raw)
        long_decimals = re.findall(r"\d+\.\d{3,}", raw)
        check("WebUI の情報（小数は 2 桁まで）", "obs" in data and not long_decimals,
              f"{len(raw)} バイト、障害物のマス {len(data.get('obs', '')) * 3 // 4 // data.get('bpi', 2)}")
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/events", timeout=5) as stream:
            line = stream.readline().decode()
            while line and not line.startswith("data: "):
                line = stream.readline().decode()
        check("WebUI の流し続ける情報（/events）", line.startswith("data: {"), line[:40])

        outs, states = run(node, 1.5, cmd=(0.3,))
        check("点群が途切れる → STOP", bool(outs) and all(o[1] == 0.0 for o in outs) and
              states[-1].startswith("STOP"), f"{states[-1:]}")

        outs, states = run(node, 1.5)
        check("指令が途切れる → IDLE で 0", bool(outs) and all(o[1] == 0.0 for o in outs) and
              states[-1].startswith("IDLE"), f"{states[-1:]}")
        node.destroy_node()
        rclpy.shutdown()
    finally:
        launch.send_signal(signal.SIGINT)
        try:
            launch.wait(timeout=10)
        except subprocess.TimeoutExpired:
            launch.kill()

    print("すべて OK" if not failures else f"NG: {failures}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
