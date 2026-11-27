"""道のベンチ用のダミーの世界の ROS 2 ノード. 実写の映像（ATC のコースのフレーム）を、ロボットが
前に走った道のりに合わせて流す. 実機の代わりに、本物の VLM・実行系・安全ゲートで「道に沿って走る」を通すためのもの.

  購読:  /cmd_vel (geometry_msgs/Twist)            ロボットへの速度指令（安全ゲートの出力）
         /bench/path/goto (std_msgs/Int32)          そのフレーム番号へ映像を移す（走った道のりはそのまま）
  配信:  /odom (nav_msgs/Odometry)                  車輪オドメトリ（起動時の位置が原点）
         /camera/compressed (sensor_msgs/CompressedImage)   今のフレーム（JPEG）
         /camera/camera_info (sensor_msgs/CameraInfo)
         /cloud (sensor_msgs/PointCloud2)            空の点群（障害物なし. 点群を見る安全ゲートを通すため）
         /tf_static                                 base_link → cloud_link
         /bench/path (std_msgs/String、JSON)         今のフレーム、走った道のり（画面の表示、記録、モック VLM 用）
  提供:  /sim/reset (std_srvs/Trigger)              最初のフレームに戻す

映像は、前に走った道のり meters_per_frame ごとに 1 枚進む（後ろへ走っても戻らない）. ロボットが曲がっても
映像は変わらないので、向きを直す動き（閉ループ）は確かめられない. 確かめるのは、実写の画像に VLM が答え、
1回の答えで step_m だけ進み、止まる条件や終了の指示で止まる、という流れと、Web UI と外からの合図.
"""
from __future__ import annotations

import array
import json
import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .frame_labels import load_frames
from .road_world import RoadParams, RoadWorld

PHYSICS_DT = 0.01


def _quat_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class BenchPathNode(Node):
    def __init__(self) -> None:
        super().__init__("bench_path")
        frames_dir = str(self.declare_parameter("frames_dir", os.environ.get("ATC_FRAMES_DIR", "/atc_frames")).value)
        first = int(self.declare_parameter("first", 1).value)
        last = int(self.declare_parameter("last", 0).value)
        self.meters_per_frame = float(self.declare_parameter("meters_per_frame", 0.6).value)
        width = int(self.declare_parameter("frame_width", 960).value)
        self.hfov_deg = float(self.declare_parameter("hfov_deg", 95.0).value)
        camera_rate = float(self.declare_parameter("camera_rate", 5.0).value)
        cloud_rate = float(self.declare_parameter("cloud_rate", 10.0).value)
        self.cmd_topic = str(self.declare_parameter("cmd_topic", "/cmd_vel").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.cloud_frame = str(self.declare_parameter("cloud_frame", "cloud_link").value)

        self.frames = load_frames(frames_dir, first, last, width)
        if not self.frames:
            raise RuntimeError(f"フレームがありません: {frames_dir}（{first}〜{last or '最後'}）")
        # 前にしか進まない、まっすぐで幅の広い道（どこへ走っても止められない）
        self.world = RoadWorld(RoadParams(points=[(0.0, 0.0), (100000.0, 0.0)], width_m=1e6), bounded=False,
                               record=False)
        self.offset = 0            # 映像の始まりのフレーム（frames の添え字）
        self.origin_m = 0.0        # 映像の始まりの道のり
        self.current = 0
        self._last_tick = time.monotonic()

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._image_pub = self.create_publisher(CompressedImage, "/camera/compressed", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self._cloud_pub = self.create_publisher(PointCloud2, "/cloud", 10)
        self._state_pub = self.create_publisher(String, "/bench/path", 10)
        self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 10)
        self.create_subscription(Int32, "/bench/path/goto", self._on_goto, 10)
        self.create_service(Trigger, "/sim/reset", self._on_reset)

        self._tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.cloud_frame
        t.transform.translation.z = 0.4
        t.transform.rotation.w = 1.0
        self._tf.sendTransform(t)

        self.create_timer(PHYSICS_DT, self._physics)
        self.create_timer(0.02, self._publish_odom)
        self.create_timer(1.0 / max(camera_rate, 1.0), self._publish_camera)
        self.create_timer(1.0 / max(cloud_rate, 1.0), self._publish_cloud)
        self.create_timer(0.2, self._publish_state)
        f0, f1 = self.frames[0], self.frames[-1]
        self.get_logger().info(
            f"道のベンチ: {frames_dir} の {f0[0]}〜{f1[0]}（{len(self.frames)} 枚、{f0[3]}x{f0[4]}）. "
            f"前に {self.meters_per_frame} m 走るごとに 1 枚進めます. 速度指令 {self.cmd_topic} を受けて動く")

    # -- 受信とサービス -------------------------------------------------------
    def _on_cmd(self, msg: Twist) -> None:
        self.world.set_cmd(float(msg.linear.x), float(msg.angular.z))

    def _index_of(self, number: int) -> int:
        return min(range(len(self.frames)), key=lambda i: abs(self.frames[i][1] - number))

    def _on_goto(self, msg: Int32) -> None:
        self.offset = self._index_of(int(msg.data))
        self.origin_m = self.world.travelled
        self.get_logger().info(f"映像を {self.frames[self.offset][0]} に移しました")

    def _on_reset(self, _request, response):
        self.offset = 0
        self.origin_m = self.world.travelled
        response.success = True
        response.message = f"映像を最初のフレーム（{self.frames[0][0]}）に戻しました"
        self.get_logger().info(response.message)
        return response

    # -- 周期処理 -------------------------------------------------------------
    def _physics(self) -> None:
        now = time.monotonic()
        elapsed = min(now - self._last_tick, 0.1)
        self._last_tick = now
        steps = max(1, int(round(elapsed / PHYSICS_DT)))
        for _ in range(steps):
            self.world.step(elapsed / steps)
        ahead = max(0.0, self.world.travelled - self.origin_m)
        self.current = min(len(self.frames) - 1, self.offset + int(ahead / max(self.meters_per_frame, 1e-3)))

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_odom(self) -> None:
        ox, oy, oyaw = self.world.odom()
        v, w = self.world.velocity()
        msg = Odometry()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "odom"
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = ox, oy
        q = _quat_from_yaw(oyaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self._odom_pub.publish(msg)

    def _publish_camera(self) -> None:
        name, _number, jpeg, width, height = self.frames[self.current]
        msg = CompressedImage()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = array.array("B", jpeg)
        self._image_pub.publish(msg)
        f = width / (2.0 * math.tan(math.radians(self.hfov_deg) / 2.0))
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = width, height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [f, 0.0, width / 2.0, 0.0, f, height / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, width / 2.0, 0.0, 0.0, f, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

    def _publish_cloud(self) -> None:
        msg = PointCloud2()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.cloud_frame
        msg.height = 1
        msg.width = 0
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 0
        msg.is_dense = True
        self._cloud_pub.publish(msg)

    def _publish_state(self) -> None:
        name, number, _jpeg, _w, _h = self.frames[self.current]
        travelled = self.world.travelled
        state = {
            "time": round(self.world.t, 3),
            "frame": name,
            "frame_number": number,
            "traveled_m": round(travelled, 2),
            "meters_per_frame": self.meters_per_frame,
            "first": self.frames[0][1],
            "last": self.frames[-1][1],
            "summary": f"{name}（{travelled:.1f} m 走った）",
        }
        self._state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BenchPathNode] = None
    try:
        node = BenchPathNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
