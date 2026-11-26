"""ベンチ用のダミーの世界の ROS 2 ノード. 実機（ロボットとエレベーター）の代わりに、実時間で動く擬似世界をつなぐ.

Orin を実機に載せる前に、本物の VLM、実行系、安全ゲートを、実機と同じトピックでつないで通すためのもの.

  購読:  /cmd_vel (geometry_msgs/Twist)            ロボットへの速度指令（安全ゲートの出力）
  配信:  /odom (nav_msgs/Odometry)                  車輪オドメトリ（起動時の位置が原点）
         /camera/compressed (sensor_msgs/CompressedImage)   ダミーのカメラ画像（samples/ から選ぶ）
         /camera/camera_info (sensor_msgs/CameraInfo)
         /cloud (sensor_msgs/PointCloud2)            壁と扉の点（cloud_link 座標系）
         /tf_static                                 base_link → cloud_link
         /elevator/state、/sim/referee (std_msgs/String、中身は JSON)   画面の表示と記録用（真値）
         /ground_truth/odom、/elevator/joint_states  真値（モック VLM と記録の分析用. 実行系は使わない）
  提供:  /elevator/call、/elevator/toggle_hold、/sim/reset (std_srvs/Trigger)
"""
from __future__ import annotations

import array
import json
import math
import os
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .bench_cloud import CloudMount, world_points
from .elevator_logic import ElevatorConfig
from .pseudo_world import PseudoWorld
from .sample_camera import SampleCamera
from .world_params import view_state

PHYSICS_DT = 0.01


def _quat_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class BenchWorldNode(Node):
    def __init__(self) -> None:
        super().__init__("bench_world")
        d = ElevatorConfig()
        cfg = ElevatorConfig(
            board_floor=int(self.declare_parameter("board_floor", d.board_floor).value),
            dest_floor=int(self.declare_parameter("dest_floor", d.dest_floor).value),
            start_floor=int(self.declare_parameter("start_floor", d.start_floor).value),
            call_delay_s=float(self.declare_parameter("call_delay_s", d.call_delay_s).value),
            door_hold_s=float(self.declare_parameter("door_hold_s", 15.0).value),
            door_clear_s=float(self.declare_parameter("door_clear_s", d.door_clear_s).value),
            door_sensor=bool(self.declare_parameter("door_sensor", d.door_sensor).value),
        )
        samples_dir = str(self.declare_parameter(
            "samples_dir", os.environ.get("ELEVATOR_SAMPLES_DIR", "/workspace/samples")).value)
        self.cmd_topic = str(self.declare_parameter("cmd_topic", "/cmd_vel").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.cloud_frame = str(self.declare_parameter("cloud_frame", "cloud_link").value)
        self.publish_cloud = bool(self.declare_parameter("publish_cloud", True).value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.mount = CloudMount(
            x=float(self.declare_parameter("cloud_x", 0.10).value),
            y=float(self.declare_parameter("cloud_y", 0.0).value),
            z=float(self.declare_parameter("cloud_z", 0.40).value),
            yaw=float(self.declare_parameter("cloud_yaw", 0.0).value),
        )
        camera_rate = float(self.declare_parameter("camera_rate", 10.0).value)
        cloud_rate = float(self.declare_parameter("cloud_rate", 10.0).value)

        self.world = PseudoWorld(cfg)
        self.camera = SampleCamera(samples_dir, self.world.wp, self.world.rp)
        self.image_name = ""
        self.image_view = ""
        self._last_tick = time.monotonic()

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._truth_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)
        self._joint_pub = self.create_publisher(JointState, self.world.wp.joint_state_topic, 10)
        self._image_pub = self.create_publisher(CompressedImage, "/camera/compressed", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self._cloud_pub = self.create_publisher(PointCloud2, "/cloud", 10)
        self._state_pub = self.create_publisher(String, "/elevator/state", 10)
        self._referee_pub = self.create_publisher(String, "/sim/referee", 10)
        self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 10)
        self.create_service(Trigger, "/elevator/call", self._on_call)
        self.create_service(Trigger, "/elevator/toggle_hold", self._on_toggle_hold)
        self.create_service(Trigger, "/sim/reset", self._on_reset)

        if self.publish_tf:
            self._tf = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = self.cloud_frame
            t.transform.translation.x = self.mount.x
            t.transform.translation.y = self.mount.y
            t.transform.translation.z = self.mount.z
            qx, qy, qz, qw = _quat_from_yaw(self.mount.yaw)
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self._tf.sendTransform(t)

        self.create_timer(PHYSICS_DT, self._physics)
        self.create_timer(0.02, self._publish_odom)
        self.create_timer(1.0 / max(camera_rate, 1.0), self._publish_camera)
        if self.publish_cloud:
            self.create_timer(1.0 / max(cloud_rate, 1.0), self._publish_cloud)
        self.create_timer(0.1, self._publish_state)
        self.create_timer(0.2, self._publish_referee)
        self.get_logger().info(
            f"ダミーの世界: {cfg.board_floor} 階で乗り {cfg.dest_floor} 階へ. 扉の全開の保持 {cfg.door_hold_s} 秒. "
            f"速度指令 {self.cmd_topic} を受けて動く. カメラは {samples_dir} の画像（{len(self.camera.samples)} 枚）から選ぶ. "
            "/elevator/call で呼ぶ")

    # -- 受信とサービス -------------------------------------------------------
    def _on_cmd(self, msg: Twist) -> None:
        self.world.set_cmd(float(msg.linear.x), float(msg.angular.z))

    def _on_call(self, _request, response):
        ok, message = self.world.call()
        self.get_logger().info(f"呼び: {message}")
        response.success, response.message = ok, message
        return response

    def _on_toggle_hold(self, _request, response):
        held = self.world.toggle_hold()
        response.success = True
        response.message = "「開」を押し続けています" if held else "「開」を離しました"
        self.get_logger().info(response.message)
        return response

    def _on_reset(self, _request, response):
        message = self.world.reset()
        response.success = True
        response.message = message + ". ロボットを待機位置に置き直しました"
        self.get_logger().info(response.message)
        return response

    # -- 周期処理 -------------------------------------------------------------
    def _physics(self) -> None:
        now = time.monotonic()
        elapsed = min(now - self._last_tick, 0.1)      # 止まっていた分を一度に進めない
        self._last_tick = now
        steps = max(1, int(round(elapsed / PHYSICS_DT)))
        for _ in range(steps):
            self.world.step(elapsed / steps)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_odom(self) -> None:
        stamp = self._stamp()
        ox, oy, oyaw = self.world.odom()
        v, w = self.world.velocity()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = ox, oy
        q = _quat_from_yaw(oyaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self._odom_pub.publish(msg)

        truth = self.world.truth()
        tmsg = Odometry()
        tmsg.header.stamp = stamp
        tmsg.header.frame_id = "world"
        tmsg.child_frame_id = self.base_frame
        tmsg.pose.pose.position.x, tmsg.pose.pose.position.y = truth.robot_x, truth.robot_y
        tmsg.pose.pose.position.z = truth.robot_z
        q = _quat_from_yaw(truth.robot_yaw)
        tmsg.pose.pose.orientation.x, tmsg.pose.pose.orientation.y = q[0], q[1]
        tmsg.pose.pose.orientation.z, tmsg.pose.pose.orientation.w = q[2], q[3]
        self._truth_pub.publish(tmsg)

        joints = self.world.joint_positions()
        jmsg = JointState()
        jmsg.header.stamp = stamp
        jmsg.name = list(joints.keys())
        jmsg.position = [float(v) for v in joints.values()]
        self._joint_pub.publish(jmsg)

    def _publish_camera(self) -> None:
        stamp = self._stamp()
        name, jpeg, view = self.camera.select(self.world.truth())
        if name != self.image_name:
            self.get_logger().info(f"カメラ画像: {name}（{view.describe()}）")
        self.image_name, self.image_view = name, view.describe()
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = array.array("B", jpeg)          # bytes のままより速く入る
        self._image_pub.publish(msg)

        rp = self.world.rp
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = rp.image_width, rp.image_height
        f = rp.focal_px
        cx, cy = rp.image_width / 2.0, rp.image_height / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

    def _publish_cloud(self) -> None:
        pts = world_points(self.world.wp, self.world.truth(), self.world.rp, self.mount)
        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        msg = PointCloud2()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.cloud_frame
        msg.height = 1
        msg.width = int(arr.shape[0])
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = array.array("B", arr.tobytes())
        self._cloud_pub.publish(msg)

    def _publish_state(self) -> None:
        state = self.world.logic.state()
        state.update(view_state(self.world.wp, self.world.truth()))
        state["time"] = round(self.world.t, 3)
        state["camera_image"] = self.image_name
        state["camera_view"] = self.image_view
        self._state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def _publish_referee(self) -> None:
        result = self.world.referee.result()
        result["time"] = round(self.world.t, 3)
        self._referee_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BenchWorldNode] = None
    try:
        node = BenchWorldNode()
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
