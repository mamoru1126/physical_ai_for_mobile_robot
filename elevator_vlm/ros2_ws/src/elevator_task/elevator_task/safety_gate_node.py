"""安全ゲートの ROS 2 ノード. スキルの出力（cmd_vel_raw）に制限をかけて台車（cmd_vel）に流す.

  購読:  cmd_vel_raw (geometry_msgs/Twist)、odom (nav_msgs/Odometry)、
         camera/camera_info (sensor_msgs/CameraInfo、カメラが生きているかの監視用)
  配信:  cmd_vel (geometry_msgs/Twist、50 Hz)、safety/state (std_msgs/String)
  提供:  safety/stop、safety/reset (std_srvs/Trigger)
"""
from __future__ import annotations

import json
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core.config import load_config
from .core.safety import SafetyGate


class SafetyGateNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_gate")
        self.declare_parameter("config_file", "")
        config_file = str(self.get_parameter("config_file").value or "")
        cfg = load_config(config_file or None)
        self.gate = SafetyGate(cfg.safety)
        self._gate_state = ""
        self._ticks = 0
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "safety/state", 10)
        self.create_subscription(Twist, "cmd_vel_raw", self._on_cmd, 10)
        self.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "camera/camera_info", self._on_camera, qos_profile_sensor_data)
        self.create_service(Trigger, "safety/stop", self._on_stop)
        self.create_service(Trigger, "safety/reset", self._on_reset)
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f"安全ゲート: 並進 {cfg.safety.max_linear} m/s、旋回 {cfg.safety.max_angular} rad/s まで")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self.gate.on_cmd(self._now(), float(msg.linear.x), float(msg.angular.z))

    def _on_odom(self, _msg: Odometry) -> None:
        self.gate.on_odom(self._now())

    def _on_camera(self, _msg: CameraInfo) -> None:
        self.gate.on_camera(self._now())

    def _on_stop(self, _request, response):
        self.gate.stop("停止の要求")
        self.get_logger().warn("停止の要求を受けました. 解除されるまで速度 0 を出します")
        response.success = True
        response.message = "停止しました"
        return response

    def _on_reset(self, _request, response):
        self.gate.reset()
        self.get_logger().info("停止を解除しました")
        response.success = True
        response.message = "停止を解除しました"
        return response

    def _tick(self) -> None:
        linear, angular, state = self.gate.output(self._now())
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)
        if state != self._gate_state:
            self.get_logger().info(f"状態: {self._gate_state or '(起動)'} → {state}")
            self._gate_state = state
        self._ticks += 1
        if self._ticks % 25 == 0:
            self._state_pub.publish(String(data=json.dumps(
                {"state": state, "linear": round(linear, 3), "angular": round(angular, 3)})))


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SafetyGateNode] = None
    try:
        node = SafetyGateNode()
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
