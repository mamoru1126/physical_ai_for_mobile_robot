"""エレベーターの制御と審判の ROS 2 ノード（シミュレーション専用）.

  購読:  /elevator/joint_states (sensor_msgs/JointState)、/ground_truth/odom (nav_msgs/Odometry)
  配信:  /elevator/lift/cmd_vel、/elevator/car_door/cmd_pos、/elevator/landing_door_<階>/cmd_pos (std_msgs/Float64)
         /elevator/state、/sim/referee (std_msgs/String、中身は JSON)
  提供:  /elevator/call         (std_srvs/Trigger)  呼びと行先階の登録（人か設備が行う操作に当たる）
         /elevator/toggle_hold  (std_srvs/Trigger)  「開」ボタンを押し続ける／離す
         /sim/reset             (std_srvs/Trigger)  エレベーターとロボットを初期位置に戻す
"""
from __future__ import annotations

import json
import subprocess
import threading
from typing import Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from .elevator_logic import ElevatorConfig, ElevatorLogic, Sensed
from .referee import Referee
from .world_params import RobotParams, Truth, WorldParams, yaw_from_quaternion


class ElevatorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("elevator_controller")
        defaults = ElevatorConfig()
        cfg = ElevatorConfig(
            board_floor=int(self.declare_parameter("board_floor", defaults.board_floor).value),
            dest_floor=int(self.declare_parameter("dest_floor", defaults.dest_floor).value),
            start_floor=int(self.declare_parameter("start_floor", defaults.start_floor).value),
            call_delay_s=float(self.declare_parameter("call_delay_s", defaults.call_delay_s).value),
            door_hold_s=float(self.declare_parameter("door_hold_s", defaults.door_hold_s).value),
            door_clear_s=float(self.declare_parameter("door_clear_s", defaults.door_clear_s).value),
            door_sensor=bool(self.declare_parameter("door_sensor", defaults.door_sensor).value),
        )
        self.wp = WorldParams()
        robot = RobotParams()
        self.logic = ElevatorLogic(self.wp, cfg)
        self.referee = Referee(self.wp, cfg.board_floor, cfg.dest_floor, robot.half_length, robot.half_width)
        self.cfg = cfg

        self._joint_pos: Dict[str, float] = {}
        self._truth_pose: Optional[tuple] = None      # (x, y, z, yaw)
        self._tick_count = 0
        self._last_phase = ""

        self._lift_pub = self.create_publisher(Float64, self.wp.lift_cmd_topic, 10)
        self._car_door_pub = self.create_publisher(Float64, self.wp.car_door_cmd_topic, 10)
        self._landing_pubs = {f: self.create_publisher(Float64, self.wp.landing_door_cmd_topic(f), 10)
                              for f in self.wp.floors()}
        self._state_pub = self.create_publisher(String, "/elevator/state", 10)
        self._referee_pub = self.create_publisher(String, "/sim/referee", 10)
        self.create_subscription(JointState, self.wp.joint_state_topic, self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_truth, qos_profile_sensor_data)
        self.create_service(Trigger, "/elevator/call", self._on_call)
        self.create_service(Trigger, "/elevator/toggle_hold", self._on_toggle_hold)
        self.create_service(Trigger, "/sim/reset", self._on_reset)
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f"エレベーター: {cfg.board_floor} 階で乗り {cfg.dest_floor} 階へ. 全開の保持 {cfg.door_hold_s} 秒、"
            f"扉のセンサ {'あり' if cfg.door_sensor else 'なし'}. /elevator/call で呼ぶ")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- 受信 ---------------------------------------------------------------
    def _on_joints(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._joint_pos[name] = float(position)

    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._truth_pose = (float(p.x), float(p.y), float(p.z), yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # -- サービス -------------------------------------------------------------
    def _on_call(self, _request, response):
        ok, message = self.logic.call(self._now())
        self.get_logger().info(f"呼び: {message}")
        response.success = ok
        response.message = message
        return response

    def _on_toggle_hold(self, _request, response):
        self.logic.set_hold_open(not self.logic.hold_open)
        response.success = True
        response.message = "「開」を押し続けています" if self.logic.hold_open else "「開」を離しました"
        self.get_logger().info(response.message)
        return response

    def _on_reset(self, _request, response):
        _ok, message = self.logic.reset(self._now())
        self.referee.reset()
        threading.Thread(target=self._teleport_robot, name="teleport", daemon=True).start()
        response.success = True
        response.message = message + ". ロボットを待機位置へ戻します"
        self.get_logger().info(response.message)
        return response

    def _teleport_robot(self) -> None:
        """ロボットを待機位置へ瞬間移動させる（gz service の set_pose）."""
        x, y, z, _yaw = self.wp.start_pose(self.cfg.board_floor)
        req = (f'name: "{self.wp.robot_model}" position: {{x: {x:.3f} y: {y:.3f} z: {z + 0.02:.3f}}} '
               "orientation: {x: 0 y: 0 z: 0 w: 1}")
        cmd = ["gz", "service", "-s", f"/world/{self.wp.world_name}/set_pose", "--reqtype", "gz.msgs.Pose",
               "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
            text = (out.stdout or "") + (out.stderr or "")
            if "data: true" not in text:
                self.get_logger().warn(f"ロボットを戻せませんでした: {text.strip()}")
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f"gz service を実行できません: {e}")

    # -- 周期処理 -------------------------------------------------------------
    def _sensed(self) -> Sensed:
        wp = self.wp
        pos = self._joint_pos
        s = Sensed()
        if wp.lift_joint in pos:
            s.lift_pos = pos[wp.lift_joint]
        car = [pos[j] for j in wp.car_door_joints if j in pos]
        if len(car) == len(wp.car_door_joints):
            s.car_door, s.car_door_max = min(car), max(car)
        for floor in wp.floors():
            vals = [pos[j] for j in wp.landing_door_joints(floor) if j in pos]
            if len(vals) == 2:
                s.landing[floor], s.landing_max[floor] = min(vals), max(vals)
        pose = self._truth_pose
        serving = self.logic.serving
        if pose is not None and serving is not None:
            s.obstructed = abs(pose[2] - wp.floor_z(serving)) < 0.3 and wp.doorway_blocked(pose[0], pose[1])
        return s

    def _tick(self) -> None:
        now = self._now()
        sensed = self._sensed()
        cmd = self.logic.update(now, sensed)
        self._lift_pub.publish(Float64(data=float(cmd.lift_vel)))
        self._car_door_pub.publish(Float64(data=float(cmd.car_door)))
        for floor, pub in self._landing_pubs.items():
            pub.publish(Float64(data=float(cmd.landing.get(floor, 0.0))))

        pose = self._truth_pose
        if pose is not None and sensed.lift_pos is not None and sensed.car_door is not None:
            self.referee.update(now, Truth(pose[0], pose[1], pose[2], pose[3], sensed.lift_pos,
                                           sensed.car_door, dict(sensed.landing)))
        if self.logic.phase != self._last_phase:
            state = self.logic.state()
            self.get_logger().info(f"状態: {self._last_phase or '(起動)'} → {self.logic.phase}"
                                   f"（階 {state['floor']}、かごの高さ {state['lift_pos']}）")
            self._last_phase = self.logic.phase
        self._tick_count += 1
        if self._tick_count % 5 == 0:
            state = self.logic.state()
            state["time"] = round(now, 3)
            self._state_pub.publish(String(data=json.dumps(state)))
        if self._tick_count % 10 == 0:
            result = self.referee.result()
            result["time"] = round(now, 3)
            self._referee_pub.publish(String(data=json.dumps(result)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[ElevatorControllerNode] = None
    try:
        node = ElevatorControllerNode()
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
