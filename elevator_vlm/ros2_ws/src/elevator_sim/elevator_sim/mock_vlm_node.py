"""モック VLM の ROS 2 ノード（シミュレーション専用）. 真値を購読し、OpenAI 互換の HTTP サーバを立てる.

  mode:=elevator（既定）  エレベーターの真値から答える
    購読:  /ground_truth/odom、/elevator/joint_states、/camera/camera_info
  mode:=path              道のベンチ（bench_path_node）の今のフレームと、その正解（labels）から答える
    購読:  /bench/path、/camera/camera_info
  HTTP:  http://<host>:<port>/v1/chat/completions、/v1/models
"""
from __future__ import annotations

import json
import math
import threading
from typing import Dict, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String

from elevator_task.core.geometry import CameraModel

from .frame_labels import FrameLabels
from .mock_vlm import MockConfig, MockVlm, make_server
from .world_params import RobotParams, Truth, WorldParams, yaw_from_quaternion


class MockVlmNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_vlm")
        host = str(self.declare_parameter("host", "127.0.0.1").value)
        port = int(self.declare_parameter("port", 8001).value)
        cfg = MockConfig(
            latency_s=float(self.declare_parameter("latency_s", 0.3).value),
            point_noise_px=float(self.declare_parameter("point_noise_px", 0.0).value),
            check_flip_prob=float(self.declare_parameter("check_flip_prob", 0.0).value),
            seed=int(self.declare_parameter("seed", 0).value),
        )
        self.mode = str(self.declare_parameter("mode", "elevator").value).strip().lower()
        labels_path = str(self.declare_parameter("labels", "/atc_frames/labels.json").value)
        if self.mode not in ("elevator", "path"):
            raise RuntimeError(f"mode は elevator か path で指定してください（{self.mode}）")
        self.wp = WorldParams()
        self.robot = RobotParams()
        self._lock = threading.Lock()
        self._pose: Optional[tuple] = None
        self._joint_pos: Dict[str, float] = {}
        self._camera: Optional[CameraModel] = None
        self._frame_number: Optional[int] = None
        self.labels: Optional[FrameLabels] = None
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_info, qos_profile_sensor_data)
        if self.mode == "path":
            self.labels = FrameLabels.load(labels_path)
            self.create_subscription(String, "/bench/path", self._on_bench_path, 10)
            self.mock = MockVlm(self.wp, cfg, lambda: None, self._get_camera, self._missing,
                                path_fn=self._path_answer, statement_fn=self._statement_answer)
            self.get_logger().info(f"道のベンチ用: 正解 {labels_path}（指差し {len(self.labels.points)} 枚、"
                                   f"止まる条件 {len(self.labels.statements)} 文）")
        else:
            self.create_subscription(Odometry, "/ground_truth/odom", self._on_truth, qos_profile_sensor_data)
            self.create_subscription(JointState, self.wp.joint_state_topic, self._on_joints, qos_profile_sensor_data)
            self.mock = MockVlm(self.wp, cfg, self._truth, self._get_camera, self._missing)
        self.create_timer(5.0, self._report_missing)
        self.server = make_server(self.mock, host, port)
        self._server_thread = threading.Thread(target=self.server.serve_forever, name="mock_vlm_http", daemon=True)
        self._server_thread.start()
        self.get_logger().info(
            f"モック VLM: http://{host}:{port}/v1（遅れ {cfg.latency_s} 秒、指差しのずれ {cfg.point_noise_px} 画素、"
            f"誤答の確率 {cfg.check_flip_prob}）. 画像は見ずに真値から答える")

    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._lock:
            self._pose = (float(p.x), float(p.y), float(p.z), yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_joints(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_pos[name] = float(position)

    def _on_info(self, msg: CameraInfo) -> None:
        fx, fy = float(msg.k[0]), float(msg.k[4])
        if fx <= 0.0 or fy <= 0.0:
            return
        r = self.robot
        # 取り付け位置は実行系の設定ではなく、シミュレーションのロボットの寸法（真の値）を使う
        model = CameraModel(fx, fy, float(msg.k[2]), float(msg.k[5]), int(msg.width), int(msg.height),
                            r.camera_z, math.radians(r.camera_pitch_deg), r.camera_x)
        with self._lock:
            self._camera = model

    def _get_camera(self) -> Optional[CameraModel]:
        with self._lock:
            return self._camera

    # -- 道のベンチ ------------------------------------------------------------
    def _on_bench_path(self, msg: String) -> None:
        try:
            number = json.loads(msg.data).get("frame_number")
        except ValueError:
            return
        with self._lock:
            self._frame_number = None if number is None else int(number)

    def _path_answer(self):
        with self._lock:
            number = self._frame_number
        return self.labels.path_point(number) if self.labels else None

    def _statement_answer(self, text: str) -> Optional[bool]:
        with self._lock:
            number = self._frame_number
        return self.labels.statement(text, number) if self.labels else None

    def _missing(self) -> List[str]:
        with self._lock:
            out = []
            if self.mode == "path":
                if self._frame_number is None:
                    out.append("/bench/path")
                return out
            if self._pose is None:
                out.append("/ground_truth/odom")
            if self.wp.lift_joint not in self._joint_pos:
                out.append(self.wp.joint_state_topic)
            if self._camera is None:
                out.append("/camera/camera_info")
        return out

    def _report_missing(self) -> None:
        missing = self._missing()
        if missing:
            self.get_logger().warn("真値が届いていないので答えられません: " + ", ".join(missing))

    def _truth(self) -> Optional[Truth]:
        wp = self.wp
        with self._lock:
            pose = self._pose
            pos = dict(self._joint_pos)
        if pose is None or wp.lift_joint not in pos:
            return None
        car = [pos.get(j, 0.0) for j in wp.car_door_joints]
        landing = {f: min(pos.get(j, 0.0) for j in wp.landing_door_joints(f)) for f in wp.floors()}
        return Truth(pose[0], pose[1], pose[2], pose[3], pos[wp.lift_joint], min(car), landing)

    def shutdown(self) -> None:
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MockVlmNode] = None
    try:
        node = MockVlmNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
