"""実行系の ROS 2 ノード. 中核（core/）とロボット本体の3つの接点をつなぐ.

  購読:  camera/image_raw (sensor_msgs/Image)、camera/camera_info (sensor_msgs/CameraInfo)、
         odom (nav_msgs/Odometry)
  配信:  cmd_vel_raw (geometry_msgs/Twist)  ← safety_gate が制限をかけて cmd_vel に流す
  呼出:  safety/stop、safety/reset (std_srvs/Trigger)

Web UI もこのプロセスの中で動く（既定は http://<ホスト>:8080/）.
"""
from __future__ import annotations

import math
import os
import threading
import time
from functools import partial
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from .core.config import TaskConfig, load_config
from .core.geometry import CameraModel, Pose2D, PoseHistory
from .core.robot_io import Frame, RobotIO, encode_jpeg
from .core.runtime import TaskRuntime
from .core.webui import WebUI

_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def image_to_rgb8(msg: Image) -> Optional[bytes]:
    """sensor_msgs/Image を RGB8 のバイト列にする. 対応していない形式は None."""
    enc = msg.encoding.lower()
    channels = _CHANNELS.get(enc)
    if channels is None:
        return None
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    data = bytes(msg.data)
    if enc == "rgb8" and step == width * 3:
        return data
    arr = np.frombuffer(data, dtype=np.uint8).reshape(height, step)[:, :width * channels]
    arr = arr.reshape(height, width, channels)
    if enc == "bgr8":
        arr = arr[:, :, ::-1]
    elif enc == "rgba8":
        arr = arr[:, :, :3]
    elif enc == "bgra8":
        arr = arr[:, :, 2::-1]
    elif enc == "mono8":
        arr = np.repeat(arr, 3, axis=2)
    return np.ascontiguousarray(arr).tobytes()


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RosRobotIO(RobotIO):
    def __init__(self, node: Node, cfg: TaskConfig):
        self._node = node
        self._cfg = cfg
        self._lock = threading.Lock()
        self._frame: Optional[Frame] = None
        self._info_camera: Optional[CameraModel] = None
        self._history = PoseHistory(horizon_s=15.0)
        self._warned_encoding = False
        self._cmd_pub = node.create_publisher(Twist, "cmd_vel_raw", 10)
        node.create_subscription(Image, "camera/image_raw", self._on_image, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, "camera/camera_info", self._on_info, qos_profile_sensor_data)
        node.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data)

    # -- 受信 ---------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        rgb = image_to_rgb8(msg)
        if rgb is None:
            if not self._warned_encoding:
                self._warned_encoding = True
                self._node.get_logger().error(f"対応していない画像の形式です: {msg.encoding}")
            return
        stamp = _stamp_to_sec(msg.header.stamp) or self.now()
        with self._lock:
            self._frame = Frame(stamp, int(msg.width), int(msg.height), rgb)

    def _on_info(self, msg: CameraInfo) -> None:
        fx, fy, cx, cy = float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])
        if fx <= 0.0 or fy <= 0.0 or msg.width == 0 or msg.height == 0:
            return
        mount = self._cfg.camera
        model = CameraModel(fx, fy, cx, cy, int(msg.width), int(msg.height), mount.height_m,
                            math.radians(mount.pitch_deg), mount.x_offset_m, mount.y_offset_m)
        with self._lock:
            self._info_camera = model

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = _stamp_to_sec(msg.header.stamp) or self.now()
        self._history.add(Pose2D(stamp, float(p.x), float(p.y), yaw))

    # -- RobotIO ------------------------------------------------------------
    def now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def sleep(self, seconds: float) -> None:
        time.sleep(max(float(seconds), 0.0))

    def get_frame(self) -> Optional[Frame]:
        with self._lock:
            return self._frame

    def get_pose(self) -> Optional[Pose2D]:
        return self._history.latest()

    def pose_at(self, stamp: float) -> Optional[Pose2D]:
        return self._history.at(stamp)

    def get_camera(self) -> Optional[CameraModel]:
        with self._lock:
            if self._info_camera is not None:
                return self._info_camera
            frame = self._frame
        if frame is None:
            return None
        # カメラ情報が届いていないときは、設定の水平画角から作る
        mount = self._cfg.camera
        f = frame.width / (2.0 * math.tan(math.radians(mount.hfov_deg) / 2.0))
        return CameraModel(f, f, frame.width / 2.0, frame.height / 2.0, frame.width, frame.height,
                           mount.height_m, math.radians(mount.pitch_deg), mount.x_offset_m, mount.y_offset_m)

    def send_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        try:
            self._cmd_pub.publish(msg)
        except Exception:  # noqa: BLE001  終了処理の途中で呼ばれても落とさない
            pass

    def status(self) -> List[str]:
        out: List[str] = []
        now = self.now()
        with self._lock:
            frame = self._frame
            has_info = self._info_camera is not None
        if frame is None:
            out.append("カメラ画像が未受信")
        elif now - frame.stamp > self._cfg.perception.max_image_age_s:
            out.append(f"カメラ画像が古い（{now - frame.stamp:.1f} 秒前）")
        if not has_info:
            out.append("カメラ情報が未受信（設定の画角で代用）")
        pose = self._history.latest()
        if pose is None:
            out.append("オドメトリが未受信")
        elif now - pose.stamp > self._cfg.skills.max_odom_age_s:
            out.append(f"オドメトリが古い（{now - pose.stamp:.1f} 秒前）")
        return out


class TaskNode(Node):
    def __init__(self) -> None:
        super().__init__("elevator_task")
        self.declare_parameter("config_file", "")
        self.declare_parameter("web_port", 0)
        self.declare_parameter("log_dir", "")
        config_file = str(self.get_parameter("config_file").value or "")
        overrides: Dict[str, object] = {}
        web_port = int(self.get_parameter("web_port").value or 0)
        if web_port > 0:
            overrides["web"] = {"port": web_port}
        log_dir = str(self.get_parameter("log_dir").value or "")
        if log_dir:
            overrides["log_dir"] = log_dir
        cfg = load_config(config_file or None, overrides)
        if not cfg.plan_dir:
            try:        # 用意済みの計画は、このパッケージの plans/ にある
                cfg.plan_dir = os.path.join(get_package_share_directory("elevator_task"), "plans")
            except Exception:  # noqa: BLE001
                pass

        self.io = RosRobotIO(self, cfg)
        stop_cli = self.create_client(Trigger, "safety/stop")
        reset_cli = self.create_client(Trigger, "safety/reset")
        triggers = {}
        for button in cfg.web.buttons:
            name, service = button.get("name", ""), button.get("service", "")
            if not name or not service:
                self.get_logger().warn(f"web.buttons の項目に name と service が必要です: {button}")
                continue
            triggers[name] = partial(self._call_trigger, self.create_client(Trigger, service), service)
        # 画面に映すだけの追加のカメラ（実行系は使わない）
        self._extra_lock = threading.Lock()
        self._extra_frames: Dict[str, Frame] = {}
        for cam in cfg.web.extra_cameras:
            name, topic = cam.get("name", ""), cam.get("topic", "")
            if not name or not topic:
                self.get_logger().warn(f"web.extra_cameras の項目に name と topic が必要です: {cam}")
                continue
            self.create_subscription(Image, topic, partial(self._on_extra_image, name), qos_profile_sensor_data)
        self.runtime = TaskRuntime(
            cfg, self.io,
            stop_hook=partial(self._call_trigger, stop_cli, "safety/stop"),
            reset_hook=partial(self._call_trigger, reset_cli, "safety/reset"),
            triggers=triggers, extra_jpeg=self._extra_jpeg)
        self.web = WebUI(self.runtime, cfg.web.host, cfg.web.port, cfg.web.stream_hz)
        self.web.start()
        self.get_logger().info(
            f"Web UI: http://localhost:{self.web.port}/  VLM: {cfg.vlm.base_url} ({cfg.vlm.model})  "
            f"記録: {self.runtime.events.run_dir or '保存しない'}")

    def _on_extra_image(self, name: str, msg: Image) -> None:
        rgb = image_to_rgb8(msg)
        if rgb is not None:
            with self._extra_lock:
                self._extra_frames[name] = Frame(_stamp_to_sec(msg.header.stamp), int(msg.width), int(msg.height), rgb)

    def _extra_jpeg(self, name: str) -> Optional[bytes]:
        with self._extra_lock:
            frame = self._extra_frames.get(name)
        return None if frame is None else encode_jpeg(frame, 75)

    def _call_trigger(self, client, service: str, timeout: float = 3.0) -> Tuple[bool, str]:
        """std_srvs/Trigger を呼んで結果を待つ（Web UI のスレッドから呼ばれる）."""
        if not client.service_is_ready():
            return False, f"サービス {service} が見つかりません"
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return False, f"サービス {service} から応答がありません"
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001
            return False, f"サービス {service} の呼び出しに失敗しました: {e}"
        if res is None:
            return False, f"サービス {service} の結果を受け取れません"
        return bool(res.success), str(res.message)

    def shutdown(self) -> None:
        try:
            self.runtime.close()
            self.web.stop()
        except Exception:  # noqa: BLE001
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TaskNode] = None
    try:
        node = TaskNode()
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
