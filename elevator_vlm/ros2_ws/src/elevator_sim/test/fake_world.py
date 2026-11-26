"""Gazebo を使わない擬似世界（テスト用）.

世界の動き（差動二輪の運動学、一定速度で動く扉とかご、エレベーターの制御、審判）は
elevator_sim.pseudo_world.PseudoWorld を使う（ベンチのダミーの世界と同じもの）. ここでは、
時刻を擬似時刻にして（RobotIO.sleep() が呼ばれた分だけ進む）、実行系の RobotIO として見せる.
"""
from __future__ import annotations

import math
import threading
from typing import Optional

from elevator_task.core.geometry import CameraModel, Pose2D, PoseHistory
from elevator_task.core.robot_io import Frame, RobotIO
from elevator_task.core.vlm_client import VlmClient, VlmConfig, VlmReply

from elevator_sim.elevator_logic import ElevatorConfig
from elevator_sim.pseudo_world import PseudoWorld
from elevator_sim.world_params import RobotParams


def true_camera(rp: RobotParams) -> CameraModel:
    f = rp.focal_px
    return CameraModel(f, f, rp.image_width / 2.0, rp.image_height / 2.0, rp.image_width, rp.image_height,
                       rp.camera_z, math.radians(rp.camera_pitch_deg), rp.camera_x)


class FakeWorld(PseudoWorld, RobotIO):
    DT = 0.01

    def __init__(self, elevator: Optional[ElevatorConfig] = None, assumed_camera: Optional[CameraModel] = None,
                 odom_scale: float = 1.0):
        super().__init__(elevator, odom_scale=odom_scale)
        self.camera_true = true_camera(self.rp)
        self.camera_assumed = assumed_camera or self.camera_true
        self.history = PoseHistory(horizon_s=30.0)
        self._frame = bytes(self.rp.image_width * self.rp.image_height * 3)
        self.history.add(Pose2D(0.0, 0.0, 0.0, 0.0))

    # -- 1ステップ ----------------------------------------------------------
    def step(self, dt: Optional[float] = None) -> None:
        super().step(self.DT if dt is None else dt)
        self.history.add(Pose2D(self.t, self.ox, self.oy, self.oyaw))

    # -- RobotIO ------------------------------------------------------------
    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        for _ in range(max(1, int(round(seconds / self.DT)))):
            self.step()

    def get_frame(self) -> Optional[Frame]:
        stamp = math.floor(self.t * self.rp.camera_rate) / self.rp.camera_rate
        return Frame(stamp, self.rp.image_width, self.rp.image_height, self._frame)

    def get_pose(self) -> Optional[Pose2D]:
        return self.history.latest()

    def pose_at(self, stamp: float) -> Optional[Pose2D]:
        return self.history.at(stamp)

    def get_camera(self) -> Optional[CameraModel]:
        return self.camera_assumed

    def send_cmd(self, linear: float, angular: float) -> None:
        self.set_cmd(linear, angular)


class SimLatencyVlm(VlmClient):
    """HTTP の往復は一瞬で済ませ、代わりに擬似時刻を latency_s だけ進めて推論の遅れを再現する."""

    def __init__(self, cfg: VlmConfig, world: FakeWorld, latency_s: float):
        super().__init__(cfg)
        self.world = world
        self.latency_s = latency_s
        self._main = threading.current_thread()

    def chat(self, *args, **kwargs) -> VlmReply:
        reply = super().chat(*args, **kwargs)
        if threading.current_thread() is self._main and self.latency_s > 0.0:
            self.world.sleep(self.latency_s)
        reply.latency_s = self.latency_s
        return reply
