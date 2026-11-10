"""Gazebo を使わない擬似世界（テスト用）.

ロボットは平面上の差動二輪の運動学、扉とかごは一定速度で動く関節として扱う. 時刻は擬似時刻で、
RobotIO.sleep() が呼ばれた分だけ進む. エレベーターの制御（ElevatorLogic）、モック VLM、審判は本物を使うので、
実行系のロジックを ROS も Gazebo もなしで通しで確かめられる.
"""
from __future__ import annotations

import math
import threading
from typing import Dict, List, Optional

from elevator_task.core.geometry import CameraModel, Pose2D, PoseHistory, wrap_angle
from elevator_task.core.robot_io import Frame, RobotIO
from elevator_task.core.vlm_client import VlmClient, VlmConfig, VlmReply

from elevator_sim.elevator_logic import ElevatorConfig, ElevatorLogic, Sensed
from elevator_sim.referee import Referee
from elevator_sim.world_params import RobotParams, Truth, WorldParams


def true_camera(rp: RobotParams) -> CameraModel:
    f = rp.focal_px
    return CameraModel(f, f, rp.image_width / 2.0, rp.image_height / 2.0, rp.image_width, rp.image_height,
                       rp.camera_z, math.radians(rp.camera_pitch_deg), rp.camera_x)


class FakeWorld(RobotIO):
    DT = 0.01

    def __init__(self, elevator: Optional[ElevatorConfig] = None, assumed_camera: Optional[CameraModel] = None,
                 odom_scale: float = 1.0):
        self.wp = WorldParams()
        self.rp = RobotParams()
        self.ecfg = elevator or ElevatorConfig()
        self.logic = ElevatorLogic(self.wp, self.ecfg)
        self.referee = Referee(self.wp, self.ecfg.board_floor, self.ecfg.dest_floor,
                               self.rp.half_length, self.rp.half_width)
        self.camera_true = true_camera(self.rp)
        self.camera_assumed = assumed_camera or self.camera_true
        self.odom_scale = odom_scale
        self.t = 0.0
        self._steps = 0
        self._lock = threading.RLock()
        # ロボット（真値はワールド座標、オドメトリは起動時の位置が原点）
        x, y, z, yaw = self.wp.start_pose(self.ecfg.board_floor)
        self.x, self.y, self.yaw = x, y, yaw
        self.floor = self.ecfg.board_floor        # 乗場にいるときの階
        self.on_car = False
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_at = -1.0
        self.ox = self.oy = self.oyaw = 0.0
        self.history = PoseHistory(horizon_s=30.0)
        # エレベーターの関節
        self.lift = self.wp.floor_z(self.ecfg.start_floor)
        self.lift_vel = 0.0
        self.car_door = 0.0
        self.landing: Dict[int, float] = {f: 0.0 for f in self.wp.floors()}
        self._targets = (0.0, {f: 0.0 for f in self.wp.floors()})
        # 記録
        self.door_contact = False
        self.wall_contact = False
        self.max_speed = 0.0
        self.cmd_log: List[tuple] = []
        self._frame = bytes(self.rp.image_width * self.rp.image_height * 3)
        self.history.add(Pose2D(0.0, 0.0, 0.0, 0.0))

    # -- 真値 ---------------------------------------------------------------
    @property
    def z(self) -> float:
        return self.lift if self.on_car else self.wp.floor_z(self.floor)

    def truth(self) -> Truth:
        with self._lock:
            return Truth(self.x, self.y, self.z, self.yaw, self.lift, self.car_door, dict(self.landing))

    # -- 1ステップ ----------------------------------------------------------
    def step(self) -> None:
        with self._lock:
            wp, dt = self.wp, self.DT
            self.t += dt
            self._steps += 1
            # エレベーター（50 Hz）
            if self._steps % 2 == 0:
                serving = self.logic.serving
                obstructed = (serving is not None and abs(self.z - wp.floor_z(serving)) < 0.3
                              and wp.doorway_blocked(self.x, self.y))
                sensed = Sensed(self.lift, self.car_door, self.car_door, dict(self.landing), dict(self.landing), obstructed)
                cmd = self.logic.update(self.t, sensed)
                self.lift_vel = cmd.lift_vel
                self._targets = (cmd.car_door, dict(cmd.landing))
            self.lift = max(-0.02, min(wp.lift_max + 0.02, self.lift + self.lift_vel * dt))
            step = wp.door_speed * dt
            self.car_door += max(-step, min(step, self._targets[0] - self.car_door))
            for f in wp.floors():
                self.landing[f] += max(-step, min(step, self._targets[1].get(f, 0.0) - self.landing[f]))

            # ロボット: 速度指令が 0.5 秒途絶えたら止まる（安全ゲートと同じ）
            cv, cw = self.cmd if self.t - self.cmd_at <= 0.5 else (0.0, 0.0)
            self.v += max(-1.5 * dt, min(1.5 * dt, cv - self.v))
            self.w += max(-4.0 * dt, min(4.0 * dt, cw - self.w))
            self.max_speed = max(self.max_speed, abs(self.v))
            nx = self.x + self.v * math.cos(self.yaw) * dt
            ny = self.y + self.v * math.sin(self.yaw) * dt
            nyaw = wrap_angle(self.yaw + self.w * dt)
            moved = self._try_move(nx, ny)
            self.yaw = nyaw
            # 車輪オドメトリ（壁に当たって進めなくても車輪は回るとする）
            self.ox += self.v * self.odom_scale * math.cos(self.oyaw) * dt
            self.oy += self.v * self.odom_scale * math.sin(self.oyaw) * dt
            self.oyaw = wrap_angle(self.oyaw + self.w * dt)
            self.history.add(Pose2D(self.t, self.ox, self.oy, self.oyaw))
            if not moved:
                self.v = 0.0
            self.referee.update(self.t, Truth(self.x, self.y, self.z, self.yaw, self.lift, self.car_door,
                                              dict(self.landing)))

    def _try_move(self, nx: float, ny: float) -> bool:
        wp, rp = self.wp, self.rp
        r = rp.half_length
        car_floor = wp.nearest_floor(self.lift, 0.01)
        # 出入口の通り道にかかっているか
        in_zone = (wp.hall_wall_x0 - r) < nx < (wp.car_front_x1 + r)
        if in_zone:
            here = car_floor if self.on_car else self.floor
            opening = 0.0
            if car_floor is not None and here == car_floor:
                opening = min(self.car_door, self.landing[car_floor])
            if opening < abs(ny) + rp.half_width:
                self.door_contact = True
                return False
        if nx > 0.0 and not self.on_car:
            self.on_car = True
        elif nx <= 0.0 and self.on_car:
            self.on_car = False
            self.floor = car_floor if car_floor is not None else self.floor
        if self.on_car and not in_zone:
            if nx > wp.car_x1 - r or abs(ny) > wp.car_width / 2.0 - rp.half_width:
                self.wall_contact = True
                return False
        if not self.on_car and nx < -wp.hall_depth + r:
            self.wall_contact = True
            return False
        self.x, self.y = nx, ny
        return True

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
        with self._lock:
            self.cmd = (linear, angular)
            self.cmd_at = self.t
            self.cmd_log.append((self.t, linear, angular))


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
