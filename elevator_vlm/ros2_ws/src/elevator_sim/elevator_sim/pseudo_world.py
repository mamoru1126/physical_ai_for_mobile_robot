"""Gazebo を使わない擬似世界. ROS に依存しない.

ロボットは平面上の差動二輪の運動学、扉とかごは一定速度で動く関節として扱う. エレベーターの制御
（ElevatorLogic）と審判（Referee）は本物を使う. 時刻は step(dt) を呼んだ分だけ進む.

使うところ:
  * テスト（test/fake_world.py）: 擬似時刻で、実行系のロジックを通しで確かめる.
  * ベンチ（bench_world_node.py）: 実時間で回し、実機の代わりに /odom やカメラ画像を出す.
"""
from __future__ import annotations

import math
import threading
from typing import Dict, List, Optional, Tuple

from .elevator_logic import ElevatorConfig, ElevatorLogic, Sensed
from .referee import Referee
from .world_params import RobotParams, Truth, WorldParams


def wrap_angle(a: float) -> float:
    """角度を (-pi, pi] に丸める."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


class PseudoWorld:
    """擬似世界の状態と、1ステップの更新."""

    def __init__(self, elevator: Optional[ElevatorConfig] = None, odom_scale: float = 1.0,
                 cmd_timeout_s: float = 0.5):
        self.wp = WorldParams()
        self.rp = RobotParams()
        self.ecfg = elevator or ElevatorConfig()
        self.logic = ElevatorLogic(self.wp, self.ecfg)
        self.referee = Referee(self.wp, self.ecfg.board_floor, self.ecfg.dest_floor,
                               self.rp.half_length, self.rp.half_width)
        self.odom_scale = odom_scale
        self.cmd_timeout_s = cmd_timeout_s
        self.t = 0.0
        self._steps = 0
        self._lock = threading.RLock()
        # ロボット（真値はワールド座標、オドメトリは起動時の位置が原点）
        x, y, _z, yaw = self.wp.start_pose(self.ecfg.board_floor)
        self.x, self.y, self.yaw = x, y, yaw
        self.floor = self.ecfg.board_floor        # 乗場にいるときの階
        self.on_car = False
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_at = -1.0
        self.ox = self.oy = self.oyaw = 0.0
        # エレベーターの関節
        self.lift = self.wp.floor_z(self.ecfg.start_floor)
        self.lift_vel = 0.0
        self.car_door = 0.0
        self.landing: Dict[int, float] = {f: 0.0 for f in self.wp.floors()}
        self._targets: Tuple[float, Dict[int, float]] = (0.0, {f: 0.0 for f in self.wp.floors()})
        # 記録
        self.door_contact = False
        self.wall_contact = False
        self.max_speed = 0.0
        self.cmd_log: List[tuple] = []

    # -- 真値 ---------------------------------------------------------------
    @property
    def z(self) -> float:
        return self.lift if self.on_car else self.wp.floor_z(self.floor)

    def truth(self) -> Truth:
        with self._lock:
            return Truth(self.x, self.y, self.z, self.yaw, self.lift, self.car_door, dict(self.landing))

    def odom(self) -> Tuple[float, float, float]:
        """車輪オドメトリ（起動時の位置が原点）."""
        with self._lock:
            return self.ox, self.oy, self.oyaw

    def velocity(self) -> Tuple[float, float]:
        with self._lock:
            return self.v, self.w

    def joint_positions(self) -> Dict[str, float]:
        """エレベーターの関節の位置（シミュレーションの /elevator/joint_states と同じ名前）."""
        wp = self.wp
        with self._lock:
            out = {wp.lift_joint: self.lift}
            for name in wp.car_door_joints:
                out[name] = self.car_door
            for floor in wp.floors():
                for name in wp.landing_door_joints(floor):
                    out[name] = self.landing[floor]
            return out

    # -- 操作 ---------------------------------------------------------------
    def set_cmd(self, linear: float, angular: float) -> None:
        with self._lock:
            self.cmd = (float(linear), float(angular))
            self.cmd_at = self.t
            self.cmd_log.append((self.t, float(linear), float(angular)))

    def call(self) -> Tuple[bool, str]:
        with self._lock:
            return self.logic.call(self.t)

    def toggle_hold(self) -> bool:
        with self._lock:
            self.logic.set_hold_open(not self.logic.hold_open)
            return self.logic.hold_open

    def reset(self) -> str:
        """エレベーターを初期の階へ戻し、ロボットを待機位置に置き直す（オドメトリはそのまま）."""
        with self._lock:
            _ok, message = self.logic.reset(self.t)
            self.referee.reset()
            x, y, _z, yaw = self.wp.start_pose(self.ecfg.board_floor)
            self.x, self.y, self.yaw = x, y, yaw
            self.floor = self.ecfg.board_floor
            self.on_car = False
            self.v = self.w = 0.0
            self.cmd = (0.0, 0.0)
            self.door_contact = False
            self.wall_contact = False
            return message

    # -- 1ステップ ----------------------------------------------------------
    def step(self, dt: float) -> None:
        with self._lock:
            wp = self.wp
            self.t += dt
            self._steps += 1
            # エレベーター（2 ステップに 1 回）
            if self._steps % 2 == 0:
                serving = self.logic.serving
                obstructed = (serving is not None and abs(self.z - wp.floor_z(serving)) < 0.3
                              and wp.doorway_blocked(self.x, self.y))
                sensed = Sensed(self.lift, self.car_door, self.car_door, dict(self.landing), dict(self.landing),
                                obstructed)
                cmd = self.logic.update(self.t, sensed)
                self.lift_vel = cmd.lift_vel
                self._targets = (cmd.car_door, dict(cmd.landing))
            self.lift = max(-0.02, min(wp.lift_max + 0.02, self.lift + self.lift_vel * dt))
            step = wp.door_speed * dt
            self.car_door += max(-step, min(step, self._targets[0] - self.car_door))
            for f in wp.floors():
                self.landing[f] += max(-step, min(step, self._targets[1].get(f, 0.0) - self.landing[f]))

            # ロボット: 速度指令が途絶えたら止まる（安全ゲートと同じ）
            cv, cw = self.cmd if self.t - self.cmd_at <= self.cmd_timeout_s else (0.0, 0.0)
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
