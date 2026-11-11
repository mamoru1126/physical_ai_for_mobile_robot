"""真値を使った審判. タスクが正常系のとおりに進んだかを、ロボットの真の位置と扉・かごの状態から判定する.

ロボットの実行系はこの結果を使わない（評価のためだけにある）.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

from .world_params import Truth, WorldParams


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Referee:
    def __init__(self, params: WorldParams, board_floor: int, dest_floor: int,
                 robot_half_length: float = 0.25, robot_half_width: float = 0.22):
        self.p = params
        self.board_floor = board_floor
        self.dest_floor = dest_floor
        self.half_length = robot_half_length
        self.half_width = robot_half_width
        self.reset()

    def reset(self) -> None:
        self.entered_car_at: Optional[float] = None
        self.turned_at: Optional[float] = None
        self.rode_at: Optional[float] = None
        self.exited_dest_at: Optional[float] = None
        self.exited_wrong_floor_at: Optional[float] = None
        self.left_behind_at: Optional[float] = None
        self.fell_at: Optional[float] = None
        self.max_x_in_car = -math.inf
        self.max_abs_y_at_door = 0.0
        self.started_at: Optional[float] = None

    def update(self, now: float, t: Truth) -> None:
        p = self.p
        if self.started_at is None:
            self.started_at = now
        z_board, z_dest = p.floor_z(self.board_floor), p.floor_z(self.dest_floor)
        in_car = p.in_car(t.robot_x, t.robot_y, margin=0.10) and abs(t.robot_z - t.lift_pos) < 0.15
        robot_floor = p.nearest_floor(t.robot_z, 0.15)

        if t.robot_z < -0.5:
            self.fell_at = self.fell_at or now
        if p.hall_wall_x0 - self.half_length < t.robot_x < p.car_front_x1 + self.half_length:
            self.max_abs_y_at_door = max(self.max_abs_y_at_door, abs(t.robot_y))
        if in_car:
            self.max_x_in_car = max(self.max_x_in_car, t.robot_x)
            if self.entered_car_at is None:
                self.entered_car_at = now
            # 扉の方（-x）を向いたか
            if self.turned_at is None and abs(_wrap(t.robot_yaw - math.pi)) < math.radians(25.0):
                self.turned_at = now
            if self.rode_at is None and self.dest_floor != self.board_floor and abs(t.robot_z - z_dest) < 0.05:
                self.rode_at = now
        elif self.entered_car_at is not None and t.robot_x < -0.3 and robot_floor is not None:
            # 一度かごに入った後で、乗場に出た
            arrived = self.dest_floor == self.board_floor or self.rode_at is not None
            if robot_floor == self.dest_floor and arrived:
                self.exited_dest_at = self.exited_dest_at or now
            elif self.exited_dest_at is None:
                self.exited_wrong_floor_at = self.exited_wrong_floor_at or now
        elif self.entered_car_at is None and self.dest_floor != self.board_floor:
            # まだ乗っていないのに、かごが乗る階を離れた
            if t.lift_pos > z_board + 0.05 and self.left_behind_at is None:
                self.left_behind_at = now

    @property
    def success(self) -> bool:
        return (self.exited_dest_at is not None and self.exited_wrong_floor_at is None
                and self.fell_at is None)

    @property
    def failed(self) -> bool:
        return (self.exited_wrong_floor_at is not None or self.left_behind_at is not None
                or self.fell_at is not None)

    def summary(self) -> str:
        """成否を一言で（画面と記録に出す）."""
        if self.fell_at is not None:
            return "失敗: 昇降路に落ちました"
        if self.exited_wrong_floor_at is not None:
            return "失敗: 行先階ではない階で降りました"
        if self.success:
            return "成功: 行先階で降りました"
        if self.left_behind_at is not None:
            return "失敗: 乗る前にかごが出発しました"
        stages = (("かごに入った", self.entered_car_at), ("扉の方を向いた", self.turned_at),
                  ("行先階に着いた", self.rode_at), ("行先階で降りた", self.exited_dest_at))
        return "未達成: " + " → ".join(f"{name} {'✓' if at is not None else '–'}" for name, at in stages)

    def result(self) -> Dict[str, Any]:
        p = self.p

        def rel(t: Optional[float]) -> Optional[float]:
            return None if t is None or self.started_at is None else round(t - self.started_at, 2)

        back_margin = None
        if self.max_x_in_car > -math.inf:
            back_margin = round(p.car_x1 - (self.max_x_in_car + self.half_length), 3)
        return {
            "summary": self.summary(),
            "success": self.success,
            "failed": self.failed,
            "board_floor": self.board_floor,
            "dest_floor": self.dest_floor,
            "entered_car_s": rel(self.entered_car_at),
            "turned_s": rel(self.turned_at),
            "arrived_dest_floor_s": rel(self.rode_at),
            "exited_dest_s": rel(self.exited_dest_at),
            "exited_wrong_floor_s": rel(self.exited_wrong_floor_at),
            "left_behind_s": rel(self.left_behind_at),
            "fell_s": rel(self.fell_at),
            # 奥の壁までの余裕（ロボットの前端から）. 負なら壁に当たった疑いがある
            "back_wall_margin_m": back_margin,
            # 出入口を通るときの横のずれと、枠までの余裕
            "max_lateral_at_door_m": round(self.max_abs_y_at_door, 3),
            "door_side_margin_m": round(p.door_width / 2.0 - self.half_width - self.max_abs_y_at_door, 3),
        }
