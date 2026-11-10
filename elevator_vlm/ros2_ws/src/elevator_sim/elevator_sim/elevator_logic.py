"""エレベーターの動き（正常系）を決める状態機械. ROS に依存しない.

  呼び → 乗る階で戸開 → 一定時間開けて戸閉 → 行先階へ昇降（途中の階には止まらない）
       → 戸開 → 一定時間開けて戸閉

扉が閉まる途中で出入口に物があれば開き直す（多光軸センサに相当. door_sensor で無効にできる）.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .world_params import WorldParams

IDLE = "idle"            # 扉を閉じて停止
MOVING = "moving"        # 昇降中
OPENING = "opening"      # 戸開中
OPEN = "open"            # 全開で保持
CLOSING = "closing"      # 戸閉中


@dataclass
class ElevatorConfig:
    board_floor: int = 1           # ロボットが乗る階
    dest_floor: int = 3            # 行先階
    start_floor: int = 1           # かごの初期位置
    call_delay_s: float = 3.0      # 呼びから動き始めるまでの時間
    door_hold_s: float = 6.0       # 全開で保持する時間
    door_clear_s: float = 2.0      # 出入口から物がなくなった後、閉め始めるまでの時間
    door_sensor: bool = True       # 出入口に物があれば閉めない・開き直す
    depart_delay_s: float = 1.0    # 戸閉から昇降を始めるまでの時間
    door_tol: float = 0.01         # 扉の全開・全閉の判定の許容 [m]
    level_tol: float = 0.004       # 着床の判定の許容 [m]
    door_timeout_s: float = 15.0   # 扉が動き切らないときに先へ進むまでの時間
    lift_kp: float = 4.0           # 着床の手前での速度の比例ゲイン [1/s]


@dataclass
class Sensed:
    lift_pos: Optional[float] = None
    car_door: Optional[float] = None                 # 2枚のうち開きが小さい方 [m]
    car_door_max: Optional[float] = None             # 2枚のうち開きが大きい方 [m]
    landing: Dict[int, float] = field(default_factory=dict)       # 階 → 開きが小さい方
    landing_max: Dict[int, float] = field(default_factory=dict)   # 階 → 開きが大きい方
    obstructed: bool = False                         # 出入口に物があるか


@dataclass
class Commands:
    lift_vel: float = 0.0
    car_door: float = 0.0
    landing: Dict[int, float] = field(default_factory=dict)


def lift_velocity(error: float, v_prev: float, dt: float, v_max: float, a_max: float, kp: float) -> float:
    """昇降の速度指令. 加減速を a_max に抑え、目標の手前で滑らかに減速する.

    急に動かすと、乗っているロボットが床から浮いたり跳ねたりするため.
    """
    mag = min(v_max, math.sqrt(2.0 * 0.7 * a_max * abs(error)), kp * abs(error))
    v_des = math.copysign(mag, error)
    step = a_max * max(dt, 0.0)
    return v_prev + max(-step, min(step, v_des - v_prev))


class ElevatorLogic:
    def __init__(self, params: WorldParams, cfg: ElevatorConfig):
        for name in ("board_floor", "dest_floor", "start_floor"):
            floor = getattr(cfg, name)
            if not 1 <= floor <= params.num_floors:
                raise ValueError(f"{name}={floor} は 1〜{params.num_floors} の範囲で指定してください")
        self.p = params
        self.cfg = cfg
        self.phase = IDLE
        self.stops: List[int] = []           # これから扉を開ける階（順番）
        self.serving: Optional[int] = None   # いま扉を開け閉めしている階
        self.return_floor: Optional[int] = None
        self.hold_open = False
        self.lift_target = params.floor_z(cfg.start_floor)
        self.door_cycles = 0
        self.reopen_count = 0
        self._v = 0.0
        self._last_now: Optional[float] = None
        self._ready_at = 0.0
        self._t_phase = 0.0
        self._t_open = 0.0
        self._t_obstructed = -1e9
        self._last: Sensed = Sensed()

    # -- 外からの操作 ---------------------------------------------------------
    def call(self, now: float) -> Tuple[bool, str]:
        """呼びと行先階の登録（人か設備が行う操作に当たる）."""
        if self.stops or self.phase != IDLE or self.return_floor is not None:
            return False, "エレベーターは動作中です"
        cfg = self.cfg
        self.stops = [cfg.board_floor] if cfg.board_floor == cfg.dest_floor else [cfg.board_floor, cfg.dest_floor]
        self._ready_at = now + cfg.call_delay_s
        return True, f"{cfg.board_floor} 階で乗り、{cfg.dest_floor} 階へ向かいます"

    def reset(self, now: float) -> Tuple[bool, str]:
        """予定を取り消し、扉を閉じて初期の階へ戻る."""
        self.stops = []
        self.hold_open = False
        self.return_floor = self.cfg.start_floor
        self.door_cycles = 0
        self.reopen_count = 0
        if self.phase in (OPENING, OPEN):
            self._enter(CLOSING, now)
        elif self.phase == MOVING:
            self.lift_target = self.p.floor_z(self.cfg.start_floor)   # 昇降中なら行き先を付け替える
        return True, f"扉を閉じて {self.cfg.start_floor} 階へ戻ります"

    def set_hold_open(self, flag: bool) -> None:
        """「開」ボタンを押し続けるのに当たる."""
        self.hold_open = bool(flag)

    # -- 状態の更新 ---------------------------------------------------------
    def _enter(self, phase: str, now: float) -> None:
        self.phase = phase
        self._t_phase = now

    def _level_floor(self, lift_pos: float) -> Optional[int]:
        return self.p.nearest_floor(lift_pos, self.cfg.level_tol)

    def update(self, now: float, s: Sensed) -> Commands:
        cfg, p = self.cfg, self.p
        dt = 0.0 if self._last_now is None else max(0.0, min(now - self._last_now, 0.2))
        if self._last_now is not None and now < self._last_now:
            dt = 0.0                      # 時刻が巻き戻った
        self._last_now = now
        self._last = s
        cmd = Commands(landing={f: 0.0 for f in p.floors()})
        if s.lift_pos is None or s.car_door is None:
            self._v = 0.0                 # 関節の状態がまだ届いていない: 何も動かさない
            return cmd
        if s.obstructed:
            self._t_obstructed = now
        travel = p.door_travel
        # 閉じているかは、どのパネルも閉じ切っていること（開きが大きい方）で判定する
        car_max = s.car_door if s.car_door_max is None else s.car_door_max
        closed = car_max <= cfg.door_tol and all(
            s.landing_max.get(f, s.landing.get(f, 0.0)) <= cfg.door_tol for f in p.floors())

        if self.phase == IDLE:
            target_floor: Optional[int] = None
            if self.stops and now >= self._ready_at:
                target_floor = self.stops[0]
            elif not self.stops and self.return_floor is not None:
                target_floor = self.return_floor
            if target_floor is not None and now - self._t_phase >= cfg.depart_delay_s:
                self.lift_target = p.floor_z(target_floor)
                if abs(s.lift_pos - self.lift_target) <= cfg.level_tol:
                    if self.stops:
                        self.serving = target_floor
                        self._enter(OPENING, now)
                    else:
                        self.return_floor = None
                elif closed:
                    self._enter(MOVING, now)

        if self.phase == MOVING:
            error = self.lift_target - s.lift_pos
            if not closed:
                self._v = lift_velocity(0.0, self._v, dt, p.lift_speed, p.lift_accel, cfg.lift_kp)
            else:
                self._v = lift_velocity(error, self._v, dt, p.lift_speed, p.lift_accel, cfg.lift_kp)
            if abs(error) <= cfg.level_tol and abs(self._v) <= 0.02:
                self._v = 0.0
                floor = self._level_floor(s.lift_pos)
                if self.stops and floor == self.stops[0]:
                    self.serving = floor
                    self._enter(OPENING, now)
                else:
                    if floor is not None and floor == self.return_floor:
                        self.return_floor = None
                    self._enter(IDLE, now)
        else:
            # 停止中は、着床位置からのずれだけをゆっくり直す
            error = self.lift_target - s.lift_pos
            self._v = lift_velocity(error if abs(error) > 0.5 * cfg.level_tol else 0.0,
                                    self._v, dt, 0.05, p.lift_accel, cfg.lift_kp)
        cmd.lift_vel = self._v

        if self.phase in (OPENING, OPEN, CLOSING) and self.serving is not None:
            floor = self.serving
            is_open = s.car_door >= travel - cfg.door_tol and s.landing.get(floor, 0.0) >= travel - cfg.door_tol
            if self.phase == OPENING:
                cmd.car_door = travel
                cmd.landing[floor] = travel
                if is_open or now - self._t_phase > cfg.door_timeout_s:
                    self._t_open = now
                    self._enter(OPEN, now)
            elif self.phase == OPEN:
                cmd.car_door = travel
                cmd.landing[floor] = travel
                clear = (not cfg.door_sensor) or (now - self._t_obstructed >= cfg.door_clear_s)
                if now - self._t_open >= cfg.door_hold_s and clear and not self.hold_open:
                    self._enter(CLOSING, now)
            else:  # CLOSING
                if (cfg.door_sensor and s.obstructed) or self.hold_open:
                    self.reopen_count += 1
                    cmd.car_door = travel
                    cmd.landing[floor] = travel
                    self._enter(OPENING, now)
                elif closed or now - self._t_phase > cfg.door_timeout_s:
                    self.door_cycles += 1
                    if self.stops and self.stops[0] == floor:
                        self.stops.pop(0)
                    self.serving = None
                    self._enter(IDLE, now)
        return cmd

    # -- 表示用 -------------------------------------------------------------
    def state(self) -> Dict[str, Any]:
        s = self._last
        lift = s.lift_pos
        return {
            "phase": self.phase,
            "floor": None if lift is None else self._level_floor(lift),
            "lift_pos": None if lift is None else round(lift, 4),
            "lift_vel": round(self._v, 4),
            "car_door_ratio": None if s.car_door is None else round(s.car_door / self.p.door_travel, 3),
            "serving": self.serving,
            "stops": list(self.stops),
            "hold_open": self.hold_open,
            "obstructed": s.obstructed,
            "door_cycles": self.door_cycles,
            "reopen_count": self.reopen_count,
        }
