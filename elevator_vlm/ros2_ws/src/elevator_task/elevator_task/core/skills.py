"""スキル: 旋回、直進、点への移動、待機. どれもオドメトリで閉ループ制御する.

対象（扉、かご、エレベーター）の知識は持たない. 「どこへ」は指差しの結果から、
「どれだけ」は指示文の数値から来る. 経路計画と障害物回避は入れていない（正常系のみ）.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from .events import EventLog
from .geometry import CameraModel, Pose2D, in_image, pixel_to_ground, robot_to_odom, wrap_angle
from .perception import Perception, PointResult
from .plan import SKILL_GO_TO_VISUAL_TARGET, SKILL_MOVE_STRAIGHT, SKILL_ROTATE, SKILL_WAIT, Action
from .robot_io import RobotIO


@dataclass
class SkillConfig:
    rate_hz: float = 20.0
    max_linear: float = 0.3            # 並進の上限 [m/s]
    max_angular: float = 0.8           # 旋回の上限 [rad/s]
    linear_decel: float = 0.4          # 減速の目安 [m/s^2]
    angular_decel: float = 1.5         # [rad/s^2]
    min_linear: float = 0.04           # 止まり際に張り付かないための最低速度
    min_angular: float = 0.12
    yaw_tol_deg: float = 1.5
    dist_tol_m: float = 0.03
    k_heading: float = 2.0
    k_lateral: float = 1.5
    max_odom_age_s: float = 0.5
    max_skill_travel_m: float = 3.5    # 1回のスキルで動ける距離の上限
    # 点への移動（指差し）
    max_target_distance_m: float = 3.0
    min_target_distance_m: float = 0.2
    image_margin_px: float = 2.0
    point_retries: int = 3
    face_threshold_deg: float = 4.0
    verify_point: bool = False         # 指差した点が床の上かを VLM にもう一度尋ねる
    departure_deadline_s: float = 6.0  # 条件の確定から発進までに許す時間. 超えたら条件を確かめ直す
    repoint_period_s: float = 1.0      # 走行中に指差しをやり直す周期. 0 で無効
    repoint_min_remaining_m: float = 0.8
    repoint_gate_m: float = 0.5        # 今の目標からこれ以上離れた点は採用しない
    repoint_max_shift_m: float = 0.4   # 最初の目標からこれ以上は動かさない


@dataclass
class SkillResult:
    ok: bool
    message: str = ""
    retry_condition: bool = False      # True なら、条件の確認からやり直す価値がある失敗
    data: Dict[str, Any] = field(default_factory=dict)


class _Stop(Exception):
    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class Skills:
    def __init__(self, robot: RobotIO, perception: Perception, cfg: SkillConfig, events: EventLog,
                 cancel: threading.Event):
        self.robot = robot
        self.perception = perception
        self.cfg = cfg
        self.events = events
        self.cancel = cancel
        self.current_target: Optional[Tuple[float, float]] = None   # 画面表示用（オドメトリ座標）
        self._period = 1.0 / max(cfg.rate_hz, 1.0)

    # -- 入口 ---------------------------------------------------------------
    def run(self, action: Action, deadline: float, confirmed_at: Optional[float] = None,
            recheck: Optional[Callable[[], bool]] = None) -> SkillResult:
        """スキルを1つ実行する. recheck は、発進の直前に条件をもう一度確かめる関数（False なら発進しない）."""
        try:
            if action.skill == SKILL_ROTATE:
                return self.rotate(math.radians(action.angle_deg), deadline)
            if action.skill == SKILL_MOVE_STRAIGHT:
                return self.move_straight(action.distance_m, deadline)
            if action.skill == SKILL_GO_TO_VISUAL_TARGET:
                return self.go_to_visual_target(action.target, deadline, confirmed_at, recheck)
            if action.skill == SKILL_WAIT:
                return self.wait(action.seconds, deadline)
            return SkillResult(False, f"知らないスキルです: {action.skill}")
        except _Stop as e:
            return SkillResult(False, e.reason)
        finally:
            self.current_target = None
            self.robot.send_cmd(0.0, 0.0)

    # -- 共通 ---------------------------------------------------------------
    def _guard(self, deadline: float) -> None:
        if self.cancel.is_set():
            raise _Stop("停止が要求されました")
        if self.robot.now() > deadline:
            raise _Stop("時間切れです")

    def _pose(self, deadline: float) -> Pose2D:
        """新しいオドメトリを返す. 途絶えている間は止まって待ち、2秒続いたら中止する."""
        waited = 0.0
        while True:
            self._guard(deadline)
            pose = self.robot.get_pose()
            if pose is not None and self.robot.now() - pose.stamp <= self.cfg.max_odom_age_s:
                return pose
            self.robot.send_cmd(0.0, 0.0)
            if waited >= 2.0:
                raise _Stop("オドメトリが途絶えました")
            self.robot.sleep(self._period)
            waited += self._period

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    # -- 旋回 ---------------------------------------------------------------
    def rotate(self, angle_rad: float, deadline: float) -> SkillResult:
        cfg = self.cfg
        tol = math.radians(cfg.yaw_tol_deg)
        pose = self._pose(deadline)
        prev_yaw = pose.yaw
        turned = 0.0
        settled = 0
        while True:
            pose = self._pose(deadline)
            turned += wrap_angle(pose.yaw - prev_yaw)
            prev_yaw = pose.yaw
            err = angle_rad - turned
            if abs(err) <= tol:
                self.robot.send_cmd(0.0, 0.0)
                settled += 1
                if settled >= 3:
                    return SkillResult(True, f"{math.degrees(turned):.1f} 度旋回しました",
                                       data={"turned_deg": math.degrees(turned)})
            else:
                settled = 0
                limit = min(cfg.max_angular, math.sqrt(2.0 * cfg.angular_decel * abs(err)))
                w = self._clamp(cfg.k_heading * err, max(limit, cfg.min_angular))
                if abs(w) < cfg.min_angular:
                    w = math.copysign(cfg.min_angular, err)
                self.robot.send_cmd(0.0, w)
            self.robot.sleep(self._period)

    # -- 直進 ---------------------------------------------------------------
    def move_straight(self, distance_m: float, deadline: float) -> SkillResult:
        start = self._pose(deadline)
        direction = 1.0 if distance_m >= 0.0 else -1.0
        gx, gy = robot_to_odom(start, distance_m, 0.0)
        self._drive_line(start, [gx, gy], deadline, direction=direction, hold_yaw=start.yaw)
        return SkillResult(True, f"{distance_m:+.2f} m 直進しました")

    def _speed_for(self, remaining: float) -> float:
        cfg = self.cfg
        v = min(cfg.max_linear, math.sqrt(2.0 * cfg.linear_decel * max(remaining, 0.0)))
        return max(v, cfg.min_linear)

    def _drive_line(self, start: Pose2D, goal: List[float], deadline: float, direction: float = 1.0,
                    hold_yaw: Optional[float] = None, on_tick=None) -> None:
        """start から goal（オドメトリ座標、途中で書き換えてよい）への線分に沿って走る."""
        cfg = self.cfg
        origin = (start.x, start.y)
        travelled = 0.0
        last = origin
        while True:
            pose = self._pose(deadline)
            travelled += math.hypot(pose.x - last[0], pose.y - last[1])
            last = (pose.x, pose.y)
            if travelled > cfg.max_skill_travel_m:
                raise _Stop("1回のスキルで動ける距離の上限を超えました")
            if on_tick is not None and on_tick(pose):
                origin = (pose.x, pose.y)        # 目標が更新された: 今の位置から引き直す
            seg_x, seg_y = goal[0] - origin[0], goal[1] - origin[1]
            length = math.hypot(seg_x, seg_y)
            if hold_yaw is not None:
                line_yaw = hold_yaw
            elif length > 1e-6:
                line_yaw = math.atan2(seg_y, seg_x)
            else:
                line_yaw = pose.yaw
            c, s = math.cos(line_yaw), math.sin(line_yaw)
            dx, dy = pose.x - origin[0], pose.y - origin[1]
            along = (dx * c + dy * s) * direction
            lateral = -dx * s + dy * c
            remaining = length - along
            if remaining <= cfg.dist_tol_m:
                self.robot.send_cmd(0.0, 0.0)
                return
            yaw_err = wrap_angle(line_yaw - pose.yaw)
            if abs(yaw_err) > math.radians(30.0):
                # 向きが大きくずれている間は、その場で向き直す
                self.robot.send_cmd(0.0, self._clamp(cfg.k_heading * yaw_err, cfg.max_angular))
            else:
                w = cfg.k_heading * yaw_err - cfg.k_lateral * lateral * direction
                self.robot.send_cmd(direction * self._speed_for(remaining),
                                    self._clamp(w, 0.6 * cfg.max_angular))
            self.robot.sleep(self._period)

    # -- 待機 ---------------------------------------------------------------
    def wait(self, seconds: float, deadline: float) -> SkillResult:
        end = self.robot.now() + seconds
        while self.robot.now() < end:
            self._guard(deadline)
            self.robot.send_cmd(0.0, 0.0)
            self.robot.sleep(self._period)
        return SkillResult(True, f"{seconds:g} 秒待ちました")

    # -- 点への移動（指差し） -------------------------------------------------
    def _ground_goal(self, cam: CameraModel, res: PointResult) -> Tuple[Optional[Tuple[float, float]], str]:
        """指差しの結果を検査し、オドメトリ座標の目標点にする. 検査に落ちたら (None, 理由)."""
        if not res.ok:
            return None, res.error or "点が返っていません"
        if not in_image(cam, res.u, res.v, self.cfg.image_margin_px):
            return None, f"点が画像の外か端にあります（u={res.u:.0f}, v={res.v:.0f}）"
        ground = pixel_to_ground(cam, res.u, res.v)
        if ground is None:
            return None, f"点が地平線より上にあります（v={res.v:.0f}、地平線 v={cam.horizon_v():.0f}）"
        # 指差した点が、撮影した時点のロボットから見て床のどこに当たるか（前, 左）. 記録の分析に使う
        self.events.emit("point_ground", image=res.image, ahead_m=round(ground[0], 3), left_m=round(ground[1], 3))
        dist = math.hypot(ground[0], ground[1])
        if dist > self.cfg.max_target_distance_m:
            return None, f"目標が遠すぎます（{dist:.2f} m > {self.cfg.max_target_distance_m:g} m）"
        if dist < self.cfg.min_target_distance_m:
            return None, f"目標が近すぎます（{dist:.2f} m）"
        # 推論の間もロボットは進むので、画像を撮った時刻の位置で直す
        pose = self.robot.pose_at(res.stamp)
        if pose is None:
            return None, "撮影時刻のオドメトリがありません"
        return robot_to_odom(pose, ground[0], ground[1]), ""

    def go_to_visual_target(self, target: str, deadline: float, confirmed_at: Optional[float] = None,
                            recheck: Optional[Callable[[], bool]] = None) -> SkillResult:
        cfg = self.cfg
        cam = self.robot.get_camera()
        if cam is None:
            return SkillResult(False, "カメラ情報（内部パラメータ）をまだ受け取っていません", retry_condition=True)

        # 1. 発進の前に指差しで目標を決める
        goal: Optional[Tuple[float, float]] = None
        reason = ""
        for _ in range(max(cfg.point_retries, 1)):
            self._guard(deadline)
            frame = self.robot.get_frame()
            if frame is None or self.robot.now() - frame.stamp > self.perception.cfg.max_image_age_s:
                reason = "新しいカメラ画像がありません"
                self.robot.sleep(self._period)
                continue
            res = self.perception.point(target, frame)
            goal, reason = self._ground_goal(cam, res)
            if goal is None:
                self.events.emit("point_rejected", target=target, reason=reason)
                continue
            if cfg.verify_point:
                chk = self.perception.verify_floor(frame, res.u, res.v)
                if chk.answer is not True:
                    goal, reason = None, "指差した点が床の上だと確認できません"
                    self.events.emit("point_rejected", target=target, reason=reason)
                    continue
            break
        if goal is None:
            return SkillResult(False, f"指差しが検査に通りません: {reason}", retry_condition=True)
        if confirmed_at is not None and self.robot.now() - confirmed_at > cfg.departure_deadline_s:
            return SkillResult(False, "条件の確定から発進までに時間がかかりすぎました", retry_condition=True)
        # 指差しの間に状況が変わっているかもしれない（扉が閉まり始めた、など）. 発進の直前にもう一度確かめる
        if recheck is not None and not recheck():
            return SkillResult(False, "発進の直前に確かめたら、条件が成り立っていませんでした", retry_condition=True)

        first = goal
        target_xy = [goal[0], goal[1]]
        self.current_target = (goal[0], goal[1])
        self.events.emit("target_set", target=target, x=round(goal[0], 3), y=round(goal[1], 3))

        # 2. 目標の方を向く
        pose = self._pose(deadline)
        bearing = wrap_angle(math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw)
        if abs(bearing) > math.radians(cfg.face_threshold_deg):
            self.rotate(bearing, deadline)

        # 3. 走りながら、目標が見えている間は指差しをやり直して補正する
        repoint: Dict[str, Any] = {"thread": None, "result": None, "last": self.robot.now()}

        def worker() -> None:
            repoint["result"] = self.perception.point(target, purpose="repoint")

        def on_tick(p: Pose2D) -> bool:
            updated = False
            thread = repoint["thread"]
            if thread is not None and not thread.is_alive():
                repoint["thread"] = None
                res, repoint["result"] = repoint["result"], None
                if res is not None:
                    new, why = self._ground_goal(cam, res)
                    if new is None:
                        self.events.emit("repoint_rejected", reason=why)
                    elif math.hypot(new[0] - target_xy[0], new[1] - target_xy[1]) > cfg.repoint_gate_m:
                        self.events.emit("repoint_rejected", reason="今の目標から離れすぎています")
                    elif math.hypot(new[0] - first[0], new[1] - first[1]) > cfg.repoint_max_shift_m:
                        self.events.emit("repoint_rejected", reason="最初の目標から離れすぎています")
                    else:
                        target_xy[0], target_xy[1] = new
                        self.current_target = (new[0], new[1])
                        self.events.emit("target_updated", x=round(new[0], 3), y=round(new[1], 3))
                        updated = True
            remaining = math.hypot(target_xy[0] - p.x, target_xy[1] - p.y)
            if (cfg.repoint_period_s > 0.0 and repoint["thread"] is None
                    and self.robot.now() - repoint["last"] >= cfg.repoint_period_s
                    and remaining >= cfg.repoint_min_remaining_m):
                repoint["last"] = self.robot.now()
                repoint["thread"] = threading.Thread(target=worker, name="repoint", daemon=True)
                repoint["thread"].start()
            return updated

        start = self._pose(deadline)
        self._drive_line(start, target_xy, deadline, on_tick=on_tick)
        return SkillResult(True, "目標点に着きました",
                           data={"x": target_xy[0], "y": target_xy[1]})
