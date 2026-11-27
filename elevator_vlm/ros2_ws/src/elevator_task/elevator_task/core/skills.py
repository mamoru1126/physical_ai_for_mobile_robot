"""スキル: 旋回、直進、点への移動、待機、道に沿って走る. どれもオドメトリで閉ループ制御する.

対象（扉、かご、エレベーター）の知識は持たない. 「どこへ」は指差しの結果から、
「どれだけ」は指示文の数値から来る. 経路計画と障害物回避は入れていない（障害物で止めるのは後ろの安全ゲート）.

道に沿って走る（follow_path）の「1回でどれだけ進むか」:
  * VLM の答え（道の指差し）1回で進めるのは、最大 follow_path.step_m（既定 1.0 m）.
    その答えに使った画像を撮った時点の位置から、走った道のりで数える.
  * 次の答えを待つ間も走り続けるが、step_m を進み切っても次の答えが来なければ、止まって待つ.
    VLM が遅くても、見ていない所を step_m より先へは走らない.
  * 手順1つで走る距離は、計画の distance_m（0 なら上限 path_cap_m まで. 止まる条件か終了の指示で終える）.
"""
from __future__ import annotations

import bisect
import math
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from .events import EventLog
from .geometry import CameraModel, Pose2D, in_image, pixel_to_ground, robot_to_odom, wrap_angle
from .perception import CheckResult, Perception, PointResult
from .plan import (SKILL_FOLLOW_PATH, SKILL_GO_TO_VISUAL_TARGET, SKILL_MOVE_STRAIGHT, SKILL_ROTATE, SKILL_WAIT,
                   Action)
from .robot_io import RobotIO


@dataclass
class FollowPathConfig:
    """道に沿って走る（follow_path）の設定. 「VLM の答え1回で進める距離」は step_m."""

    step_m: float = 1.0               # VLM の答え（道の指差し）1回で進める距離の上限 [m]. その画像を撮った時点の
                                      # 位置から数える. 次の答えが来ないまま進み切ったら、止まって待つ
    speed: float = 0.3                # 走る速さの上限 [m/s]（skills.max_linear も超えない）
    k_heading: float = 1.5            # 向きのずれ [rad] に掛けて、旋回の速さ [rad/s] にする
    turn_in_place_deg: float = 35.0   # 向きのずれがこれより大きければ、止まって向き直してから進む
    aim_max_m: float = 5.0            # これより遠くを指したら方向だけを使う（舵を切る目印をこの距離に置く）
    min_point_m: float = 0.3          # これより近く（ロボットの真下）を指した答えは使わない
    max_bearing_deg: float = 60.0     # これより横を指した答えは使わない
    max_point_failures: int = 6       # 使えない答え（読めない、地平線より上、など）がこれだけ続いたら中止
    max_answer_wait_s: float = 30.0   # 使える答えがこれだけの時間来なければ中止（カメラか VLM が止まっている）
    points_per_check: int = 1         # 止まる条件があるとき、指差し N 回ごとに 1 回、条件を判定する
    stop_confirm_count: int = 2       # 止まる条件に yes がこの回数続いたら止まる
    arrive_tol_m: float = 0.08        # 指差した点（か、進める距離の端）にここまで近づいたら、次の答えを待つ
    stall_s: float = 8.0              # 進めと言っても進まない時間がこれを超えたら、記録と画面に出す（安全ゲートが止めている、など）


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
    # 道に沿って走る
    follow_path: FollowPathConfig = field(default_factory=FollowPathConfig)


@dataclass
class SkillResult:
    ok: bool
    message: str = ""
    retry_condition: bool = False      # True なら、条件の確認からやり直す価値がある失敗
    data: Dict[str, Any] = field(default_factory=dict)
    finished: bool = False             # 終了の指示（ここまで）で打ち切った


class _Stop(Exception):
    def __init__(self, reason: str, finished: bool = False):
        super().__init__(reason)
        self.reason = reason
        self.finished = finished


class Skills:
    def __init__(self, robot: RobotIO, perception: Perception, cfg: SkillConfig, events: EventLog,
                 cancel: threading.Event, finish: Optional[threading.Event] = None, path_cap_m: float = 100.0):
        self.robot = robot
        self.perception = perception
        self.cfg = cfg
        self.events = events
        self.cancel = cancel
        self.finish = finish if finish is not None else threading.Event()   # 終了の指示（ここまで）
        self.path_cap_m = path_cap_m       # 道に沿って走る1手順の距離の上限（distance_m が 0 のとき）
        self.current_target: Optional[Tuple[float, float]] = None   # 画面表示用（オドメトリ座標）
        self.path_progress: Optional[Dict[str, Any]] = None          # 画面表示用（道に沿って走る進み具合）
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
            if action.skill == SKILL_FOLLOW_PATH:
                return self.follow_path(action, deadline)
            return SkillResult(False, f"知らないスキルです: {action.skill}")
        except _Stop as e:
            return SkillResult(e.finished, e.reason, finished=e.finished)
        finally:
            self.current_target = None
            self.robot.send_cmd(0.0, 0.0)

    # -- 共通 ---------------------------------------------------------------
    def _guard(self, deadline: float) -> None:
        if self.cancel.is_set():
            raise _Stop("停止が要求されました")
        if self.finish.is_set():
            raise _Stop("終了の指示（ここまで）で止めました", finished=True)
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

    # -- 道に沿って走る -------------------------------------------------------
    def _path_answer(self, cam: CameraModel, res: PointResult
                     ) -> Tuple[Optional[Tuple[Pose2D, float, float]], str]:
        """道の指差しを検査し、(撮影した時点の位置, 指した方向 [rad], 指した点までの距離 [m]) にする."""
        cfg = self.cfg.follow_path
        if not res.ok:
            return None, res.error or "点が返っていません"
        if not in_image(cam, res.u, res.v, self.cfg.image_margin_px):
            return None, f"点が画像の外か端にあります（u={res.u:.0f}, v={res.v:.0f}）"
        ground = pixel_to_ground(cam, res.u, res.v)
        if ground is None:
            return None, f"点が地平線より上にあります（v={res.v:.0f}、地平線 v={cam.horizon_v():.0f}）"
        dist = math.hypot(ground[0], ground[1])
        bearing = math.atan2(ground[1], ground[0])
        self.events.emit("point_ground", image=res.image, ahead_m=round(ground[0], 3), left_m=round(ground[1], 3))
        if dist < cfg.min_point_m:
            return None, f"点が近すぎます（{dist:.2f} m）"
        if abs(bearing) > math.radians(cfg.max_bearing_deg):
            return None, f"点が横にずれすぎています（{math.degrees(bearing):.0f} 度）"
        pose = self.robot.pose_at(res.stamp)
        if pose is None:
            return None, "撮影時刻のオドメトリがありません"
        return (pose, bearing, dist), ""

    def follow_path(self, action: Action, deadline: float) -> SkillResult:
        """道に沿って走る. VLM に道の上の次の点を指差させ続け、答え1回につき最大 step_m だけその方へ進む.

        終わり方: 走った道のりが distance_m に達した（0 なら path_cap_m）、止まる条件に yes が続いた、
        終了の指示（ここまで）、停止、時間切れ、使える答えが来ない.
        """
        cfg = self.cfg
        pcfg = cfg.follow_path
        cam = self.robot.get_camera()
        if cam is None:
            return SkillResult(False, "カメラ情報（内部パラメータ）をまだ受け取っていません", retry_condition=True)
        limit = action.distance_m if action.distance_m > 0.0 else self.path_cap_m
        speed = min(pcfg.speed, cfg.max_linear)
        start = self._pose(deadline)
        started_at = self.robot.now()
        progress: Dict[str, Any] = {
            "active": True, "path": action.path, "stop_when": action.stop_when,
            "distance_m": action.distance_m, "limit_m": limit, "step_m": pcfg.step_m,
            "traveled_m": 0.0, "allow_m": 0.0, "answers": 0, "accepted": 0, "rejected": 0,
            "checks": 0, "stop_streak": 0, "last_latency_s": None, "state": "waiting", "reason": "", "result": "",
        }
        self.path_progress = progress
        self.events.emit("path_start", path=action.path, distance_m=action.distance_m, limit_m=limit,
                         stop_when=action.stop_when, step_m=pcfg.step_m, t=round(started_at, 3))

        worker = _PathWorker(self.robot, self.perception, action.path, action.stop_when, started_at,
                             max(pcfg.points_per_check, 1))
        worker.start()
        travelled = 0.0
        last_xy = (start.x, start.y)
        stamps: List[float] = [start.stamp]          # 走った道のりの履歴（撮影時刻での道のりを引くため）
        dists: List[float] = [0.0]
        goal: Optional[Tuple[float, float]] = None   # 進んでよい点（指した点か、進める距離の端）
        aim: Optional[Tuple[float, float]] = None    # 舵を切る目印（指した点. 遠ければ aim_max_m に縮める）
        allow_until = 0.0                            # ここまでの道のりなら進んでよい
        failures = 0
        streak = 0
        last_good = started_at
        moving_since: Optional[float] = None
        moved_at_cmd = 0.0
        stalled = False

        def end(ok: bool, message: str) -> SkillResult:
            progress.update(active=False, result=message, state="done" if ok else "failed",
                            traveled_m=round(travelled, 2), allow_m=0.0)
            self.events.emit("path_end", ok=ok, message=message, traveled_m=round(travelled, 3),
                             answers=progress["answers"], accepted=progress["accepted"], checks=progress["checks"],
                             t=round(self.robot.now(), 3))
            return SkillResult(ok, message, data={"traveled_m": travelled})

        try:
            while True:
                self._guard(deadline)
                pose = self._pose(deadline)
                travelled += math.hypot(pose.x - last_xy[0], pose.y - last_xy[1])
                last_xy = (pose.x, pose.y)
                if pose.stamp > stamps[-1]:
                    stamps.append(pose.stamp)
                    dists.append(travelled)
                now = self.robot.now()

                # 1. 届いた答えを使う
                while True:
                    try:
                        kind, res = worker.results.get_nowait()
                    except queue.Empty:
                        break
                    if kind == "check":
                        chk: CheckResult = res
                        progress["checks"] += 1
                        if chk.answer is True:
                            streak += 1
                        elif chk.answer is False:
                            streak = 0
                        progress["stop_streak"] = streak
                        if streak >= max(pcfg.stop_confirm_count, 1):
                            self.robot.send_cmd(0.0, 0.0)
                            self.events.emit("path_stop_condition", condition=action.stop_when,
                                             traveled_m=round(travelled, 3), t=round(now, 3))
                            return end(True, f"止まる条件が成り立ちました（{travelled:.1f} m 走りました）")
                        continue
                    pt: PointResult = res
                    progress["answers"] += 1
                    if pt.latency_s:
                        progress["last_latency_s"] = round(pt.latency_s, 2)
                    answer, why = self._path_answer(cam, pt)
                    if answer is None:
                        failures += 1
                        progress["rejected"] += 1
                        progress["reason"] = why
                        self.events.emit("path_point_rejected", reason=why, failures=failures)
                        if failures >= max(pcfg.max_point_failures, 1):
                            return end(False, f"道の指差しが続けて使えません（{failures} 回）: {why}")
                        continue
                    failures = 0
                    last_good = now
                    shot, bearing, dist = answer
                    # 撮影した時点までに走っていた道のり. そこから step_m までなら進んでよい
                    i = bisect.bisect_left(stamps, shot.stamp)
                    if i <= 0:
                        at_shot = dists[0]
                    elif i >= len(stamps):
                        at_shot = dists[-1]
                    else:
                        span = stamps[i] - stamps[i - 1]
                        r = 0.0 if span <= 0.0 else (shot.stamp - stamps[i - 1]) / span
                        at_shot = dists[i - 1] + r * (dists[i] - dists[i - 1])
                    c, s = math.cos(bearing), math.sin(bearing)
                    reach = min(dist, pcfg.step_m)
                    goal = robot_to_odom(shot, reach * c, reach * s)
                    far = min(dist, pcfg.aim_max_m)
                    aim = robot_to_odom(shot, far * c, far * s)
                    allow_until = at_shot + pcfg.step_m
                    progress["accepted"] += 1
                    progress["reason"] = ""
                    self.current_target = goal
                    self.events.emit("path_target", bearing_deg=round(math.degrees(bearing), 1),
                                     point_m=round(dist, 2), reach_m=round(reach, 2),
                                     allow_until_m=round(allow_until, 2), traveled_m=round(travelled, 2),
                                     x=round(goal[0], 3), y=round(goal[1], 3), t=round(now, 3))

                # 2. 終わったか（止まり際は arrive_tol_m の手前で止まって待つので、その分は走ったとみなす）
                if travelled >= limit - max(cfg.dist_tol_m, pcfg.arrive_tol_m):
                    self.robot.send_cmd(0.0, 0.0)
                    if action.distance_m > 0.0:
                        return end(True, f"道に沿って {travelled:.1f} m 走りました")
                    return end(True, f"距離の上限（{limit:g} m）まで走りました")
                if now - last_good > pcfg.max_answer_wait_s:
                    return end(False, f"使える答えが {pcfg.max_answer_wait_s:g} 秒来ません（カメラ画像か VLM を確かめてください）")

                # 3. 進む
                allow = allow_until - travelled
                progress.update(traveled_m=round(travelled, 2), allow_m=round(max(allow, 0.0), 2))
                linear = 0.0
                if goal is None or aim is None:
                    self.robot.send_cmd(0.0, 0.0)
                    progress["state"] = "waiting"
                else:
                    gx, gy = goal[0] - pose.x, goal[1] - pose.y
                    cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)
                    ahead = gx * cy + gy * sy                 # 進んでよい点が、今の向きで前にどれだけあるか
                    remaining = min(allow, math.hypot(gx, gy), limit - travelled)
                    if remaining <= pcfg.arrive_tol_m or ahead <= pcfg.arrive_tol_m:
                        self.robot.send_cmd(0.0, 0.0)
                        progress["state"] = "holding"         # 次の答えを待つ
                    else:
                        err = wrap_angle(math.atan2(aim[1] - pose.y, aim[0] - pose.x) - pose.yaw)
                        w = self._clamp(pcfg.k_heading * err, cfg.max_angular)
                        if abs(err) > math.radians(pcfg.turn_in_place_deg):
                            if abs(w) < cfg.min_angular:
                                w = math.copysign(cfg.min_angular, err)
                            self.robot.send_cmd(0.0, w)
                            progress["state"] = "turning"
                        else:
                            # 進んでよい距離の端（の arrive_tol_m 手前）で止まれるように、早めに減速する
                            brake = max(remaining - pcfg.arrive_tol_m, 0.0)
                            linear = max(min(speed, math.sqrt(2.0 * cfg.linear_decel * brake)), cfg.min_linear)
                            self.robot.send_cmd(linear, w)
                            progress["state"] = "driving"

                # 4. 進めと言っているのに進まない（安全ゲートが止めている、など）
                if linear > 0.0:
                    if moving_since is None:
                        moving_since, moved_at_cmd = now, travelled
                    elif travelled - moved_at_cmd > 0.05:
                        moving_since, moved_at_cmd = now, travelled
                        if stalled:
                            stalled = False
                            self.events.emit("path_resumed", traveled_m=round(travelled, 3), t=round(now, 3))
                    elif not stalled and now - moving_since > pcfg.stall_s:
                        stalled = True
                        self.events.emit("path_stalled", traveled_m=round(travelled, 3), t=round(now, 3))
                else:
                    moving_since = None
                if stalled:
                    progress["state"] = "stalled"
                self.robot.sleep(self._period)
        except _Stop as e:
            # 停止、終了の指示、時間切れ、オドメトリの途絶. 記録を残してから上に伝える
            progress.update(active=False, result=e.reason, traveled_m=round(travelled, 2), allow_m=0.0,
                            state="done" if e.finished else "failed")
            self.events.emit("path_end", ok=e.finished, message=e.reason, traveled_m=round(travelled, 3),
                             answers=progress["answers"], accepted=progress["accepted"], checks=progress["checks"],
                             t=round(self.robot.now(), 3))
            raise
        finally:
            worker.halt.set()
            progress["active"] = False


class _PathWorker(threading.Thread):
    """道に沿って走る間、VLM への問いかけを続ける（走りながら次の答えを用意する）.

    新しい画像が届くたびに1回問いかける. 止まる条件があれば、最初と、指差し points_per_check 回ごとに
    1回は条件の判定にする. 答えは results に (種類, 結果) で入れる.
    """

    def __init__(self, robot: RobotIO, perception: Perception, path: str, stop_when: str, since: float,
                 points_per_check: int):
        super().__init__(name="follow_path", daemon=True)
        self.robot = robot
        self.perception = perception
        self.path = path
        self.stop_when = stop_when
        self.since = since
        self.points_per_check = points_per_check
        self.results: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self.halt = threading.Event()

    def run(self) -> None:
        last_stamp: Optional[float] = None
        points_since_check = self.points_per_check if self.stop_when else 0   # 条件があれば最初に判定する
        max_age = self.perception.cfg.max_image_age_s
        while not self.halt.is_set():
            frame = self.robot.get_frame()
            if (frame is None or frame.stamp < self.since or (last_stamp is not None and frame.stamp <= last_stamp)
                    or self.robot.now() - frame.stamp > max_age):
                time.sleep(0.005)        # 新しい画像を待つ（実時間. 擬似世界の時刻は進めない）
                continue
            last_stamp = frame.stamp
            try:
                if self.stop_when and points_since_check >= self.points_per_check:
                    points_since_check = 0
                    kind = "check"
                    res: Any = self.perception.check(self.stop_when, frame=frame, purpose="stop_check")
                else:
                    points_since_check += 1
                    kind = "point"
                    res = self.perception.point_path(self.path, frame=frame)
            except Exception as e:  # noqa: BLE001  想定外の失敗も、使えない答えとして数える（スレッドを止めない）
                res = (CheckResult(None, stamp=frame.stamp, error=repr(e)) if kind == "check"
                       else PointResult(False, stamp=frame.stamp, error=f"問いかけに失敗しました: {e!r}"))
            if not self.halt.is_set():
                self.results.put((kind, res))
