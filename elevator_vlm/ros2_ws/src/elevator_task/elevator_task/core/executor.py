"""計画を上から順に進める小さな状態機械.

各手順は「条件待ち」→「動作中」→「完了」と進み、時間切れか異常で「中止」に移る.
「再び扉が開いたら」のような履歴に依存する条件は、モデルの記憶ではなくここで持つ状態
（成り立たないのを見たか）で判定する.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .events import EventLog
from .perception import Perception
from .plan import WHEN_AFTER_FALSE_THEN_TRUE, WHEN_NOW, Plan, Step
from .robot_io import RobotIO
from .skills import SkillResult, Skills

PHASE_PENDING = "pending"        # まだ始まっていない
PHASE_WAITING = "waiting"        # 条件待ち
PHASE_ACTING = "acting"          # 動作中
PHASE_DONE = "done"              # 完了
PHASE_FAILED = "failed"          # 中止

PATH_CLEAR_STATEMENT = "The path straight ahead of the robot is clear of people and obstacles."


@dataclass
class ExecutorConfig:
    confirm_count: int = 2           # 同じ答えがこの回数続いたら確定する
    check_period_s: float = 0.5      # 判定の周期（推論がこれより遅ければ推論の速さで回る）
    max_check_errors: int = 20       # 判定の失敗（通信など）がこの回数続いたら中止
    yes_threshold: float = 0.0       # 0 より大きければ、yes の確率がこの値以上のときだけ yes とみなす
    pre_departure_check: bool = False  # 発進の直前に「進路に人や物がないか」を VLM に確かめる
    pre_departure_retries: int = 3


@dataclass
class StepStatus:
    phase: str = PHASE_PENDING
    seen_false: bool = False         # after_false_then_true で「成り立たない」を確定したか
    streak_answer: Optional[bool] = None
    streak: int = 0
    checks: int = 0
    started_at: Optional[float] = None
    confirmed_at: Optional[float] = None
    finished_at: Optional[float] = None
    message: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "phase": self.phase, "seen_false": self.seen_false, "checks": self.checks,
            "streak": self.streak,
            "streak_answer": None if self.streak_answer is None else ("yes" if self.streak_answer else "no"),
            "started_at": self.started_at, "confirmed_at": self.confirmed_at,
            "finished_at": self.finished_at, "message": self.message,
        }


@dataclass
class TaskResult:
    ok: bool
    message: str = ""
    failed_step: Optional[int] = None
    steps: List[StepStatus] = field(default_factory=list)


class TaskExecutor:
    def __init__(self, robot: RobotIO, perception: Perception, skills: Skills, cfg: ExecutorConfig,
                 events: EventLog, cancel: threading.Event):
        self.robot = robot
        self.perception = perception
        self.skills = skills
        self.cfg = cfg
        self.events = events
        self.cancel = cancel
        self.plan: Optional[Plan] = None
        self.statuses: List[StepStatus] = []
        self.current: int = -1
        self._lock = threading.Lock()

    # -- 画面表示用 -----------------------------------------------------------
    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {"current": self.current, "steps": [s.to_dict() for s in self.statuses]}

    # -- 実行 ---------------------------------------------------------------
    def run(self, plan: Plan) -> TaskResult:
        with self._lock:
            self.plan = plan
            self.statuses = [StepStatus() for _ in plan.steps]
            self.current = -1
        self.events.emit("task_start", steps=len(plan.steps), t=round(self.robot.now(), 3))
        for i, step in enumerate(plan.steps):
            status = self.statuses[i]
            with self._lock:
                self.current = i
            ok = self._run_step(i, step, status)
            status.finished_at = self.robot.now()
            if not ok:
                status.phase = PHASE_FAILED
                self.robot.send_cmd(0.0, 0.0)
                self.events.emit("task_failed", step=i, message=status.message, t=round(self.robot.now(), 3))
                return TaskResult(False, status.message, failed_step=i, steps=self.statuses)
            status.phase = PHASE_DONE
        self.robot.send_cmd(0.0, 0.0)
        self.events.emit("task_done", t=round(self.robot.now(), 3))
        return TaskResult(True, "すべての手順が完了しました", steps=self.statuses)

    def _run_step(self, index: int, step: Step, status: StepStatus) -> bool:
        now = self.robot.now()
        status.started_at = now
        deadline = now + step.timeout_s
        self.events.emit("step_start", step=index, instruction=step.instruction,
                         when=step.when.to_dict(), do=step.do.to_dict(), t=round(now, 3))
        # 「一度成り立たないのを見る」必要があるのは after_false_then_true だけ
        status.seen_false = step.when.type != WHEN_AFTER_FALSE_THEN_TRUE
        while True:
            if step.when.type != WHEN_NOW:
                status.phase = PHASE_WAITING
                if not self._wait_condition(index, step, status, deadline):
                    return False
                if self.cfg.pre_departure_check and not self._path_clear(index, status, deadline):
                    if self.cancel.is_set() or self.robot.now() > deadline:
                        status.message = status.message or "進路の確認が通らないまま時間切れになりました"
                        return False
                    self._reset_streak(status)
                    continue
            status.phase = PHASE_ACTING
            self.events.emit("skill_start", step=index, do=step.do.to_dict(), t=round(self.robot.now(), 3))
            result: SkillResult = self.skills.run(step.do, deadline, confirmed_at=status.confirmed_at)
            self.events.emit("skill_end", step=index, ok=result.ok, message=result.message,
                             t=round(self.robot.now(), 3))
            status.message = result.message
            if result.ok:
                return True
            if (result.retry_condition and step.when.type != WHEN_NOW
                    and not self.cancel.is_set() and self.robot.now() < deadline):
                # 発進できなかった: 条件の確認からやり直す（扉が閉まり始めているかもしれない）
                self._reset_streak(status)
                continue
            return False

    @staticmethod
    def _reset_streak(status: StepStatus) -> None:
        status.streak = 0
        status.streak_answer = None
        status.confirmed_at = None

    def _wait_condition(self, index: int, step: Step, status: StepStatus, deadline: float) -> bool:
        """条件が確定するまで判定を繰り返す. 確定したら True、時間切れや停止なら False."""
        cfg = self.cfg
        errors = 0
        while True:
            if self.cancel.is_set():
                status.message = "停止が要求されました"
                return False
            t0 = self.robot.now()
            if t0 > deadline:
                status.message = "条件が成り立たないまま時間切れになりました"
                return False
            self.robot.send_cmd(0.0, 0.0)
            res = self.perception.check(step.when.condition)
            status.checks += 1
            answer = res.answer
            if answer is True and cfg.yes_threshold > 0.0 and res.p_yes is not None:
                answer = res.p_yes >= cfg.yes_threshold
            if answer is None:
                errors += 1
                if errors >= cfg.max_check_errors:
                    status.message = f"判定に続けて失敗しました: {res.error}"
                    return False
            else:
                errors = 0
                if answer == status.streak_answer:
                    status.streak += 1
                else:
                    status.streak_answer = answer
                    status.streak = 1
                if status.streak >= cfg.confirm_count:
                    if not status.seen_false:
                        if answer is False:
                            status.seen_false = True
                            self.events.emit("condition_false_confirmed", step=index,
                                             condition=step.when.condition, t=round(self.robot.now(), 3))
                    elif answer is True:
                        status.confirmed_at = self.robot.now()
                        self.events.emit("condition_confirmed", step=index,
                                         condition=step.when.condition, t=round(status.confirmed_at, 3))
                        return True
            # 推論が速いときだけ、周期を守るために待つ
            remaining = cfg.check_period_s - (self.robot.now() - t0)
            if remaining > 0.0:
                self.robot.sleep(remaining)

    def _path_clear(self, index: int, status: StepStatus, deadline: float) -> bool:
        for _ in range(max(self.cfg.pre_departure_retries, 1)):
            if self.cancel.is_set() or self.robot.now() > deadline:
                return False
            res = self.perception.check(PATH_CLEAR_STATEMENT, purpose="path_clear")
            if res.answer is True:
                return True
            self.robot.sleep(self.cfg.check_period_s)
        self.events.emit("path_not_clear", step=index, t=round(self.robot.now(), 3))
        status.message = "進路に人か物があると判定されました"
        return False
