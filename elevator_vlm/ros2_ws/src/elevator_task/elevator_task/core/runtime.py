"""実行系のまとめ役. 計画の作成、承認後の実行、停止、画面表示用の状態を受け持つ.

検討資料の task_planner、task_executor、vlm_perception、skill_server に当たる部品を
1つのプロセスの中で組み合わせる. 部品の間は Python の呼び出しでつなぐ.
"""
from __future__ import annotations

import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from .config import TaskConfig
from .events import EventLog
from .executor import TaskExecutor
from .geometry import ground_to_pixel, odom_to_robot
from .perception import Perception
from .plan import Plan, PlanError, parse_plan
from .planner import make_plan, split_instructions
from .robot_io import Marker, RobotIO, encode_jpeg
from .skills import Skills
from .vlm_client import VlmClient, VlmError

Trigger = Callable[[], Tuple[bool, str]]

TASK_IDLE = "idle"
TASK_RUNNING = "running"
TASK_DONE = "done"
TASK_FAILED = "failed"
TASK_STOPPED = "stopped"

PLAN_NONE = "none"
PLAN_PLANNING = "planning"
PLAN_READY = "ready"
PLAN_ERROR = "error"


class TaskRuntime:
    def __init__(self, cfg: TaskConfig, robot: RobotIO,
                 stop_hook: Optional[Trigger] = None, reset_hook: Optional[Trigger] = None,
                 triggers: Optional[Dict[str, Trigger]] = None,
                 extra_jpeg: Optional[Callable[[str], Optional[bytes]]] = None,
                 status_fn: Optional[Callable[[], Dict[str, Any]]] = None):
        self.cfg = cfg
        self.robot = robot
        self._extra_jpeg = extra_jpeg
        self._status_fn = status_fn
        self._stop_hook = stop_hook
        self._reset_hook = reset_hook
        self._triggers = triggers or {}
        run_dir = os.path.join(cfg.log_dir, time.strftime("%Y%m%d_%H%M%S")) if cfg.log_dir else None
        self.events = EventLog(run_dir)
        # log: true ならその状態の全体を、項目名の並びならその項目だけを、判定と指差しの記録に添える
        logged = {str(t.get("name", "")): t.get("log") for t in cfg.web.status_topics if t.get("log")}
        if status_fn is not None and logged:
            def context() -> Dict[str, Any]:
                out: Dict[str, Any] = {}
                for name, data in status_fn().items():
                    keys = logged.get(name)
                    if not keys or not isinstance(data, dict):
                        continue
                    out[name] = dict(data) if keys is True else {k: data[k] for k in keys if k in data}
                return out
            self.events.context_fn = context
        self.vlm = VlmClient(cfg.vlm)
        self.cancel = threading.Event()
        self.perception = Perception(robot, self.vlm, cfg.perception, self.events)
        self.skills = Skills(robot, self.perception, cfg.skills, self.events, self.cancel)
        self.executor = TaskExecutor(robot, self.perception, self.skills, cfg.executor, self.events, self.cancel)

        self._lock = threading.Lock()
        self.plan: Optional[Plan] = None
        self.plan_state = PLAN_NONE
        self.plan_error = ""
        self.plan_text = ""
        self.plan_latency_s = 0.0
        self.task_state = TASK_IDLE
        self.task_message = ""
        self.stopped = False
        self._task_thread: Optional[threading.Thread] = None
        self._plan_thread: Optional[threading.Thread] = None

        self.vlm_ok = False
        self.vlm_info = "未確認"
        self._closing = threading.Event()
        self._ping_thread = threading.Thread(target=self._ping_loop, name="vlm_ping", daemon=True)
        self._ping_thread.start()
        self.events.emit("runtime_start", vlm_base_url=cfg.vlm.base_url, vlm_model=cfg.vlm.model,
                         run_dir=self.events.run_dir or "")

    # -- VLM サーバの生存確認 -------------------------------------------------
    def _ping_loop(self) -> None:
        while not self._closing.is_set():
            # 実行中は推論の邪魔をしない
            if self.task_state != TASK_RUNNING:
                self.vlm_ok, self.vlm_info = self.vlm.ping()
            self._closing.wait(5.0)

    # -- 計画 ---------------------------------------------------------------
    def request_plan(self, text: str) -> Tuple[bool, str]:
        """指示文から計画を作る（別スレッドで VLM を呼ぶ）."""
        instructions = split_instructions(text)
        if not instructions:
            return False, "指示文が空です"
        with self._lock:
            if self.task_state == TASK_RUNNING:
                return False, "実行中は計画を作り直せません"
            if self.plan_state == PLAN_PLANNING:
                return False, "計画を作成中です"
            self.plan_state = PLAN_PLANNING
            self.plan_error = ""
            self.plan = None
        self._plan_thread = threading.Thread(target=self._plan_worker, args=(instructions,),
                                             name="planner", daemon=True)
        self._plan_thread.start()
        return True, "計画を作成しています"

    def _plan_worker(self, instructions: List[str]) -> None:
        self.events.emit("plan_request", instructions=instructions)
        jpeg = None
        frame = self.robot.get_frame()
        if frame is not None:
            jpeg = encode_jpeg(frame, self.cfg.perception.jpeg_quality)
            self.events.save_image("plan", jpeg)
        try:
            result = make_plan(self.vlm, instructions, jpeg, self.cfg.limits,
                               thinking=self.cfg.planner_thinking)
        except VlmError as e:
            with self._lock:
                self.plan_state = PLAN_ERROR
                self.plan_error = str(e)
            self.events.emit("plan_error", error=str(e))
            return
        with self._lock:
            self.plan_text = result.text
            self.plan_latency_s = result.latency_s
            if result.plan is None:
                self.plan_state = PLAN_ERROR
                self.plan_error = result.error
            else:
                self.plan = result.plan
                self.plan_state = PLAN_READY
        if result.plan is None:
            self.events.emit("plan_error", error=result.error, text=result.text[:500])
        else:
            self.events.emit("plan_ready", plan=result.plan.to_dict(), latency_s=round(result.latency_s, 3),
                             attempts=result.attempts)

    def set_plan(self, data: Any) -> Tuple[bool, str]:
        """計画の JSON を直接読み込む（計画の生成を使わない場合）."""
        try:
            plan = parse_plan(data, self.cfg.limits)
        except PlanError as e:
            return False, str(e)
        with self._lock:
            if self.task_state == TASK_RUNNING:
                return False, "実行中は計画を差し替えられません"
            self.plan = plan
            self.plan_state = PLAN_READY
            self.plan_error = ""
            self.plan_text = ""
        self.events.emit("plan_loaded", plan=plan.to_dict())
        return True, "計画を読み込みました"

    def presets(self) -> List[str]:
        """用意済みの計画（plan_dir の JSON ファイル）の名前."""
        try:
            return sorted(f for f in os.listdir(self.cfg.plan_dir) if f.endswith(".json")) if self.cfg.plan_dir else []
        except OSError:
            return []

    def load_preset(self, name: str) -> Tuple[bool, str]:
        if name not in self.presets():
            return False, f"用意済みの計画にありません: {name}"
        try:
            with open(os.path.join(self.cfg.plan_dir, name), "r", encoding="utf-8") as f:
                return self.set_plan(f.read())
        except OSError as e:
            return False, f"計画のファイルを読めません: {e}"

    # -- 実行 ---------------------------------------------------------------
    def execute(self) -> Tuple[bool, str]:
        """表示中の計画を承認して実行する."""
        with self._lock:
            if self.task_state == TASK_RUNNING:
                return False, "すでに実行中です"
            if self.plan is None or self.plan_state != PLAN_READY:
                return False, "承認できる計画がありません"
            if self.stopped:
                return False, "停止中です. 先に「停止を解除」を押してください"
            plan = self.plan
            self.cancel.clear()
            self.task_state = TASK_RUNNING
            self.task_message = ""
        self._task_thread = threading.Thread(target=self._task_worker, args=(plan,), name="executor", daemon=True)
        self._task_thread.start()
        return True, "実行を始めました"

    def _task_worker(self, plan: Plan) -> None:
        try:
            result = self.executor.run(plan)
            ok, message = result.ok, result.message
        except Exception as e:  # noqa: BLE001  想定外の例外でも必ず止めて状態を残す
            ok, message = False, f"実行中に例外が起きました: {e!r}"
            self.events.emit("task_exception", error=repr(e))
        finally:
            self.robot.send_cmd(0.0, 0.0)
        if self._status_fn is not None:
            # 終わった時点の状態（シミュレーションでは真値による判定を含む）を記録に残す.
            # 状態に summary（一言のまとめ）があれば、実行の結果の表示にも添える
            status = self._status_fn()
            self.events.emit("status", task_ok=ok, status=status)
            for topic in self.cfg.web.status_topics:
                data = status.get(str(topic.get("name", "")))
                if isinstance(data, dict) and data.get("summary"):
                    message += f"（{topic.get('label', topic.get('name', ''))}: {data['summary']}）"
        with self._lock:
            if self.cancel.is_set():
                self.task_state = TASK_STOPPED
            else:
                self.task_state = TASK_DONE if ok else TASK_FAILED
            self.task_message = message

    def stop(self) -> Tuple[bool, str]:
        """停止. 実行を中断し、安全ゲートにも停止を要求する."""
        self.cancel.set()
        self.robot.send_cmd(0.0, 0.0)
        with self._lock:
            self.stopped = True
        self.events.emit("stop_requested")
        if self._stop_hook is not None:
            ok, msg = self._stop_hook()
            if not ok:
                return False, f"安全ゲートに停止を伝えられません: {msg}"
        return True, "停止しました"

    def reset(self) -> Tuple[bool, str]:
        """停止を解除する（実行は再開しない）."""
        with self._lock:
            if self.task_state == TASK_RUNNING:
                return False, "実行の中断が終わるまで待ってください"
        if self._reset_hook is not None:
            ok, msg = self._reset_hook()
            if not ok:
                return False, f"安全ゲートの停止を解除できません: {msg}"
        with self._lock:
            self.stopped = False
            if self.task_state != TASK_RUNNING:
                self.task_state = TASK_IDLE
                self.task_message = ""
        self.cancel.clear()
        self.events.emit("stop_released")
        return True, "停止を解除しました"

    def trigger(self, name: str) -> Tuple[bool, str]:
        fn = self._triggers.get(name)
        if fn is None:
            return False, f"知らないボタンです: {name}"
        ok, msg = fn()
        self.events.emit("button", name=name, ok=ok, message=msg)
        return ok, msg

    def wait_task(self, timeout: Optional[float] = None) -> None:
        if self._task_thread is not None:
            self._task_thread.join(timeout)

    def wait_plan(self, timeout: Optional[float] = None) -> None:
        if self._plan_thread is not None:
            self._plan_thread.join(timeout)

    def close(self) -> None:
        self._closing.set()
        self.cancel.set()
        self.robot.send_cmd(0.0, 0.0)
        self.events.close()

    # -- 画面表示 -------------------------------------------------------------
    def state(self, since: int = 0) -> Dict[str, Any]:
        pose = self.robot.get_pose()
        with self._lock:
            out: Dict[str, Any] = {
                "time": round(self.robot.now(), 3),
                "task_state": self.task_state,
                "task_message": self.task_message,
                "stopped": self.stopped,
                "plan_state": self.plan_state,
                "plan_error": self.plan_error,
                "plan": self.plan.to_dict() if self.plan else None,
                "plan_latency_s": round(self.plan_latency_s, 3),
            }
        out["executor"] = self.executor.snapshot()
        out["pose"] = None if pose is None else [round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 4)]
        out["robot_status"] = self.robot.status()
        out["vlm"] = {"base_url": self.cfg.vlm.base_url, "model": self.cfg.vlm.model,
                      "ok": self.vlm_ok, "info": self.vlm_info}
        chk = self.perception.last_check
        out["last_check"] = None if chk is None else {
            "answer": None if chk.answer is None else ("yes" if chk.answer else "no"),
            "latency_s": round(chk.latency_s, 3), "stamp": round(chk.stamp, 3), "error": chk.error}
        out["buttons"] = [{"name": b.get("name", ""), "label": b.get("label", b.get("name", ""))}
                          for b in self.cfg.web.buttons]
        out["extra_cameras"] = [{"name": c.get("name", ""), "label": c.get("label", c.get("name", ""))}
                                for c in self.cfg.web.extra_cameras]
        out["instructions"] = self.cfg.instructions
        out["presets"] = self.presets()
        status = self._status_fn() if self._status_fn is not None else {}
        out["status"] = [{"name": str(t.get("name", "")), "label": str(t.get("label", t.get("name", ""))),
                          "data": status.get(str(t.get("name", "")))} for t in self.cfg.web.status_topics]
        out["run_dir"] = self.events.run_dir or ""
        out["events"] = self.events.since(since)
        return out

    def extra_jpeg(self, name: str) -> Optional[bytes]:
        """追加のカメラ（外から見る固定カメラなど）の最新の画像."""
        if self._extra_jpeg is None:
            return None
        return self._extra_jpeg(name)

    def overlay_jpeg(self, quality: int = 80) -> Optional[bytes]:
        """最新の画像に、指差した点・目標点・状態を重ねた JPEG（画面の映像用）."""
        frame = self.robot.get_frame()
        if frame is None:
            return None
        markers: List[Marker] = []
        now = self.robot.now()
        pt = self.perception.last_point
        if pt is not None and pt.ok and now - pt.stamp < 3.0:
            markers.append(Marker(pt.u, pt.v, color=(255, 60, 60), label="point"))
        target = self.skills.current_target
        cam = self.robot.get_camera()
        pose = self.robot.get_pose()
        if target is not None and cam is not None and pose is not None:
            rx, ry = odom_to_robot(pose, target[0], target[1])
            uv = ground_to_pixel(cam, rx, ry)
            if uv is not None and 0 <= uv[0] < frame.width and 0 <= uv[1] < frame.height:
                markers.append(Marker(uv[0], uv[1], color=(40, 220, 80), radius=7, label="goal"))
        lines = [f"task: {self.task_state}"]
        snap = self.executor.snapshot()
        cur = snap["current"]
        if self.task_state == TASK_RUNNING and 0 <= cur < len(snap["steps"]):
            lines.append(f"step {cur + 1}/{len(snap['steps'])}: {snap['steps'][cur]['phase']}")
        chk = self.perception.last_check
        if chk is not None and now - chk.stamp < 5.0:
            ans = "?" if chk.answer is None else ("yes" if chk.answer else "no")
            lines.append(f"check: {ans} ({chk.latency_s:.2f}s)")
        return encode_jpeg(frame, quality, markers=markers, lines=lines)
