"""通し試験. Web UI の API で「計画 → 承認 → 実行」を行い、エレベーターを呼んで、審判の結果を確かめる.

  ros2 run elevator_sim e2e_runner --ros-args -p out_dir:=/runs/e2e

人が画面で行う操作と同じ経路（HTTP の API）を通るので、実行系と Web UI のつながりも確かめられる.
終了コードは、タスクが完了して審判が成功と判定すれば 0、それ以外は 1.

mode:=path（道のベンチ）では、途中から・ここまで の流れを通す:
  映像を start_frame に移す → 計画を読み込む → 「承認して開始の合図を待つ」（API）→
  ROS のトピック /vlm_drive/command に start → 終わるまで見守る（finish_after_m を超えたら finish）.
  成功は、実行が完了し、結果が expect_result（done か finished）で、映像が expect_min_frame 以上まで進み、
  /vlm_drive/active が true → false と変わったこと.
"""
from __future__ import annotations

import base64
import io
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from elevator_task.core.http_util import urlopen

SNAPSHOT_EVENTS = ("condition_confirmed", "condition_false_confirmed", "skill_end", "target_set")


class E2ERunner(Node):
    def __init__(self) -> None:
        super().__init__("e2e_runner")
        self.ui_url = str(self.declare_parameter("ui_url", "http://127.0.0.1:8080").value).rstrip("/")
        self.out_dir = str(self.declare_parameter("out_dir", "/runs/e2e").value)
        self.ready_timeout_s = float(self.declare_parameter("ready_timeout_s", 240.0).value)
        self.task_timeout_s = float(self.declare_parameter("task_timeout_s", 900.0).value)
        self.plan_file = str(self.declare_parameter("plan_file", "").value)
        self.call_button = str(self.declare_parameter("call_button", "call").value)
        self.print_images = bool(self.declare_parameter("print_images", False).value)
        self.mode = str(self.declare_parameter("mode", "elevator").value).strip().lower()
        # 道のベンチ（mode:=path）
        self.start_frame = int(self.declare_parameter("start_frame", 0).value)
        self.arm_from = int(self.declare_parameter("arm_from", 0).value)
        self.arm_to = int(self.declare_parameter("arm_to", 0).value)
        self.finish_after_m = float(self.declare_parameter("finish_after_m", 0.0).value)
        self.expect_result = str(self.declare_parameter("expect_result", "done").value)
        self.expect_min_frame = int(self.declare_parameter("expect_min_frame", 0).value)
        self.referee: Dict[str, Any] = {}
        self.elevator: Dict[str, Any] = {}
        self.bench_path: Dict[str, Any] = {}
        self.drive_states: List[Dict[str, Any]] = []
        self.active_seen: List[bool] = []
        self.exit_code = 1
        self.finished = threading.Event()
        self._shots: List[str] = []
        self._extra_shots: List[str] = []
        self._extra_cameras: List[str] = []
        self.create_subscription(String, "/sim/referee", self._on_referee, 10)
        self.create_subscription(String, "/elevator/state", self._on_elevator, 10)
        if self.mode == "path":
            self.create_subscription(String, "/bench/path", self._on_bench_path, 10)
            self.create_subscription(String, "/vlm_drive/state", self._on_drive_state, 10)
            self.create_subscription(Bool, "/vlm_drive/active", self._on_active, 10)
            self._goto_pub = self.create_publisher(Int32, "/bench/path/goto", 10)
            self._command_pub = self.create_publisher(String, "/vlm_drive/command", 10)
        os.makedirs(self.out_dir, exist_ok=True)
        self._worker = threading.Thread(target=self._run_guarded, name="e2e", daemon=True)
        self._worker.start()

    def _on_referee(self, msg: String) -> None:
        try:
            self.referee = json.loads(msg.data)
        except ValueError:
            pass

    def _on_elevator(self, msg: String) -> None:
        try:
            self.elevator = json.loads(msg.data)
        except ValueError:
            pass

    def _on_bench_path(self, msg: String) -> None:
        try:
            self.bench_path = json.loads(msg.data)
        except ValueError:
            pass

    def _on_drive_state(self, msg: String) -> None:
        try:
            self.drive_states.append(json.loads(msg.data))
        except ValueError:
            pass

    def _on_active(self, msg: Bool) -> None:
        if not self.active_seen or self.active_seen[-1] != bool(msg.data):
            self.active_seen.append(bool(msg.data))

    # -- HTTP ---------------------------------------------------------------
    def _get(self, path: str, timeout: float = 5.0) -> bytes:
        with urlopen(self.ui_url + path, timeout=timeout) as resp:
            return resp.read()

    def _state(self, since: int = 0) -> Dict[str, Any]:
        return json.loads(self._get(f"/api/state?since={since}").decode("utf-8"))

    def _post(self, path: str, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        req = urllib.request.Request(self.ui_url + path, data=json.dumps(body or {}).encode("utf-8"),
                                     method="POST", headers={"Content-Type": "application/json"})
        with urlopen(req, timeout=10.0) as resp:
            out = json.loads(resp.read().decode("utf-8"))
        self._log(f"POST {path} → {out}")
        return out

    def _log(self, text: str) -> None:
        print(f"[e2e] {text}", flush=True)

    def _snapshot(self, label: str) -> None:
        name = f"{len(self._shots):02d}_{label}.jpg"
        try:
            data = self._get("/snapshot.jpg")
        except (urllib.error.URLError, OSError) as e:
            self._log(f"画像を取得できません（{label}）: {e}")
            return
        path = os.path.join(self.out_dir, name)
        with open(path, "wb") as f:
            f.write(data)
        self._shots.append(path)
        # 外から見る固定カメラの画像も残す
        for cam in self._extra_cameras:
            try:
                data = self._get(f"/extra/{cam}.jpg")
            except (urllib.error.URLError, OSError):
                continue
            extra_path = os.path.join(self.out_dir, name.replace(".jpg", f"_view_{cam}.jpg"))
            with open(extra_path, "wb") as f:
                f.write(data)
            self._extra_shots.append(extra_path)

    # -- 手順 ---------------------------------------------------------------
    def _run_guarded(self) -> None:
        try:
            self.exit_code = self._run()
        except Exception as e:  # noqa: BLE001
            self._log(f"例外で中断しました: {e!r}")
            self.exit_code = 1
        finally:
            self.finished.set()

    def _run(self) -> int:
        # 1. 実行系と VLM、カメラ、オドメトリの準備ができるまで待つ
        t0 = time.monotonic()
        state: Dict[str, Any] = {}
        while True:
            try:
                state = self._state()
                if not state["robot_status"] and state["vlm"]["ok"]:
                    break
                detail = f"入力={state['robot_status']} VLM={state['vlm']['ok']}（{state['vlm']['info']}）"
            except (urllib.error.URLError, OSError, ValueError, KeyError) as e:
                detail = f"Web UI に接続できません: {e}"
            if time.monotonic() - t0 > self.ready_timeout_s:
                self._log(f"準備が整いません: {detail}")
                return self._finish("not_ready", state)
            time.sleep(1.0)
        self._log(f"準備完了（{time.monotonic() - t0:.0f} 秒）. VLM: {state['vlm']['base_url']} / {state['vlm']['info']}")
        self._extra_cameras = [c["name"] for c in state.get("extra_cameras", [])]
        if self.mode == "path":
            return self._run_path(state)
        self._snapshot("start")

        # 2. 計画
        if self.plan_file:
            with open(self.plan_file, "r", encoding="utf-8") as f:
                res = self._post("/api/set_plan", {"plan": json.load(f)})
        else:
            res = self._post("/api/plan", {"instructions": "\n".join(state["instructions"])})
        if not res.get("ok"):
            return self._finish("plan_rejected", state)
        t1 = time.monotonic()
        while True:
            state = self._state()
            if state["plan_state"] in ("ready", "error"):
                break
            if time.monotonic() - t1 > 180.0:
                return self._finish("plan_timeout", state)
            time.sleep(0.5)
        if state["plan_state"] != "ready":
            self._log(f"計画を作れません: {state['plan_error']}")
            return self._finish("plan_error", state)
        self._log("計画:\n" + json.dumps(state["plan"], ensure_ascii=False, indent=2))

        # 3. 承認して実行し、エレベーターを呼ぶ
        if not self._post("/api/execute").get("ok"):
            return self._finish("execute_rejected", state)
        if not self._post("/api/trigger", {"name": self.call_button}).get("ok"):
            return self._finish("call_failed", state)

        # 4. 終わるまで見守る（記録は、この実行の分だけ出す）
        since = max([e["id"] for e in self._state()["events"]] or [0])
        t2 = time.monotonic()
        last_report = 0.0
        while True:
            state = self._state(since)
            for event in state["events"]:
                since = max(since, event["id"])
                kind = event["kind"]
                if kind in ("check", "point", "check_error", "point_error"):
                    continue
                self._log("event " + json.dumps({k: v for k, v in event.items() if k not in ("id", "wall")},
                                                ensure_ascii=False))
                if kind in SNAPSHOT_EVENTS:
                    self._snapshot(f"{kind}_step{event.get('step', 'x')}")
            if state["task_state"] in ("done", "failed", "stopped"):
                break
            wall = time.monotonic() - t2
            if wall - last_report > 20.0:
                last_report = wall
                self._log(f"実行中: 実時間 {wall:.0f} 秒、シミュレーション時刻 {state['time']:.1f} 秒、"
                          f"エレベーター {self.elevator.get('phase')} 階 {self.elevator.get('floor')}")
            if wall > self.task_timeout_s:
                self._log("時間切れです. 停止します")
                self._post("/api/stop")
                return self._finish("timeout", state)
            time.sleep(0.5)
        self._snapshot("end")

        # 5. 審判の結果が出そろうのを少し待つ
        t3 = time.monotonic()
        while state["task_state"] == "done" and not self.referee.get("success") and time.monotonic() - t3 < 8.0:
            time.sleep(0.5)
        return self._finish(state["task_state"], state)

    # -- 道のベンチ: 途中から・ここまで ------------------------------------------------
    def _load_plan(self, state: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if self.plan_file:
            with open(self.plan_file, "r", encoding="utf-8") as f:
                res = self._post("/api/set_plan", {"plan": json.load(f)})
        else:
            res = self._post("/api/plan", {"instructions": "\n".join(state["instructions"])})
        if not res.get("ok"):
            return None
        t1 = time.monotonic()
        while True:
            state = self._state()
            if state["plan_state"] in ("ready", "error"):
                break
            if time.monotonic() - t1 > 180.0:
                return None
            time.sleep(0.5)
        if state["plan_state"] != "ready":
            self._log(f"計画を作れません: {state['plan_error']}")
            return None
        self._log("計画:\n" + json.dumps(state["plan"], ensure_ascii=False, indent=2))
        return state

    def _run_path(self, state: Dict[str, Any]) -> int:
        # 1. 映像を、VLM に任せる区間の始まりに移す
        if self.start_frame > 0:
            t0 = time.monotonic()
            while time.monotonic() - t0 < 15.0:
                self._goto_pub.publish(Int32(data=self.start_frame))
                time.sleep(0.5)
                if abs(int(self.bench_path.get("frame_number") or -999) - self.start_frame) <= 1:
                    break
            self._log(f"映像: {self.bench_path.get('frame')}")
        self._snapshot("start")
        # 2. 計画を読み込み、承認して開始の合図を待つ
        loaded = self._load_plan(state)
        if loaded is None:
            return self._finish("plan_error", state, path=True)
        body = {k: v for k, v in (("from", self.arm_from), ("to", self.arm_to)) if v > 0}
        if not self._post("/api/arm", body).get("ok"):
            return self._finish("arm_rejected", loaded, path=True)
        # 3. 外（ROS1システム）からの開始の合図: ROS のトピックに start.
        #    前に画面で走らせた結果（完了・失敗）が残っていても、走らせた回数（run）が増えるまで合図を出す
        t1 = time.monotonic()
        state = self._state()
        runs_before = int(state["drive"].get("run") or 0)
        while int(state["drive"].get("run") or 0) <= runs_before:
            if time.monotonic() - t1 > 15.0:
                self._log("開始の合図を受け付けません: " + json.dumps(state.get("last_command"), ensure_ascii=False))
                return self._finish("start_not_accepted", state, path=True)
            self._log("合図を出します: /vlm_drive/command ← start")
            self._command_pub.publish(String(data="start"))
            time.sleep(1.0)
            state = self._state()
        # 4. 終わるまで見守る（記録は、この実行の分だけ出す）
        since = max([e["id"] for e in self._state()["events"]] or [0])
        finished_sent = False
        t2 = time.monotonic()
        last_report = 0.0
        while True:
            state = self._state(since)
            for event in state["events"]:
                since = max(since, event["id"])
                if event["kind"] in ("path_start", "path_end", "path_stop_condition", "path_stalled", "skill_end",
                                     "task_finished", "task_done", "task_failed", "command", "armed"):
                    self._log("event " + json.dumps({k: v for k, v in event.items() if k not in ("id", "wall")},
                                                    ensure_ascii=False))
                if event["kind"] in ("path_stop_condition", "skill_end"):
                    self._snapshot(event["kind"])
            if state["task_state"] in ("done", "failed", "stopped"):
                break
            progress = state.get("path") or {}
            if self.finish_after_m > 0.0 and not finished_sent and (progress.get("traveled_m") or 0.0) >= self.finish_after_m:
                self._log(f"{self.finish_after_m} m を超えたので、合図を出します: /vlm_drive/command ← finish")
                self._command_pub.publish(String(data="finish"))
                finished_sent = True
            wall = time.monotonic() - t2
            if wall - last_report > 10.0:
                last_report = wall
                self._log(f"走行中: {progress.get('traveled_m')} m、答え {progress.get('answers')} 回、"
                          f"{progress.get('state')}、映像 {self.bench_path.get('frame')}")
            if wall > self.task_timeout_s:
                self._log("時間切れです. 停止します")
                self._post("/api/stop")
                return self._finish("timeout", state, path=True)
            time.sleep(0.5)
        self._snapshot("end")
        time.sleep(1.5)                   # 最後の状態のトピックを受け取る
        return self._finish(state["task_state"], self._state(), path=True)

    def _finish(self, outcome: str, state: Dict[str, Any], path: bool = False) -> int:
        if path:
            return self._finish_path(outcome, state)
        return self._finish_elevator(outcome, state)

    def _finish_path(self, outcome: str, state: Dict[str, Any]) -> int:
        drive = state.get("drive") or {}
        frame = int(self.bench_path.get("frame_number") or 0)
        checks = {
            "完了した": outcome == "done",
            f"結果が {self.expect_result}": drive.get("result") == self.expect_result,
            f"映像が {self.expect_min_frame} 枚目まで進んだ": frame >= self.expect_min_frame,
            "外への知らせ（/vlm_drive/active）が true → false": self.active_seen[-2:] == [True, False],
            "外への知らせ（/vlm_drive/state）を受け取った": bool(self.drive_states),
        }
        passed = all(checks.values())
        summary = {
            "result": "PASS" if passed else "FAIL",
            "outcome": outcome,
            "checks": checks,
            "task_state": state.get("task_state"),
            "task_message": state.get("task_message"),
            "steps": (state.get("executor") or {}).get("steps"),
            "path": state.get("path"),
            "drive": drive,
            "bench_path": self.bench_path,
            "active_seen": self.active_seen,
            "drive_state_last": self.drive_states[-1] if self.drive_states else None,
            "run_dir": state.get("run_dir"),
        }
        with open(os.path.join(self.out_dir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        self._log("まとめ:\n" + json.dumps(summary, ensure_ascii=False, indent=2))
        if self.print_images:
            self._print_images()
        self._log(f"E2E RESULT: {summary['result']}")
        return 0 if passed else 1

    def _finish_elevator(self, outcome: str, state: Dict[str, Any]) -> int:
        passed = outcome == "done" and bool(self.referee.get("success"))
        summary = {
            "result": "PASS" if passed else "FAIL",
            "outcome": outcome,
            "task_state": state.get("task_state"),
            "task_message": state.get("task_message"),
            "steps": (state.get("executor") or {}).get("steps"),
            "referee": self.referee,
            "elevator": self.elevator,
            "run_dir": state.get("run_dir"),
        }
        with open(os.path.join(self.out_dir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        self._log("まとめ:\n" + json.dumps(summary, ensure_ascii=False, indent=2))
        if self.print_images:
            self._print_images()
        self._log(f"E2E RESULT: {summary['result']}")
        return 0 if passed else 1

    def _print_images(self) -> None:
        """画像を小さくして base64 でログに出す（CI のログから取り出して目で確かめるため）."""
        try:
            from PIL import Image
        except ImportError:
            return
        for path in self._shots + self._extra_shots:
            try:
                img = Image.open(path).convert("RGB")
                img.thumbnail((360, 270))
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=65)
            except OSError:
                continue
            print(f"IMG_BEGIN {os.path.basename(path)}", flush=True)
            print(base64.b64encode(buf.getvalue()).decode("ascii"), flush=True)
            print("IMG_END", flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[E2ERunner] = None
    code = 1
    try:
        node = E2ERunner()
        while rclpy.ok() and not node.finished.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
        code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
