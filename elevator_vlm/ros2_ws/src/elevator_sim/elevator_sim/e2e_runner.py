"""通し試験. Web UI の API で「計画 → 承認 → 実行」を行い、エレベーターを呼んで、審判の結果を確かめる.

  ros2 run elevator_sim e2e_runner --ros-args -p out_dir:=/runs/e2e

人が画面で行う操作と同じ経路（HTTP の API）を通るので、実行系と Web UI のつながりも確かめられる.
終了コードは、タスクが完了して審判が成功と判定すれば 0、それ以外は 1.
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
from std_msgs.msg import String

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
        self.referee: Dict[str, Any] = {}
        self.elevator: Dict[str, Any] = {}
        self.exit_code = 1
        self.finished = threading.Event()
        self._shots: List[str] = []
        self._extra_shots: List[str] = []
        self._extra_cameras: List[str] = []
        self.create_subscription(String, "/sim/referee", self._on_referee, 10)
        self.create_subscription(String, "/elevator/state", self._on_elevator, 10)
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

        # 4. 終わるまで見守る
        since = 0
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

    def _finish(self, outcome: str, state: Dict[str, Any]) -> int:
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
