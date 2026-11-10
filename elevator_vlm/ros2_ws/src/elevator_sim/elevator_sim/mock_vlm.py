"""モック VLM. 画像は見ずに、シミュレータの真値から答える OpenAI 互換のサーバ.

本物の VLM を使う前に、世界・制御・実行系のつながりを確かめるためのもの.
「VLM が完璧だったらどうなるか」の基準にもなる. 応答の遅れ、指差しのずれ、誤答を足すこともできる.
"""
from __future__ import annotations

import json
import math
import random
import re
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional

from elevator_task.core.geometry import CameraModel, ground_to_pixel

from .world_params import Truth, WorldParams, door_fully_open

DOOR_CONDITION = "Elevator door is fully open."
TARGET_CAR = "center of the elevator car floor"
TARGET_HALL = "hallway floor 1 m past the door"


@dataclass
class MockConfig:
    latency_s: float = 0.3          # 応答の遅れ [s]（実時間）
    point_noise_px: float = 0.0     # 指差しに足すずれの標準偏差 [画素]
    check_flip_prob: float = 0.0    # 判定をわざと間違える確率
    seed: int = 0
    model_name: str = "mock-vlm"


def _user_text(request: Dict[str, Any]) -> str:
    texts: List[str] = []
    for msg in request.get("messages", []):
        if msg.get("role") != "user":
            continue
        content = msg.get("content")
        if isinstance(content, str):
            texts.append(content)
        elif isinstance(content, list):
            texts.extend(str(p.get("text", "")) for p in content if isinstance(p, dict) and p.get("type") == "text")
    return "\n".join(texts)


def _kind(request: Dict[str, Any]) -> str:
    """問いの種類（check / point / plan）を、構造化出力の名前かシステムプロンプトから決める."""
    fmt = request.get("response_format") or {}
    name = str((fmt.get("json_schema") or {}).get("name", ""))
    if name in ("condition_check", "point_target", "task_plan"):
        return {"condition_check": "check", "point_target": "point", "task_plan": "plan"}[name]
    system = " ".join(str(m.get("content", "")) for m in request.get("messages", []) if m.get("role") == "system")
    if "task planner" in system:
        return "plan"
    if "point_2d" in system:
        return "point"
    return "check"


def plan_from_instructions(lines: List[str]) -> Dict[str, Any]:
    """指示文から計画を作る簡単な規則（モック専用. 本物では VLM が作る）."""
    steps = []
    for line in lines:
        low = line.lower()
        again = any(k in low for k in ("再び", "もう一度", "again", "once more"))
        waits_door = any(k in low for k in ("開いたら", "door open", "door is open"))
        if again:
            when: Dict[str, Any] = {"type": "after_false_then_true", "condition": DOOR_CONDITION}
            timeout = 300
        elif waits_door:
            when = {"type": "when_true", "condition": DOOR_CONDITION}
            timeout = 120
        else:
            when = {"type": "now"}
            timeout = 30
        angle = re.search(r"(-?\d+(?:\.\d+)?)\s*(?:度|°|deg)", low)
        dist = re.search(r"(-?\d+(?:\.\d+)?)\s*(?:m|メートル)(?![a-z])", low)
        if angle:
            do: Dict[str, Any] = {"skill": "rotate", "angle_deg": float(angle.group(1))}
        elif dist:
            do = {"skill": "move_straight", "distance_m": float(dist.group(1))}
        elif any(k in low for k in ("出て", "降り", "exit", "go out", "get off", "leave")):
            do = {"skill": "go_to_visual_target", "target": TARGET_HALL}
        elif any(k in low for k in ("入って", "乗って", "enter", "go in", "get on", "inside")):
            do = {"skill": "go_to_visual_target", "target": TARGET_CAR}
        else:
            do = {"skill": "wait", "seconds": 1.0}
        steps.append({"instruction": line, "when": when, "do": do, "timeout_s": timeout})
    return {"steps": steps}


class MockVlm:
    def __init__(self, params: WorldParams, cfg: MockConfig,
                 truth_fn: Callable[[], Optional[Truth]], camera_fn: Callable[[], Optional[CameraModel]]):
        self.p = params
        self.cfg = cfg
        self._truth_fn = truth_fn
        self._camera_fn = camera_fn
        self._rng = random.Random(cfg.seed)
        self._lock = threading.Lock()
        self.calls = {"check": 0, "point": 0, "plan": 0}

    # -- 3種類の答え ---------------------------------------------------------
    def _check(self, text: str) -> str:
        m = re.search(r'Statement:\s*"(.*?)"', text, re.S)
        statement = (m.group(1) if m else text).lower()
        truth = self._truth_fn()
        if "marker" in statement or "path" in statement:
            answer = True                      # 印の確認と進路の確認は、正常系では常に yes
        elif truth is None:
            answer = False
        elif "door" in statement:
            answer = door_fully_open(self.p, truth)
            if "closed" in statement or "not open" in statement:
                answer = not answer
        else:
            answer = False
        with self._lock:
            if self.cfg.check_flip_prob > 0.0 and self._rng.random() < self.cfg.check_flip_prob:
                answer = not answer
        return json.dumps({"answer": "yes" if answer else "no"})

    def _point(self, text: str) -> str:
        m = re.search(r"point on the floor:\s*(.*?)\.\s*Output", text, re.S)
        target = (m.group(1) if m else text).lower()
        truth = self._truth_fn()
        cam = self._camera_fn()
        if truth is None or cam is None:
            return json.dumps({"point_2d": [500, 50]})      # 地平線より上 → 検査で落ちる
        if any(k in target for k in ("hall", "corridor", "lobby", "outside", "past the door", "beyond")):
            floor = self.p.nearest_floor(truth.robot_z) or 1
            wx, wy, wz = -1.0, 0.0, self.p.floor_z(floor)
        elif any(k in target for k in ("elevator", "car", "cabin", "inside", "interior")):
            wx, wy, wz = self.p.car_center_x, 0.0, truth.lift_pos
        else:
            c, s = math.cos(truth.robot_yaw), math.sin(truth.robot_yaw)
            wx, wy, wz = truth.robot_x + c * 1.0, truth.robot_y + s * 1.0, truth.robot_z
        # ワールド座標 → ロボット座標
        dx, dy = wx - truth.robot_x, wy - truth.robot_y
        c, s = math.cos(truth.robot_yaw), math.sin(truth.robot_yaw)
        rx, ry, rz = c * dx + s * dy, -s * dx + c * dy, wz - truth.robot_z
        uv = ground_to_pixel(cam, rx, ry, rz)
        if uv is None:
            return json.dumps({"point_2d": [500, 50]})
        u, v = uv
        with self._lock:
            if self.cfg.point_noise_px > 0.0:
                u += self._rng.gauss(0.0, self.cfg.point_noise_px)
                v += self._rng.gauss(0.0, self.cfg.point_noise_px)
        x = max(0.0, min(1000.0, u / cam.width * 1000.0))
        y = max(0.0, min(1000.0, v / cam.height * 1000.0))
        return json.dumps({"point_2d": [round(x), round(y)]})

    def _plan(self, text: str) -> str:
        lines = [m.group(1).strip() for m in re.finditer(r"^\s*\d+\.\s*(.+)$", text, re.M)]
        return json.dumps(plan_from_instructions(lines), ensure_ascii=False)

    # -- OpenAI 互換の応答 ---------------------------------------------------
    def handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        kind = _kind(request)
        text = _user_text(request)
        if kind == "plan":
            content = self._plan(text)
        elif kind == "point":
            content = self._point(text)
        else:
            content = self._check(text)
        with self._lock:
            self.calls[kind] += 1
        if self.cfg.latency_s > 0.0:
            time.sleep(self.cfg.latency_s)
        return {
            "id": "mock", "object": "chat.completion", "created": int(time.time()),
            "model": self.cfg.model_name,
            "choices": [{"index": 0, "message": {"role": "assistant", "content": content},
                         "finish_reason": "stop"}],
            "usage": {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
        }


def make_server(mock: MockVlm, host: str = "127.0.0.1", port: int = 8001) -> ThreadingHTTPServer:
    """モック VLM を OpenAI 互換の HTTP サーバにする（/v1/chat/completions、/v1/models）."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _json(self, obj: Any, code: int = 200) -> None:
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path.rstrip("/").endswith("/models"):
                self._json({"object": "list", "data": [{"id": mock.cfg.model_name, "object": "model"}]})
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self) -> None:  # noqa: N802
            try:
                length = int(self.headers.get("Content-Length") or 0)
                request = json.loads(self.rfile.read(length).decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as e:
                self._json({"error": f"bad request: {e}"}, 400)
                return
            if self.path.rstrip("/").endswith("/chat/completions"):
                self._json(mock.handle(request))
            else:
                self._json({"error": "not found"}, 404)

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
