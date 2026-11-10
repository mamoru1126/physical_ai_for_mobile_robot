"""擬似世界での通し試験. 実行系（計画、判定、指差し、スキル、実行器）とエレベーターの制御、モック VLM、
審判をすべて本物のコードでつなぎ、3つの指示が正常系のとおりに進むことを確かめる.

VLM とのやり取りは本物の HTTP クライアントとモックの HTTP サーバを通す.
"""
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass

import pytest
from fake_world import FakeWorld, SimLatencyVlm, true_camera

from elevator_sim.elevator_logic import OPEN, OPENING, ElevatorConfig
from elevator_sim.mock_vlm import MockConfig, MockVlm, make_server
from elevator_task.core.config import DEFAULT_INSTRUCTIONS, WebConfig, load_config
from elevator_task.core.events import EventLog
from elevator_task.core.executor import ExecutorConfig, TaskExecutor
from elevator_task.core.geometry import CameraModel
from elevator_task.core.perception import Perception, PerceptionConfig
from elevator_task.core.plan import parse_plan
from elevator_task.core.planner import make_plan
from elevator_task.core.runtime import TaskRuntime
from elevator_task.core.skills import SkillConfig, Skills
from elevator_task.core.vlm_client import VlmConfig
from elevator_task.core.webui import WebUI

TASK_PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "elevator_task")


@dataclass
class Stack:
    world: FakeWorld
    mock: MockVlm
    vlm: SimLatencyVlm
    events: EventLog
    cancel: threading.Event
    executor: TaskExecutor
    url: str

    def kinds(self):
        return [e["kind"] for e in self.events.since(0, limit=100000)]


@pytest.fixture
def make_stack():
    servers = []

    def _make(world: FakeWorld, latency=0.3, noise=0.0, flip=0.0, seed=0, skill=None, executor=None) -> Stack:
        mock = MockVlm(world.wp, MockConfig(latency_s=0.0, point_noise_px=noise, check_flip_prob=flip, seed=seed),
                       world.truth, lambda: world.camera_true)
        server = make_server(mock, "127.0.0.1", 0)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
        url = f"http://127.0.0.1:{server.server_address[1]}/v1"
        vlm = SimLatencyVlm(VlmConfig(base_url=url, model="mock-vlm"), world, latency)
        events = EventLog(None, keep=100000)
        cancel = threading.Event()
        perception = Perception(world, vlm, PerceptionConfig(), events)
        skill_cfg = SkillConfig(**{"repoint_period_s": 0.0, **(skill or {})})
        skills = Skills(world, perception, skill_cfg, events, cancel)
        ex = TaskExecutor(world, perception, skills, ExecutorConfig(**(executor or {})), events, cancel)
        return Stack(world, mock, vlm, events, cancel, ex, url)

    yield _make
    for server in servers:
        server.shutdown()
        server.server_close()


def run_task(stack: Stack, plan=None):
    world = stack.world
    world.sleep(1.0)
    if plan is None:
        planned = make_plan(stack.vlm, DEFAULT_INSTRUCTIONS)
        assert planned.plan is not None, planned.error
        plan = planned.plan
    ok, message = world.logic.call(world.t)
    assert ok, message
    result = stack.executor.run(plan)
    world.sleep(3.0)
    return result


def test_nominal_three_instructions(make_stack):
    world = FakeWorld()
    stack = make_stack(world)
    result = run_task(stack)
    ref = world.referee.result()
    assert result.ok, (result.message, ref)
    assert ref["success"] and not ref["failed"], ref
    # 乗った → 扉の方を向いた → 行先階に着いた → 降りた、の順
    assert ref["entered_car_s"] < ref["turned_s"] < ref["arrived_dest_floor_s"] < ref["exited_dest_s"]
    assert not world.door_contact and not world.wall_contact
    assert world.floor == world.ecfg.dest_floor and world.x < -0.5
    assert ref["back_wall_margin_m"] > 0.25 and ref["door_side_margin_m"] > 0.15
    assert world.max_speed <= 0.3 + 1e-6
    assert world.v == 0.0 and world.cmd == (0.0, 0.0)             # 終わったら止まっている
    assert world.logic.reopen_count == 0
    kinds = stack.kinds()
    assert kinds.count("condition_confirmed") == 2 and kinds.count("condition_false_confirmed") == 1
    assert stack.mock.calls["plan"] == 1 and stack.mock.calls["point"] == 2


def test_plan_from_mock_matches_design_doc(make_stack):
    stack = make_stack(FakeWorld())
    plan = make_plan(stack.vlm, DEFAULT_INSTRUCTIONS).plan
    with open(os.path.join(TASK_PKG, "plans", "elevator_expected.json"), encoding="utf-8") as f:
        assert plan.to_dict() == parse_plan(f.read()).to_dict()


def test_does_not_leave_while_door_is_still_open_at_departure_floor(make_stack):
    # 扉を長く開けておく: 旋回が終わった時点でまだ出発階の扉が開いている（検討資料の図2の A）.
    # 「再び」は、閉じたのを見た後の「開いた」でだけ成り立つので、ここでは降りてはいけない.
    world = FakeWorld(ElevatorConfig(door_hold_s=20.0))
    stack = make_stack(world)
    result = run_task(stack)
    ref = world.referee.result()
    assert result.ok and ref["success"], (result.message, ref)
    assert ref["exited_wrong_floor_s"] is None
    events = stack.events.since(0, limit=100000)
    turn_end = next(e for e in events if e["kind"] == "skill_end" and e["step"] == 1)
    first_yes_after_turn = next(e for e in events if e["kind"] == "check" and e["id"] > turn_end["id"])
    assert first_yes_after_turn["answer"] == "yes"                # 開いた扉が見えているが…
    confirmed = [e for e in events if e["kind"] == "condition_confirmed" and e["step"] == 2][0]
    false_seen = [e for e in events if e["kind"] == "condition_false_confirmed" and e["step"] == 2][0]
    assert false_seen["id"] < confirmed["id"]                     # …発進は「閉」を確定した後


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_pointing_noise_is_tolerated(make_stack, seed):
    world = FakeWorld()
    stack = make_stack(world, noise=8.0, seed=seed)
    result = run_task(stack)
    ref = world.referee.result()
    assert result.ok and ref["success"], (result.message, ref)
    assert not world.wall_contact and not world.door_contact
    assert ref["back_wall_margin_m"] > 0.1 and ref["door_side_margin_m"] > 0.08


def test_repointing_while_driving(make_stack):
    world = FakeWorld()
    stack = make_stack(world, noise=5.0, seed=4, skill={"repoint_period_s": 0.5})
    result = run_task(stack)
    assert result.ok and world.referee.success, (result.message, world.referee.result())
    assert not world.wall_contact and not world.door_contact


def test_fixed_distance_plan_without_pointing(make_stack):
    world = FakeWorld()
    stack = make_stack(world)
    with open(os.path.join(TASK_PKG, "plans", "elevator_fixed_distance.json"), encoding="utf-8") as f:
        plan = parse_plan(f.read())
    result = run_task(stack, plan)
    assert result.ok and world.referee.success, (result.message, world.referee.result())
    assert stack.mock.calls["point"] == 0


def test_slow_vlm_misses_a_short_door_time(make_stack):
    # 判定が遅く、扉が開いている時間が短く、扉のセンサもないと乗り損ねる.
    # 2回続けて yes がそろう前に扉が閉まって置いていかれるか、閉まる扉に阻まれる.
    world = FakeWorld(ElevatorConfig(door_hold_s=2.0, door_sensor=False))
    stack = make_stack(world, latency=1.5)
    result = run_task(stack)
    assert not result.ok and not world.referee.success
    assert world.door_contact or world.referee.left_behind_at is not None
    assert world.referee.entered_car_at is None
    assert world.cmd == (0.0, 0.0)


def test_slow_vlm_succeeds_when_the_door_is_held_open(make_stack):
    # 同じ遅い VLM でも、人が「開」ボタンで扉を保持すれば通る（最初の実験のやり方）
    world = FakeWorld(ElevatorConfig(door_hold_s=2.0))
    stack = make_stack(world, latency=1.5)
    original_step = world.step

    def step_with_person() -> None:
        original_step()
        boarding = world.referee.entered_car_at is None
        leaving = world.referee.rode_at is not None and world.referee.exited_dest_at is None
        world.logic.set_hold_open(world.logic.phase in (OPENING, OPEN) and (boarding or leaving))

    world.step = step_with_person
    result = run_task(stack)
    assert result.ok and world.referee.success, (result.message, world.referee.result())


def test_occasional_wrong_answers_are_filtered_by_confirmation(make_stack):
    world = FakeWorld(ElevatorConfig(door_hold_s=10.0))
    stack = make_stack(world, flip=0.08, seed=7, executor={"confirm_count": 3})
    result = run_task(stack)
    ref = world.referee.result()
    assert ref["exited_wrong_floor_s"] is None and not world.door_contact, ref
    assert result.ok and ref["success"], (result.message, ref)


@pytest.mark.parametrize("pitch_error_deg,odom_scale", [(1.0, 1.0), (-1.0, 1.0), (0.0, 1.03), (0.0, 0.97)])
def test_small_calibration_errors_are_tolerated(make_stack, pitch_error_deg, odom_scale):
    probe = FakeWorld()
    cam = true_camera(probe.rp)
    assumed = CameraModel(cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height, cam.height_m,
                          cam.pitch_rad + math.radians(pitch_error_deg), cam.x_offset_m)
    world = FakeWorld(assumed_camera=assumed, odom_scale=odom_scale)
    stack = make_stack(world)
    result = run_task(stack)
    ref = world.referee.result()
    assert result.ok and ref["success"], (result.message, ref)
    assert not world.wall_contact and not world.door_contact


def test_stop_request_halts_the_robot(make_stack):
    world = FakeWorld()
    stack = make_stack(world)
    original_step = world.step

    def step_with_stop() -> None:
        original_step()
        if world.referee.entered_car_at is None and world.x > -0.5:      # 出入口へ向かっている途中で停止
            stack.cancel.set()

    world.step = step_with_stop
    result = run_task(stack)
    assert not result.ok and "停止" in result.message
    assert world.cmd == (0.0, 0.0) and world.v == 0.0
    assert world.referee.entered_car_at is None


def test_condition_timeout_when_elevator_never_comes(make_stack):
    world = FakeWorld()
    stack = make_stack(world)
    world.sleep(1.0)
    plan = parse_plan({"steps": [{"instruction": "扉が開いたら中に入って",
                                  "when": {"type": "when_true", "condition": "Elevator door is fully open."},
                                  "do": {"skill": "go_to_visual_target", "target": "center of the elevator car floor"},
                                  "timeout_s": 20}]})
    result = stack.executor.run(plan)                                      # エレベーターを呼ばない
    assert not result.ok and "時間切れ" in result.message
    assert 20.0 <= world.t <= 23.0 and world.x == pytest.approx(-1.0)


# -- 実行系のまとめ役と Web UI（HTTP の API）を通した試験 -------------------------
def _http(url, body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as resp:
        return resp.read()


def _wait(fn, timeout=30.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        value = fn()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError("時間内に条件が成り立ちませんでした")


def test_runtime_and_web_api_end_to_end(make_stack, tmp_path):
    world = FakeWorld()
    stack = make_stack(world)                     # モックの HTTP サーバだけ借りる
    cfg = load_config(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config",
                                   "task_sim.yaml"), env={"VLM_BASE_URL": stack.url, "VLM_MODEL": "mock-vlm"})
    cfg.log_dir = str(tmp_path)
    cfg.skills.repoint_period_s = 0.0
    cfg.web = WebConfig(host="127.0.0.1", port=0, buttons=cfg.web.buttons)
    hooks = {"stop": 0, "reset": 0}

    def stop_hook():
        hooks["stop"] += 1
        return True, "ok"

    def reset_hook():
        hooks["reset"] += 1
        return True, "ok"

    runtime = TaskRuntime(cfg, world, stop_hook=stop_hook, reset_hook=reset_hook,
                          triggers={"call": lambda: world.logic.call(world.t)},
                          extra_jpeg=lambda name: runtime.overlay_jpeg() if name == "view" else None)
    ui = WebUI(runtime, cfg.web.host, cfg.web.port)
    ui.start()
    base = f"http://127.0.0.1:{ui.port}"
    try:
        assert "画像と言語の指示" in _http(base + "/").decode("utf-8")
        assert _http(base + "/snapshot.jpg")[:2] == b"\xff\xd8"           # JPEG
        assert _http(base + "/extra/view.jpg")[:2] == b"\xff\xd8"
        with pytest.raises(urllib.error.HTTPError):
            _http(base + "/extra/unknown.jpg")
        state = json.loads(_http(base + "/api/state"))
        assert state["task_state"] == "idle" and [b["name"] for b in state["buttons"]] == ["call", "hold", "reset_sim"]
        # 計画がないうちは承認できない
        assert not json.loads(_http(base + "/api/execute", {}))["ok"]
        assert json.loads(_http(base + "/api/plan", {"instructions": "\n".join(state["instructions"])}))["ok"]
        state = _wait(lambda: (lambda s: s if s["plan_state"] in ("ready", "error") else None)(
            json.loads(_http(base + "/api/state"))))
        assert state["plan_state"] == "ready" and len(state["plan"]["steps"]) == 3
        assert json.loads(_http(base + "/api/execute", {}))["ok"]
        assert json.loads(_http(base + "/api/trigger", {"name": "call"}))["ok"]
        assert not json.loads(_http(base + "/api/trigger", {"name": "nope"}))["ok"]
        state = _wait(lambda: (lambda s: s if s["task_state"] != "running" else None)(
            json.loads(_http(base + "/api/state"))), timeout=120.0)
        assert state["task_state"] == "done", state["task_message"]
        assert [s["phase"] for s in state["executor"]["steps"]] == ["done", "done", "done"]
        world.sleep(1.0)
        assert world.referee.success, world.referee.result()
        # 停止と解除
        assert json.loads(_http(base + "/api/stop", {}))["ok"] and hooks["stop"] == 1
        assert not json.loads(_http(base + "/api/execute", {}))["ok"]     # 停止中は実行できない
        assert json.loads(_http(base + "/api/reset", {}))["ok"] and hooks["reset"] == 1
        # 計画の JSON を直接読み込む
        with open(os.path.join(TASK_PKG, "plans", "elevator_fixed_distance.json"), encoding="utf-8") as f:
            assert json.loads(_http(base + "/api/set_plan", {"plan": f.read()}))["ok"]
        assert not json.loads(_http(base + "/api/set_plan", {"plan": {"steps": []}}))["ok"]
        # VLM に渡した画像と記録がディスクに残っている
        run_dir = state["run_dir"]
        assert os.path.isfile(os.path.join(run_dir, "events.jsonl"))
        assert len(os.listdir(os.path.join(run_dir, "images"))) >= 5
    finally:
        runtime.close()
        ui.stop()
