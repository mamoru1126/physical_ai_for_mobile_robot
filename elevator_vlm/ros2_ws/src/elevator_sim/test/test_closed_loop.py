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
from elevator_sim.mock_vlm import DOOR_CONDITION, TARGET_CAR, MockConfig, MockVlm, make_server
from elevator_sim.world_params import door_ratio_at_robot, robot_in_car, view_state
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
from elevator_task.core.http_util import urlopen
from elevator_task.core.webui import WebUI

TASK_PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "elevator_task")
SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))), "scripts", "inside")


def load_script(name: str):
    """scripts/inside/ のスクリプトをモジュールとして読み込む."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def world_status(world: FakeWorld):
    """シミュレーションで /elevator/state と /sim/referee に流れるものと同じ中身."""
    state = world.logic.state()
    state.update(view_state(world.wp, world.truth()))
    return {"elevator": state, "referee": world.referee.result()}


class DoorViewVlm(MockVlm):
    """本物の VLM が「The door is open.」に返した答え方をまねる（最初の実験の記録から）.

    扉のパネルが見えていて、すき間があるときに yes と答える. 乗場からは半分以上開いたら yes.
    かごの中の近くからは、開き切ると扉が画面の外に出るので no、動いている途中だけ yes.
    """

    def _check(self, text: str) -> str:
        truth = self._truth_fn()
        ratio = door_ratio_at_robot(self.p, truth)
        answer = (0.05 < ratio < 0.95) if robot_in_car(self.p, truth) else ratio >= 0.5
        return json.dumps({"answer": "yes" if answer else "no"})


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

    def _make(world: FakeWorld, latency=0.3, noise=0.0, flip=0.0, seed=0, skill=None, executor=None,
              run_dir=None, mock_cls=MockVlm) -> Stack:
        mock = mock_cls(world.wp, MockConfig(latency_s=0.0, point_noise_px=noise, check_flip_prob=flip, seed=seed),
                        world.truth, lambda: world.camera_true)
        server = make_server(mock, "127.0.0.1", 0)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
        url = f"http://127.0.0.1:{server.server_address[1]}/v1"
        vlm = SimLatencyVlm(VlmConfig(base_url=url, model="mock-vlm"), world, latency)
        events = EventLog(run_dir, keep=100000)
        events.context_fn = lambda: {"elevator": world_status(world)["elevator"]}      # 記録に真値を添える
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
                                  "when": {"type": "when_true", "condition": DOOR_CONDITION},
                                  "do": {"skill": "go_to_visual_target", "target": TARGET_CAR},
                                  "timeout_s": 20}]})
    result = stack.executor.run(plan)                                      # エレベーターを呼ばない
    assert not result.ok and "時間切れ" in result.message
    assert 20.0 <= world.t <= 23.0 and world.x == pytest.approx(-1.0)


def test_export_labelled_samples_from_a_run(make_stack, tmp_path):
    # 記録に添えた真値から、正解つきの画像の組（机上評価用）が作れる.
    # 扉を長く開けておくと、かごの中から開いた扉を見ている画像（出発階）も取れる
    module = load_script("export_samples")
    run_dir = str(tmp_path / "run")
    world = FakeWorld(ElevatorConfig(door_hold_s=20.0))
    stack = make_stack(world, run_dir=run_dir)
    assert run_task(stack).ok
    stack.events.close()
    samples = module.select(module.load_events(run_dir))
    checks = {(s["place"], s["floor"], s["door"]) for s in samples if s["kind"] == "check"}
    assert {("hall", 1, "closed"), ("hall", 1, "open"), ("car", 1, "open"), ("car", 3, "open")} <= checks
    assert any(place == "car" and door == "closed" for place, _, door in checks)
    assert any(door == "partial" for _, _, door in checks)
    assert all(0.15 <= s["door_ratio"] <= 0.85 for s in samples if s["kind"] == "check" and s["door"] == "partial")
    points = {(s["place"], s["floor"]): s for s in samples if s["kind"] == "point"}
    assert set(points) == {("hall", 1), ("car", 3)}
    assert points[("hall", 1)]["goal_ahead_m"] == pytest.approx(1.0 + world.wp.car_center_x, abs=0.02)
    assert points[("hall", 1)]["door_ahead_m"] == pytest.approx(1.0, abs=0.02)
    assert points[("car", 3)]["goal_ahead_m"] == pytest.approx(world.wp.car_center_x + 1.0, abs=0.05)
    assert all(os.path.isfile(os.path.join(run_dir, s["image"])) for s in samples)
    names = [module.file_name(s, i) for i, s in enumerate(samples)]
    assert len(set(names)) == len(names)


def test_analysis_agrees_with_truth_for_a_perfect_vlm(make_stack, tmp_path):
    # 記録の分析: 真値どおりに答える VLM では、食い違いも、気づいた点も出ない
    module = load_script("analyze_run")
    run_dir = str(tmp_path / "run")
    world = FakeWorld(ElevatorConfig(door_hold_s=20.0))
    stack = make_stack(world, run_dir=run_dir)
    assert run_task(stack).ok
    stack.events.emit("status", task_ok=True, status=world_status(world))
    stack.events.close()
    analysis = module.Analysis(module.load_events(run_dir))
    text = analysis.run(run_dir)
    assert "食い違い" not in text and analysis.findings == [], text
    assert "かごの中・全開" in text and "乗場・全閉" in text
    assert "真値: 成功: 行先階で降りました" in text
    assert "敷居の先 1.00 m" in text                       # 降りるときの目標


def test_analysis_flags_the_answers_of_the_first_real_run(make_stack, tmp_path):
    # 最初の実験の VLM の答え方（DoorViewVlm）を入れると、分析がその食い違いを拾う
    module = load_script("analyze_run")
    run_dir = str(tmp_path / "run")
    world = FakeWorld(ElevatorConfig(door_hold_s=12.0))
    stack = make_stack(world, run_dir=run_dir, mock_cls=DoorViewVlm)
    with open(os.path.join(TASK_PKG, "plans", "elevator_expected.json"), encoding="utf-8") as f:
        data = json.load(f)
    for step in data["steps"]:
        step["timeout_s"] = min(step["timeout_s"], 60)
    run_task(stack, parse_plan(data))
    stack.events.emit("status", task_ok=True, status=world_status(world))
    stack.events.close()
    analysis = module.Analysis(module.load_events(run_dir))
    text = analysis.run(run_dir)
    assert any("扉が全開なのに no" in f for f in analysis.findings), text
    assert any("途中" in f and "yes" in f for f in analysis.findings), text
    assert any("「一度閉じた」と確定" in f for f in analysis.findings), text


def test_phrase_evaluation_tallies_answers_by_place_and_door_state(make_stack, tmp_path):
    # 言い回しの比較（scripts/inside/eval_phrases.py）: 記録の画像を真値で分類し、答えの当たり外れを数える.
    # ここでは、いつも yes と答え、いつも同じ点を指す VLM を使って、集計と報告の形だけを確かめる
    from elevator_task.core.vlm_client import VlmClient, VlmReply

    module = load_script("eval_phrases")
    run_dir = str(tmp_path / "20260101_000000")
    world = FakeWorld(ElevatorConfig(door_hold_s=20.0))
    stack = make_stack(world, run_dir=run_dir)
    stack.events.emit("plan_ready", plan={"steps": [
        {"when": {"type": "when_true", "condition": "The door is open."},
         "do": {"skill": "go_to_visual_target", "target": "floor inside the room"}},
        {"when": {"type": "now"}, "do": {"skill": "rotate", "angle_deg": 180}},
        {"when": {"type": "after_false_then_true", "condition": "The door is open."},
         "do": {"skill": "go_to_visual_target", "target": "floor outside the room"}}]})
    assert run_task(stack).ok
    stack.events.close()
    assert module.plan_phrases(run_dir) == (["The door is open."], {"hall": ["floor inside the room"],
                                                                    "car": ["floor outside the room"]})

    class Canned(VlmClient):
        def chat(self, messages, max_tokens, schema_name=None, **kwargs):
            # 点は画像の中央の少し下（床の上で、前 1.01 m）
            text = '{"point_2d": [500, 700]}' if schema_name == "point_target" else '{"answer": "yes"}'
            return VlmReply(text, 0.2)

    cfg = load_config(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config",
                                   "task_sim.yaml"))
    vlm = Canned(cfg.vlm)
    samples = module.load_samples(str(tmp_path / "no_samples"), [run_dir], 2)
    checks = [s for s in samples if s["kind"] == "check"]
    points = [s for s in samples if s["kind"] == "point"]
    frames = {s["path"]: module.frame_of(s["path"]) for s in samples}
    cond = module.eval_conditions(cfg, vlm, ["X is open."], ["guided", "basic"], checks, frames, 2)
    assert [r["system"] for r in cond] == ["guided", "basic"]
    for r in cond:
        by = r["by_category"]
        assert by["hall/open"][0] == by["hall/open"][1] > 0 and by["car/open"][0] == by["car/open"][1] > 0
        assert by["hall/closed"][0] == 0 and by["car/closed"][0] == 0          # いつも yes なので、全閉は全部外れ
        assert r["correct"] == sum(1 for c in checks if c["door"] == "open") and r["total"] == len(checks)
        assert {w["door"] for w in r["wrong"]} == {"closed", "partial"}
    targ = module.eval_targets(cfg, vlm, {"hall": ["a"], "car": ["b"]}, points, frames, 2)
    by_place = {r["place"]: r for r in targ}
    # 乗場（敷居まで 1.0 m）からは敷居の上で、かごに入り切らない. かごの中央（敷居まで 0.78 m）からは
    # 敷居の先 0.24 m で、出入口に残る. どちらも NG
    assert by_place["hall"]["results"][0]["beyond_m"] == pytest.approx(0.01, abs=0.03)
    assert by_place["car"]["results"][0]["beyond_m"] == pytest.approx(0.235, abs=0.05)
    assert by_place["hall"]["ok"] == 0 and by_place["car"]["ok"] == 0
    text = module.report("canned", checks, points, cond, targ)
    assert "[C1] X is open." in text and "合計の高い順" in text and "[T2] b" in text and "かごの中: 全閉 0/" in text


def test_recheck_before_departure_catches_a_door_that_closed_while_pointing(make_stack):
    # 条件が確定した後、指差しの推論に時間がかかっている間に扉が閉まった.
    # 発進の直前の確認があれば発進しない. なければ、閉まった扉に向かって走り出す
    def run(recheck: bool):
        world = FakeWorld(ElevatorConfig(door_hold_s=2.0, door_sensor=False))
        stack = make_stack(world, executor={"recheck_before_departure": recheck})
        chat = stack.vlm.chat

        def slow_pointing(messages, *args, **kwargs):
            reply = chat(messages, *args, **kwargs)
            if kwargs.get("schema_name") == "point_target":
                world.sleep(3.0)
            return reply

        stack.vlm.chat = slow_pointing
        plan = parse_plan({"steps": [{"instruction": "扉が開いたら中に入って",
                                      "when": {"type": "when_true", "condition": DOOR_CONDITION},
                                      "do": {"skill": "go_to_visual_target", "target": TARGET_CAR},
                                      "timeout_s": 25}]})
        result = run_task(stack, plan)
        return world, stack, result

    world, stack, result = run(recheck=True)
    assert "departure_recheck_failed" in stack.kinds()
    assert not world.door_contact and world.x == pytest.approx(-1.0)          # 動いていない
    assert not result.ok and "時間切れ" in result.message
    world, stack, result = run(recheck=False)
    assert world.door_contact


# -- 実行系のまとめ役と Web UI（HTTP の API）を通した試験 -------------------------
def _http(url, body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                 headers={"Content-Type": "application/json"})
    with urlopen(req, timeout=10) as resp:
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
    cfg.plan_dir = os.path.join(TASK_PKG, "plans")
    cfg.skills.repoint_period_s = 0.0
    cfg.web = WebConfig(host="127.0.0.1", port=0, buttons=cfg.web.buttons, status_topics=cfg.web.status_topics)
    hooks = {"stop": 0, "reset": 0}

    def stop_hook():
        hooks["stop"] += 1
        return True, "ok"

    def reset_hook():
        hooks["reset"] += 1
        return True, "ok"

    runtime = TaskRuntime(cfg, world, stop_hook=stop_hook, reset_hook=reset_hook,
                          triggers={"call": lambda: world.logic.call(world.t)},
                          extra_jpeg=lambda name: runtime.overlay_jpeg() if name == "view" else None,
                          status_fn=lambda: world_status(world))
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
        # シミュレーションの真値が画面に出ていて、終わり方の表示にも真値による成否が添えられている
        assert [s["name"] for s in state["status"]] == ["elevator", "referee"]
        assert state["status"][0]["data"]["door"] in ("closed", "partial", "open")
        assert "成否の判定（真値）: 成功" in state["task_message"]
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
        # 用意済みの計画
        assert state["presets"] == ["elevator_expected.json", "elevator_fixed_distance.json"]
        assert json.loads(_http(base + "/api/load_preset", {"name": "elevator_expected.json"}))["ok"]
        assert not json.loads(_http(base + "/api/load_preset", {"name": "../config/task.yaml"}))["ok"]
        # VLM に渡した画像と記録がディスクに残っている
        run_dir = state["run_dir"]
        assert os.path.isfile(os.path.join(run_dir, "events.jsonl"))
        assert len(os.listdir(os.path.join(run_dir, "images"))) >= 5
        with open(os.path.join(run_dir, "events.jsonl"), encoding="utf-8") as f:
            events = [json.loads(line) for line in f]
        checks = [e for e in events if e["kind"] == "check"]
        # 判定の記録には、設定で選んだ真値の項目だけが添えられる
        assert checks and all(set(e["ctx"]) == {"elevator"} for e in checks)
        assert {"door", "door_ratio", "robot_in_car", "goal_ahead_m"} <= set(checks[0]["ctx"]["elevator"])
        assert "lift_vel" not in checks[0]["ctx"]["elevator"]
        final = [e for e in events if e["kind"] == "status"]
        assert len(final) == 1 and final[0]["task_ok"] and final[0]["status"]["referee"]["success"]
        assert any(e["kind"] == "point_ground" for e in events)
    finally:
        runtime.close()
        ui.stop()
