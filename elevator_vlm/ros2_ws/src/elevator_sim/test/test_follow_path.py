"""道に沿って走る（follow_path）の通し試験. 道のある擬似世界、モック VLM（HTTP）、実行系をつなぐ.

確かめること:
  * 道に沿って、言われた距離だけ走る（直線と曲がり角）. 道から外れない.
  * VLM の答え1回で進むのは、その画像を撮った位置から最大 step_m. 答えが遅ければ止まって待つ.
  * 止まる条件、終了の指示（ここまで）、範囲を指定した実行、使えない答えが続いたときの中止.
  * 実行系のまとめ役と Web UI: 承認して開始の合図を待つ → start → finish.
"""
import json
import threading
import time
import urllib.request
from dataclasses import dataclass

import pytest
from fake_road import FakeRoad, RoadSimVlm

from elevator_sim.mock_vlm import PATH_END_CONDITION, PATH_TARGET, MockConfig, MockVlm, make_server
from elevator_sim.road_world import RoadParams, curve_points
from elevator_sim.world_params import WorldParams
from elevator_task.core.config import TaskConfig
from elevator_task.core.events import EventLog
from elevator_task.core.executor import (PHASE_DONE, PHASE_FINISHED, PHASE_SKIPPED, ExecutorConfig,
                                         TaskExecutor)
from elevator_task.core.http_util import urlopen
from elevator_task.core.perception import Perception, PerceptionConfig
from elevator_task.core.plan import PlanLimits, parse_plan
from elevator_task.core.planner import make_plan
from elevator_task.core.runtime import TaskRuntime
from elevator_task.core.skills import FollowPathConfig, SkillConfig, Skills
from elevator_task.core.vlm_client import VlmClient, VlmConfig
from elevator_task.core.webui import WebUI


@dataclass
class RoadStack:
    world: FakeRoad
    mock: MockVlm
    events: EventLog
    cancel: threading.Event
    finish: threading.Event
    skills: Skills
    executor: TaskExecutor
    url: str

    def kinds(self):
        return [e["kind"] for e in self.events.since(0, limit=100000)]

    def of(self, kind):
        return [e for e in self.events.since(0, limit=100000) if e["kind"] == kind]


def _statement(world: FakeRoad):
    return lambda st: world.landmark_ahead() if st == PATH_END_CONDITION else False


@pytest.fixture
def make_road():
    servers = []

    def _make(road=None, latency=0.5, path=None, executor=None, path_fn=None, bounded=True) -> RoadStack:
        world = FakeRoad(road, bounded=bounded)
        mock = MockVlm(WorldParams(), MockConfig(latency_s=0.0), lambda: None, lambda: world.camera,
                       path_fn=path_fn or world.path_answer, statement_fn=_statement(world))
        server = make_server(mock, "127.0.0.1", 0)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
        url = f"http://127.0.0.1:{server.server_address[1]}/v1"
        vlm = RoadSimVlm(VlmConfig(base_url=url, model="mock-vlm"), world, latency)
        events = EventLog(None, keep=100000)
        cancel, finish = threading.Event(), threading.Event()
        perception = Perception(world, vlm, PerceptionConfig(), events)
        skills = Skills(world, perception, SkillConfig(follow_path=FollowPathConfig(**(path or {}))), events,
                        cancel, finish=finish, path_cap_m=100.0)
        ex = TaskExecutor(world, perception, skills, ExecutorConfig(**(executor or {})), events, cancel, finish=finish)
        return RoadStack(world, mock, events, cancel, finish, skills, ex, url)

    yield _make
    for server in servers:
        server.shutdown()
        server.server_close()


def path_step(distance=0.0, stop_when="", instruction="道に沿って走って", timeout=300):
    return {"instruction": instruction, "when": {"type": "now"},
            "do": {"skill": "follow_path", "path": PATH_TARGET, "distance_m": distance, "stop_when": stop_when},
            "timeout_s": timeout}


def plan_of(*steps):
    return parse_plan({"steps": list(steps)})


# -- 言われた距離だけ、道に沿って走る -----------------------------------------
def test_follows_straight_road_for_the_distance(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (40.0, 0.0)]))
    res = st.executor.run(plan_of(path_step(6.0, instruction="道に沿って6m走って")))
    assert res.ok, res.message
    assert 5.9 <= st.world.travelled <= 6.3
    assert st.world.max_lateral < 0.25 and not st.world.left_road
    assert "道に沿って 5.9 m 走りました" in st.executor.statuses[0].message or "6.0 m" in st.executor.statuses[0].message
    end = st.of("path_end")[-1]
    assert end["ok"] and end["accepted"] >= 6           # 1 m ずつなら、少なくとも 6 回は答えを使う


def test_follows_a_curve_without_leaving_the_road(make_road):
    road = RoadParams(points=curve_points(straight1=6.0, radius=5.0, angle_deg=90.0, straight2=12.0), width_m=2.4)
    st = make_road(road)
    res = st.executor.run(plan_of(path_step(20.0)))
    assert res.ok, res.message
    s, lat = st.world.along()
    assert s > 19.0                                     # 角を曲がり、中心線に沿って先まで進んだ
    assert not st.world.left_road and st.world.max_lateral < 0.9
    assert abs(lat) < 0.5


# -- 1回の答えで進めるのは最大 step_m -----------------------------------------
def test_each_answer_allows_at_most_step_m(make_road):
    # 推論に 6 秒かかる. 0.3 m/s で走り続ければ答えの間に 1.8 m 進んでしまうが、1 回の答えでは 1.0 m まで
    st = make_road(RoadParams(points=[(0.0, 0.0), (40.0, 0.0)]), latency=6.0, path={"step_m": 1.0})
    res = st.executor.run(plan_of(path_step(5.0)))
    assert res.ok, res.message
    targets = st.of("path_target")
    assert targets, "答えを使っていない"
    first_wall = targets[0]["t"]
    allows = [(e["t"], e["allow_until_m"]) for e in targets]
    for i, (t, travelled) in enumerate(st.world.trace):
        if t < first_wall:
            assert travelled < 0.01                     # 最初の答えが来るまでは動かない
            continue
        allowed = max(a for (ta, a) in allows if ta <= t)
        assert travelled <= allowed + 0.05, (t, travelled, allowed)
    # 答えを待って止まっていた時間がある（止まって待つ）
    stops = [e for e in st.world.cmd_log if e[0] > first_wall + 1.0 and e[1] == 0.0 and e[2] == 0.0]
    assert len(stops) > 20
    # 撮影した時点から数える: 次の答えの撮影位置は、前の答えの撮影位置より先にある
    assert all(b[1] >= a[1] for a, b in zip(allows, allows[1:]))


def test_fast_answers_keep_the_robot_moving(make_road):
    # 推論が速ければ（0.5 秒）、止まらずに走り続ける
    st = make_road(RoadParams(points=[(0.0, 0.0), (40.0, 0.0)]), latency=0.5)
    st.executor.run(plan_of(path_step(8.0)))
    first = st.of("path_target")[0]["t"]
    moving = [v for (t, v, w) in st.world.cmd_log if first + 1.0 < t < first + 20.0]
    assert moving and sum(1 for v in moving if v == 0.0) / len(moving) < 0.1


# -- 止まる条件、ここまで、範囲 -------------------------------------------------
def test_stops_when_the_condition_becomes_true(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (40.0, 0.0)], landmark_s=12.0, landmark_radius_m=2.0))
    res = st.executor.run(plan_of(path_step(0.0, PATH_END_CONDITION, instruction="目印が見えるまで道に沿って走って")))
    assert res.ok, res.message
    assert "止まる条件" in st.executor.statuses[0].message
    assert 9.5 < st.world.travelled < 11.9
    assert "path_stop_condition" in st.kinds()


def test_finish_stops_here_and_skips_the_rest(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (60.0, 0.0)]))
    plan = plan_of(path_step(0.0), {"instruction": "左に90度回って", "when": {"type": "now"},
                                    "do": {"skill": "rotate", "angle_deg": 90}, "timeout_s": 30})
    box = {}
    thread = threading.Thread(target=lambda: box.setdefault("res", st.executor.run(plan)))
    thread.start()
    give_up = time.monotonic() + 60.0
    while st.world.travelled < 3.0 and time.monotonic() < give_up:
        time.sleep(0.01)
    st.finish.set()
    thread.join(60.0)
    res = box["res"]
    assert res.ok and res.finished, res.message
    assert st.executor.statuses[0].phase == PHASE_FINISHED
    assert st.executor.statuses[1].phase == PHASE_SKIPPED
    assert abs(st.world.yaw) < 0.2                      # 旋回はしていない
    assert st.world.cmd_log[-1][1:] == (0.0, 0.0)


def test_runs_only_the_selected_range(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (40.0, 0.0)]))
    plan = plan_of({"instruction": "1秒待って", "when": {"type": "now"}, "do": {"skill": "wait", "seconds": 1},
                    "timeout_s": 30},
                   path_step(3.0),
                   {"instruction": "左に90度回って", "when": {"type": "now"},
                    "do": {"skill": "rotate", "angle_deg": 90}, "timeout_s": 30})
    res = st.executor.run(plan, 1, 1)
    assert res.ok and "手順2〜2" in res.message
    phases = [s.phase for s in st.executor.statuses]
    assert phases == [PHASE_SKIPPED, PHASE_DONE, PHASE_SKIPPED]
    assert 2.9 <= st.world.travelled <= 3.3 and abs(st.world.yaw) < 0.2
    bad = st.executor.run(plan, 2, 1)
    assert not bad.ok and "範囲" in bad.message


# -- 使えない答え、来ない答え ---------------------------------------------------
def test_aborts_when_answers_keep_failing(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (40.0, 0.0)]), path_fn=lambda: None,
                   path={"max_point_failures": 4})
    res = st.executor.run(plan_of(path_step(5.0)))
    assert not res.ok and "道の指差しが続けて使えません" in res.message
    assert st.world.travelled < 0.01
    assert len(st.of("path_point_rejected")) == 4


def test_aborts_when_no_new_images_arrive(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (40.0, 0.0)]), path={"max_answer_wait_s": 5.0})
    st.world.frames_on = False
    res = st.executor.run(plan_of(path_step(5.0)))
    assert not res.ok and "使える答え" in res.message


def test_cancel_stops_with_failure(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (60.0, 0.0)]))
    box = {}
    thread = threading.Thread(target=lambda: box.setdefault("res", st.executor.run(plan_of(path_step(0.0)))))
    thread.start()
    give_up = time.monotonic() + 60.0
    while st.world.travelled < 1.5 and time.monotonic() < give_up:
        time.sleep(0.01)
    st.cancel.set()
    thread.join(60.0)
    assert not box["res"].ok and not box["res"].finished


# -- 計画づくり（モック VLM の規則と、プロンプトに書いた既定の距離） ----------------
def test_planner_writes_follow_path_steps(make_road):
    st = make_road()
    vlm = VlmClient(VlmConfig(base_url=st.url, model="mock-vlm"))
    lines = ["道に沿って走って", "道に沿って20m走って", "目印が見えるまで道に沿って走って"]
    result = make_plan(vlm, lines, limits=PlanLimits(default_path_distance_m=7.0))
    assert result.plan is not None, result.error
    dos = [s.do for s in result.plan.steps]
    assert [d.skill for d in dos] == ["follow_path"] * 3
    assert [d.distance_m for d in dos] == [7.0, 20.0, 0.0]          # 距離を言わなければプロンプトの既定値
    assert dos[0].stop_when == "" and dos[2].stop_when == PATH_END_CONDITION


# -- 実行系のまとめ役と Web UI: 承認して開始の合図を待つ → start → finish -----------------
def _wait(pred, seconds=60.0):
    give_up = time.monotonic() + seconds
    while not pred() and time.monotonic() < give_up:
        time.sleep(0.01)
    return pred()


def _post(port, path, body=None):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=json.dumps(body or {}).encode("utf-8"),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _state(port):
    with urlopen(f"http://127.0.0.1:{port}/api/state", timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def test_runtime_and_webui_arm_start_finish(make_road):
    st = make_road(RoadParams(points=[(0.0, 0.0), (80.0, 0.0)]))
    cfg = TaskConfig(log_dir="", vlm=VlmConfig(base_url=st.url, model="mock-vlm"))
    runtime = TaskRuntime(cfg, st.world)
    runtime.perception.vlm = RoadSimVlm(VlmConfig(base_url=st.url, model="mock-vlm"), st.world, 0.5)
    web = WebUI(runtime, "127.0.0.1", 0)
    web.start()
    try:
        plan = {"steps": [path_step(0.0),
                          {"instruction": "左に90度回って", "when": {"type": "now"},
                           "do": {"skill": "rotate", "angle_deg": 90}, "timeout_s": 30}]}
        assert _post(web.port, "/api/set_plan", {"plan": plan})["ok"]
        # 承認していない計画は、開始の合図では走らない
        r = _post(web.port, "/api/command", {"command": "start"})
        assert not r["ok"] and "承認" in r["message"]
        assert _post(web.port, "/api/arm", {"from": 1, "to": 1})["ok"]
        s = _state(web.port)
        assert s["drive"]["armed"] == [1, 1] and not s["drive"]["active"]
        assert s["follow_path_cfg"]["step_m"] == 1.0 and s["follow_path_cfg"]["default_distance_m"] == 10.0
        # ROS のトピックから来るのと同じ合図（文字列）
        ok, msg = runtime.command("start", source="ros")
        assert ok, msg
        assert _wait(lambda: st.world.travelled > 2.0)
        s = _state(web.port)
        assert s["drive"]["active"] and s["drive"]["range"] == [1, 1] and s["drive"]["skill"] == "follow_path"
        assert s["path"]["active"] and s["path"]["step_m"] == 1.0 and s["path"]["traveled_m"] > 1.0
        assert s["last_command"]["source"] == "ros"
        assert _post(web.port, "/api/finish")["ok"]
        runtime.wait_task(60.0)
        s = _state(web.port)
        assert s["task_state"] == "done" and s["drive"]["result"] == "finished" and not s["drive"]["active"]
        assert [x["phase"] for x in s["executor"]["steps"]] == [PHASE_FINISHED, PHASE_SKIPPED]
        assert abs(st.world.yaw) < 0.2
        # 承認は残る（次の区間を、範囲を変えて走らせられる）
        assert s["drive"]["armed"] == [1, 1]
        ok, msg = runtime.command('{"command": "start", "from": 2, "to": 2}', source="ros")
        assert ok, msg
        runtime.wait_task(60.0)
        assert runtime.last_result == "done" and abs(st.world.yaw - 1.5708) < 0.1
        # 停止は承認も取り消す
        runtime.stop()
        assert runtime.armed is None
        assert not runtime.command("start")[0]
        assert not runtime.command("jump")[0]
    finally:
        web.stop()
        runtime.close()
