"""中核部分の単体テスト（幾何、計画の検査、VLM の返答の読み取り、安全ゲート、設定）."""
import json
import math
import os

import pytest

from elevator_task.core import config as config_mod
from elevator_task.core.geometry import (CameraModel, Pose2D, PoseHistory, ground_to_pixel, odom_to_robot,
                                         pixel_to_ground, robot_to_odom, wrap_angle)
from elevator_task.core.plan import PlanError, parse_plan
from elevator_task.core.planner import split_instructions
from elevator_task.core.safety import (STATE_CAMERA_LOST, STATE_IDLE, STATE_ODOM_LOST, STATE_OK, STATE_STOPPED,
                                       SafetyConfig, SafetyGate)
from elevator_task.core.vlm_client import extract_json, parse_point, parse_yes_no, yes_probability

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAM = CameraModel(600.0, 600.0, 320.0, 240.0, 640, 480, 0.5, math.radians(15.0), 0.2)


# -- 幾何 -------------------------------------------------------------------
@pytest.mark.parametrize("x,y", [(1.0, 0.0), (1.7, 0.3), (2.5, -0.6), (0.95, 0.1)])
def test_ground_projection_roundtrip(x, y):
    u, v = ground_to_pixel(CAM, x, y)
    gx, gy = pixel_to_ground(CAM, u, v)
    assert gx == pytest.approx(x, abs=1e-6) and gy == pytest.approx(y, abs=1e-6)


def test_projection_matches_design_doc_formula():
    # 検討資料 5.3 節の式（カメラがロボット中心にある場合）: 画像の中心の画素は h / tan(俯角) 先の床
    cam = CameraModel(600.0, 600.0, 320.0, 240.0, 640, 480, 0.5, math.radians(15.0), 0.0)
    x, y = pixel_to_ground(cam, 320.0, 240.0)
    assert x == pytest.approx(0.5 / math.tan(math.radians(15.0)), rel=1e-6) and y == pytest.approx(0.0)
    # 画像の左（u が小さい）はロボットの左（Y が正）
    assert pixel_to_ground(cam, 200.0, 300.0)[1] > 0.0


def test_pixels_above_horizon_do_not_hit_ground():
    assert pixel_to_ground(CAM, 320.0, CAM.horizon_v() - 1.0) is None
    assert pixel_to_ground(CAM, 320.0, CAM.horizon_v() + 5.0) is not None
    assert ground_to_pixel(CAM, -1.0, 0.0) is None          # カメラの後ろ


def test_floor_visible_from_about_70cm_ahead_of_camera():
    assert CAM.nearest_ground_m() - CAM.x_offset_m == pytest.approx(0.67, abs=0.03)


def test_odom_transforms_and_history():
    pose = Pose2D(0.0, 1.0, 2.0, math.radians(90.0))
    xo, yo = robot_to_odom(pose, 1.0, 0.0)
    assert (xo, yo) == pytest.approx((1.0, 3.0))
    assert odom_to_robot(pose, xo, yo) == pytest.approx((1.0, 0.0))
    assert wrap_angle(math.radians(190.0)) == pytest.approx(math.radians(-170.0))
    hist = PoseHistory(horizon_s=5.0)
    hist.add(Pose2D(0.0, 0.0, 0.0, math.radians(170.0)))
    hist.add(Pose2D(1.0, 1.0, 0.0, math.radians(-170.0)))
    mid = hist.at(0.5)
    assert mid.x == pytest.approx(0.5) and abs(wrap_angle(mid.yaw - math.pi)) < 1e-6   # ±180 度をまたいで補間
    assert hist.at(-1.0).x == 0.0 and hist.at(9.0).x == 1.0
    hist.add(Pose2D(0.2, 5.0, 0.0, 0.0))                     # 時刻の巻き戻り → 履歴を捨てる
    assert hist.at(0.0).x == 5.0


# -- 計画 -------------------------------------------------------------------
@pytest.mark.parametrize("name", ["elevator_expected.json", "elevator_fixed_distance.json"])
def test_bundled_plans_are_valid(name):
    with open(os.path.join(PKG, "plans", name), encoding="utf-8") as f:
        plan = parse_plan(f.read())
    assert [s.when.type for s in plan.steps] == ["when_true", "now", "after_false_then_true"]
    assert parse_plan(plan.to_dict()).to_dict() == plan.to_dict()        # 書き出して読み直しても同じ


def _step(**over):
    step = {"instruction": "x", "when": {"type": "now"}, "do": {"skill": "rotate", "angle_deg": 90}, "timeout_s": 10}
    step.update(over)
    return {"steps": [step]}


@pytest.mark.parametrize("bad", [
    {"steps": []},
    {"nosteps": 1},
    _step(when={"type": "sometime"}),
    _step(when={"type": "when_true"}),                                   # 条件文がない
    _step(do={"skill": "fly"}),
    _step(do={"skill": "move_straight", "distance_m": 5.0}),             # 3 m を超える
    _step(do={"skill": "move_straight", "distance_m": "far"}),
    _step(do={"skill": "rotate", "angle_deg": 0}),
    _step(do={"skill": "go_to_visual_target", "target": " "}),
    _step(do={"skill": "wait", "seconds": -1}),
    _step(timeout_s=0),
    "not json",
])
def test_invalid_plans_are_rejected(bad):
    with pytest.raises(PlanError):
        parse_plan(bad)


def test_plan_schema_has_one_variant_per_skill_and_when_type():
    from elevator_task.core.plan import PLAN_JSON_SCHEMA, SKILLS, WHEN_TYPES
    item = PLAN_JSON_SCHEMA["properties"]["steps"]["items"]["properties"]
    skills = [v["properties"]["skill"]["enum"] for v in item["do"]["anyOf"]]
    assert sorted(sum(skills, [])) == sorted(SKILLS) and all(len(s) == 1 for s in skills)
    whens = sum((v["properties"]["type"]["enum"] for v in item["when"]["anyOf"]), [])
    assert sorted(whens) == sorted(WHEN_TYPES)
    for variant in item["do"]["anyOf"] + item["when"]["anyOf"]:
        assert set(variant["required"]) == set(variant["properties"]) and variant["additionalProperties"] is False


def test_split_instructions():
    assert split_instructions(" a \r\n\n b\n") == ["a", "b"]


# -- VLM の返答の読み取り -----------------------------------------------------
@pytest.mark.parametrize("text,expected", [
    ('{"answer": "yes"}', True), ('{"answer":"no"}', False), ("Yes.", True), ("no", False),
    ('```json\n{"answer": "yes"}\n```', True), ("<think>the door looks open, yes</think>\n{\"answer\": \"no\"}", False),
    ("The answer is No, it is closed.", False), ('{"answer": true}', True), ("unclear", None), ("", None),
])
def test_parse_yes_no(text, expected):
    assert parse_yes_no(text) is expected


@pytest.mark.parametrize("text,expected", [
    ('{"point_2d": [512, 730]}', (512.0, 730.0)),
    ('```json\n[\n\t{"point_2d": [925, 618], "label": "point_1"}\n]\n```', (925.0, 618.0)),   # Qwen の公式の形
    ('<points x1="45.5" y1="60.2" alt="floor">floor</points>', (45.5, 60.2)),
    ("The point is (100, 200).", (100.0, 200.0)),
    ('{"x": 3, "y": 4}', (3.0, 4.0)),
    ("[640, 360]", (640.0, 360.0)),
    ("I cannot find it.", None),
])
def test_parse_point(text, expected):
    assert parse_point(text) == expected


def test_extract_json_and_yes_probability():
    assert extract_json('noise {"a": [1, 2]} tail') == {"a": [1, 2]}
    assert extract_json("no json here") is None
    logprobs = [{"token": '{"', "logprob": -0.1},
                {"token": "yes", "logprob": math.log(0.8),
                 "top_logprobs": [{"token": "yes", "logprob": math.log(0.8)}, {"token": "no", "logprob": math.log(0.2)}]}]
    assert yes_probability(logprobs) == pytest.approx(0.8)
    assert yes_probability(None) is None


# -- 安全ゲート ---------------------------------------------------------------
def test_safety_gate_limits_and_watchdogs():
    gate = SafetyGate(SafetyConfig(max_linear=0.5, max_angular=1.0, cmd_timeout_s=0.5, odom_timeout_s=1.0,
                                   camera_timeout_s=2.0))
    assert gate.output(0.0)[2] == STATE_ODOM_LOST            # 何も届いていない
    gate.on_odom(0.0)
    assert gate.output(0.1)[2] == STATE_CAMERA_LOST
    gate.on_camera(0.0)
    assert gate.output(0.1) == (0.0, 0.0, STATE_IDLE)
    gate.on_cmd(0.1, 2.0, -3.0)
    assert gate.output(0.2) == (0.5, -1.0, STATE_OK)         # 上限で切る
    assert gate.output(0.7)[2] == STATE_IDLE                 # 速度指令が途絶えた
    gate.on_cmd(0.8, 0.2, 0.0)
    assert gate.output(1.5)[2] == STATE_ODOM_LOST            # オドメトリが途絶えた
    gate.on_odom(1.5)
    gate.on_camera(1.5)
    gate.on_cmd(1.5, 0.2, 0.0)
    gate.stop("test")
    assert gate.output(1.6) == (0.0, 0.0, STATE_STOPPED)     # 停止は解除するまで続く
    gate.on_cmd(1.7, 0.2, 0.0)
    assert gate.output(1.8)[2] == STATE_STOPPED
    gate.reset()
    assert gate.output(1.9)[2] == STATE_IDLE                 # 解除しても、古い速度指令では動き出さない
    gate.on_cmd(2.0, 0.2, 0.0)
    assert gate.output(2.1) == (0.2, 0.0, STATE_OK)


# -- 設定 -------------------------------------------------------------------
def test_bundled_config_loads_and_env_overrides():
    cfg = config_mod.load_config(os.path.join(PKG, "config", "task.yaml"), env={})
    assert cfg.vlm.base_url.endswith("/v1") and cfg.web.port == 8080 and len(cfg.instructions) == 3
    cfg = config_mod.load_config(None, overrides={"web": {"port": 9000}},
                                 env={"VLM_BASE_URL": "http://gpu:8000/v1", "VLM_MODEL": "m"})
    assert (cfg.web.port, cfg.vlm.base_url, cfg.vlm.model) == (9000, "http://gpu:8000/v1", "m")


def test_config_typos_are_errors(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text(json.dumps({"skills": {"max_liner": 0.3}}), encoding="utf-8")
    with pytest.raises(config_mod.ConfigError):
        config_mod.load_config(str(path), env={})
    path.write_text(json.dumps({"skills": {"max_linear": "fast"}}), encoding="utf-8")
    with pytest.raises(config_mod.ConfigError):
        config_mod.load_config(str(path), env={})


# -- プロキシ ---------------------------------------------------------------
def test_local_requests_bypass_proxy(monkeypatch):
    import threading
    from http.server import BaseHTTPRequestHandler, HTTPServer

    from elevator_task.core.http_util import is_local, urlopen

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"ok")

    server = HTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    # 存在しないプロキシを設定しても、127.0.0.1 宛ては直接つながる
    for name in ("http_proxy", "HTTP_PROXY"):
        monkeypatch.setenv(name, "http://127.0.0.1:9")
    monkeypatch.delenv("no_proxy", raising=False)
    monkeypatch.delenv("NO_PROXY", raising=False)
    try:
        with urlopen(f"http://127.0.0.1:{server.server_address[1]}/", timeout=5) as resp:
            assert resp.read() == b"ok"
    finally:
        server.shutdown()
    assert is_local("http://localhost:8000/v1") and is_local("http://[::1]:8000") and is_local("http://127.0.0.2/")
    assert not is_local("http://192.168.0.10:8000/v1") and not is_local("https://huggingface.co")
