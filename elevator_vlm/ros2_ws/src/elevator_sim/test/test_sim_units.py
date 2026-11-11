"""シミュレーション側の単体テスト（エレベーターの状態機械、世界の生成、モック VLM の規則、設定の整合）."""
import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml
from fake_world import FakeWorld

from elevator_sim import world_gen
from elevator_sim.elevator_logic import CLOSING, IDLE, MOVING, OPEN, OPENING, ElevatorConfig, ElevatorLogic, lift_velocity
from elevator_sim.mock_vlm import plan_from_instructions
from elevator_sim.referee import Referee
from elevator_sim.world_params import (RobotParams, Truth, WorldParams, door_fully_open, door_label,
                                       door_ratio_at_robot, view_state)
from elevator_task.core.config import load_config
from elevator_task.core.plan import parse_plan

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# -- エレベーターの状態機械 ---------------------------------------------------
def _phases(world: FakeWorld, seconds: float):
    seen = []
    for _ in range(int(seconds / world.DT)):
        world.step()
        if not seen or seen[-1][0] != world.logic.phase:
            seen.append((world.logic.phase, round(world.t, 2), world.logic.state()["floor"]))
    return seen


def test_elevator_normal_sequence_skips_intermediate_floor():
    world = FakeWorld(ElevatorConfig(board_floor=1, dest_floor=3, call_delay_s=3.0, door_hold_s=5.0))
    world.x = -3.0                                   # ロボットは離れた所にいる（出入口をふさがない）
    world.sleep(1.0)
    assert world.logic.call(world.t)[0]
    assert not world.logic.call(world.t)[0]          # 動作中の呼びは受けない
    seen = _phases(world, 60.0)
    assert [p for p, _, _ in seen] == [IDLE, OPENING, OPEN, CLOSING, IDLE, MOVING, OPENING, OPEN, CLOSING, IDLE]
    t = {i: seen[i][1] for i in range(len(seen))}
    assert t[1] == pytest.approx(4.0, abs=0.1)                       # 呼びの3秒後に開き始める
    assert t[2] - t[1] == pytest.approx(1.8, abs=0.15)               # 戸開 0.45 m ÷ 0.25 m/s
    assert t[3] - t[2] == pytest.approx(5.0, abs=0.1)                # 全開で保持
    assert seen[6][2] == 3 and world.lift == pytest.approx(6.0, abs=0.004)
    assert world.car_door == pytest.approx(0.0, abs=1e-6) and world.logic.door_cycles == 2
    assert all(v == pytest.approx(0.0, abs=1e-6) for v in world.landing.values())
    # 2階の乗場の扉は一度も動かない（指定階以外では止まらない）
    assert world.landing[2] == 0.0


def test_lift_profile_is_smooth_and_accurate():
    wp = WorldParams()
    pos, v, t, dt = 0.0, 0.0, 0.0, 0.02
    max_v = max_a = 0.0
    while t < 30.0:
        nv = lift_velocity(6.0 - pos, v, dt, wp.lift_speed, wp.lift_accel, 4.0)
        max_a = max(max_a, abs(nv - v) / dt)
        v = nv
        max_v = max(max_v, abs(v))
        pos += v * dt
        t += dt
        if abs(6.0 - pos) < 0.002 and abs(v) < 0.01:
            break
    assert t < 12.0 and pos == pytest.approx(6.0, abs=0.003) and pos <= 6.003       # 行き過ぎない
    assert max_v <= wp.lift_speed + 1e-9 and max_a <= wp.lift_accel + 1e-6          # 急な加減速がない


def test_door_reopens_when_doorway_is_blocked_and_reset_returns_home():
    world = FakeWorld(ElevatorConfig(door_hold_s=2.0, call_delay_s=0.5))
    world.x = -0.2                                   # ロボットが出入口にいる
    world.logic.call(world.t)
    world.sleep(12.0)
    assert world.logic.phase == OPEN                 # センサが働いて閉まらない
    world.x = -3.0
    world.sleep(1.0)
    assert world.logic.phase == OPEN                 # 物がなくなっても、少し待ってから閉める
    world.sleep(2.0)
    assert world.logic.phase in (CLOSING, IDLE, MOVING)
    world.sleep(6.0)
    assert world.logic.phase == MOVING
    world.logic.reset(world.t)                       # 昇降中にリセット → 行き先を付け替えて 1 階へ戻る
    world.sleep(20.0)
    assert world.logic.phase == IDLE and world.lift == pytest.approx(0.0, abs=0.004)
    assert world.logic.stops == [] and world.logic.return_floor is None
    assert world.logic.call(world.t)[0]              # 戻ったら、また呼べる


def test_hold_open_keeps_the_door_open():
    world = FakeWorld(ElevatorConfig(door_hold_s=1.0, call_delay_s=0.5))
    world.x = -3.0
    world.logic.call(world.t)
    world.logic.set_hold_open(True)
    world.sleep(15.0)
    assert world.logic.phase == OPEN
    world.logic.set_hold_open(False)
    world.sleep(4.0)
    assert world.logic.phase != OPEN


def test_invalid_floor_is_rejected():
    with pytest.raises(ValueError):
        ElevatorLogic(WorldParams(), ElevatorConfig(dest_floor=9))


def test_door_fully_open_truth():
    wp = WorldParams()
    travel = wp.door_travel
    hall = dict(robot_x=-1.0, robot_y=0.0, robot_yaw=0.0)
    assert door_fully_open(wp, Truth(robot_z=0.0, lift_pos=0.0, car_door=travel, landing={1: travel}, **hall))
    assert not door_fully_open(wp, Truth(robot_z=0.0, lift_pos=0.0, car_door=0.3, landing={1: 0.3}, **hall))    # 開く途中
    assert not door_fully_open(wp, Truth(robot_z=0.0, lift_pos=6.0, car_door=travel, landing={3: travel}, **hall))  # 別の階
    in_car = dict(robot_x=wp.car_center_x, robot_y=0.0, robot_yaw=math.pi)
    assert door_fully_open(wp, Truth(robot_z=6.0, lift_pos=6.0, car_door=travel, landing={3: travel}, **in_car))
    assert not door_fully_open(wp, Truth(robot_z=3.3, lift_pos=3.3, car_door=0.0, landing={}, **in_car))        # 昇降中


# -- モック VLM の計画の規則 ---------------------------------------------------
def test_mock_planner_rules():
    plan = parse_plan(plan_from_instructions(["扉が開いたら 1.5 m 前進して", "90度左に旋回して", "5秒待って"]))
    assert plan.steps[0].do.to_dict() == {"skill": "move_straight", "distance_m": 1.5}
    assert plan.steps[0].when.type == "when_true"
    assert plan.steps[1].do.to_dict() == {"skill": "rotate", "angle_deg": 90.0} and plan.steps[1].when.type == "now"
    assert plan.steps[2].do.skill == "wait"


# -- 世界の生成 ---------------------------------------------------------------
def test_committed_files_match_the_generator():
    wp, rp = WorldParams(), RobotParams()
    expected = {
        os.path.join("worlds", "elevator.sdf"): world_gen.world_sdf(wp),
        os.path.join("models", wp.robot_model, "model.sdf"): world_gen.robot_sdf(wp, rp),
        os.path.join("config", "bridge.yaml"): world_gen.bridge_yaml(wp),
    }
    for rel, text in expected.items():
        with open(os.path.join(PKG, rel), encoding="utf-8") as f:
            assert f.read() == text, f"{rel} が古い. python3 -m elevator_sim.world_gen --no-textures で作り直すこと"


def test_world_structure():
    wp = WorldParams()
    root = ET.parse(os.path.join(PKG, "worlds", "elevator.sdf")).getroot()
    model = next(m for m in root.iter("model") if m.get("name") == wp.building_model)
    links = {link.get("name") for link in model.findall("link")}
    joints = {j.get("name"): j for j in model.findall("joint")}
    assert {"base", "car", "car_door_left", "car_door_right"} <= links
    for floor in wp.floors():
        assert {f"landing_door_{floor}_left", f"landing_door_{floor}_right"} <= links
        assert set(wp.landing_door_joints(floor)) <= set(joints)
    assert joints["fix_base"].find("parent").text == "world"
    lift = joints[wp.lift_joint]
    assert float(lift.find("axis/limit/upper").text) >= wp.lift_max
    # 速度指令で動かす関節には、有限の力の上限が要る（無いと関節の可動範囲が守られない）
    for name, joint in joints.items():
        if joint.get("type") == "prismatic":
            assert math.isfinite(float(joint.find("axis/limit/effort").text)), name
    # 扉の2枚は逆向きに動く
    axes = [joints[j].find("axis/xyz").text for j in wp.car_door_joints]
    assert axes == ["0 1 0", "0 -1 0"]
    plugin_joints = [p.find("joint_name").text for p in model.findall("plugin") if p.find("joint_name") is not None]
    assert len(plugin_joints) == 1 + 2 + 2 * wp.num_floors and len(set(plugin_joints)) == len(plugin_joints)
    # テクスチャはすべて存在する
    tex_dir = os.path.join(PKG, "models", world_gen.TEX_MODEL, "materials", "textures")
    for elem in root.iter("albedo_map"):
        assert os.path.isfile(os.path.join(tex_dir, os.path.basename(elem.text))), elem.text
    # リンクと衝突・見た目の名前に重複がない
    for link in model.findall("link"):
        names = [e.get("name") for e in list(link.findall("collision")) + list(link.findall("visual"))]
        assert len(names) == len(set(names)), link.get("name")


def test_robot_fits_through_the_door_and_into_the_car():
    wp, rp = WorldParams(), RobotParams()
    assert 2 * rp.half_width < wp.door_width - 0.3            # 出入口で左右に 15 cm 以上の余裕
    assert rp.base_length < wp.car_depth - 0.5                 # かごの中央で前後に余裕
    assert wp.car_center_x == pytest.approx(0.775) and wp.start_distance + wp.car_center_x == pytest.approx(1.775)


def test_camera_settings_agree_between_robot_model_and_task_config():
    rp = RobotParams()
    cfg = load_config(os.path.join(PKG, "config", "task_sim.yaml"), env={})
    assert cfg.camera.height_m == pytest.approx(rp.camera_z)
    assert cfg.camera.pitch_deg == pytest.approx(rp.camera_pitch_deg)
    assert cfg.camera.x_offset_m == pytest.approx(rp.camera_x)
    assert math.radians(cfg.camera.hfov_deg) == pytest.approx(rp.camera_hfov, abs=0.01)
    assert rp.focal_px == pytest.approx(320.0, abs=0.5)
    # ロボットのモデル（SDF）にも同じ値が書かれている
    root = ET.parse(os.path.join(PKG, "models", WorldParams().robot_model, "model.sdf")).getroot()
    cam_link = next(link for link in root.iter("link") if link.get("name") == "camera_link")
    x, _y, z, _roll, pitch, _yaw = (float(v) for v in cam_link.find("pose").text.split())
    assert (x, z) == pytest.approx((rp.camera_x, rp.camera_z)) and math.degrees(pitch) == pytest.approx(15.0, abs=0.01)
    # 画面のボタンが呼ぶサービスは、エレベーターの制御ノードが提供するもの
    services = {b["service"] for b in cfg.web.buttons}
    assert services == {"/elevator/call", "/elevator/toggle_hold", "/sim/reset"}
    # 画面に映す固定カメラは、世界にあるもの
    wp = WorldParams()
    assert {c["topic"] for c in cfg.web.extra_cameras} <= {wp.observer_topic(n) for n in wp.observer_names()}


def test_bridge_config_covers_all_topics():
    wp = WorldParams()
    with open(os.path.join(PKG, "config", "bridge.yaml"), encoding="utf-8") as f:
        entries = {e["ros_topic_name"]: e for e in yaml.safe_load(f)}
    for topic in ["/clock", "/camera/image_raw", "/camera/camera_info", "/odom", "/ground_truth/odom",
                  wp.joint_state_topic] + [wp.observer_topic(n) for n in wp.observer_names()]:
        assert entries[topic]["direction"] == "GZ_TO_ROS"
    for topic in ["/cmd_vel", wp.lift_cmd_topic, wp.car_door_cmd_topic] + [wp.landing_door_cmd_topic(f) for f in wp.floors()]:
        assert entries[topic]["direction"] == "ROS_TO_GZ"


def test_mock_reports_missing_ground_truth_instead_of_answering_no():
    import threading

    from elevator_sim.mock_vlm import MockConfig, MockVlm, make_server
    from elevator_task.core.vlm_client import VlmClient, VlmConfig, VlmError

    missing = ["/ground_truth/odom"]
    mock = MockVlm(WorldParams(), MockConfig(latency_s=0.0), lambda: None, lambda: None, lambda: list(missing))
    server = make_server(mock, "127.0.0.1", 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    client = VlmClient(VlmConfig(base_url=f"http://127.0.0.1:{server.server_address[1]}/v1"))
    try:
        ok, info = client.ping()
        assert not ok and "/ground_truth/odom" in info
        msg = [{"role": "user", "content": [{"type": "text", "text": 'Statement: "door open"'}]}]
        with pytest.raises(VlmError, match="ground_truth"):
            client.chat(msg, max_tokens=8, schema_name="condition_check", schema={})
        missing.clear()
        assert client.ping()[0]
        assert "answer" in client.chat(msg, max_tokens=8, schema_name="condition_check", schema={}).text
    finally:
        server.shutdown()


# -- ロボットから見た様子の真値（画面の表示と、記録の分析に使う） ---------------------
def test_view_state_describes_what_the_robot_faces():
    p = WorldParams()
    full = p.door_travel
    closed = {1: 0.0, 2: 0.0, 3: 0.0}
    # 乗場の待機位置. 扉は全閉. 次に向かうのはかごの中央、敷居までは 1 m
    v = view_state(p, Truth(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, closed))
    assert (v["robot_in_car"], v["robot_floor"], v["door"], v["door_ratio"]) == (False, 1, "closed", 0.0)
    assert v["goal_ahead_m"] == pytest.approx(1.0 + p.car_center_x) and v["goal_left_m"] == pytest.approx(0.0)
    assert v["door_ahead_m"] == pytest.approx(1.0)
    # 扉が半分開いたところ. かごの扉と乗場の扉のうち、開きが小さい方で決まる
    half = Truth(-1.0, 0.0, 0.0, 0.0, 0.0, 0.5 * full, {1: 0.8 * full, 2: 0.0, 3: 0.0})
    assert door_ratio_at_robot(p, half) == pytest.approx(0.5) and view_state(p, half)["door"] == "partial"
    # かごが別の階にいる間は、乗場からは閉じた扉しか見えない
    away = Truth(-1.0, 0.0, 0.0, 0.0, 6.0, full, {1: 0.0, 2: 0.0, 3: full})
    assert view_state(p, away)["door"] == "closed"
    # かごの中で扉を向いている. 3 階で全開. 次に向かうのは乗場の 1 m 先
    v = view_state(p, Truth(0.6, 0.05, 6.0, math.pi, 6.0, full, {1: 0.0, 2: 0.0, 3: full}))
    assert (v["robot_in_car"], v["door"], v["door_ratio"]) == (True, "open", 1.0)
    assert v["goal_ahead_m"] == pytest.approx(1.6) and v["goal_left_m"] == pytest.approx(0.05)
    assert v["door_ahead_m"] == pytest.approx(0.6)
    # 昇降中は、かごの扉だけで決まる（全閉）
    assert view_state(p, Truth(0.6, 0.0, 3.3, math.pi, 3.3, 0.0, closed))["door"] == "closed"
    assert [door_label(r) for r in (0.0, 0.02, 0.03, 0.5, 0.94, 0.95, 1.0)] == [
        "closed", "closed", "partial", "partial", "partial", "open", "open"]


def test_referee_summary_in_words():
    p = WorldParams()
    full = p.door_travel
    ref = Referee(p, board_floor=1, dest_floor=3)
    assert ref.summary().startswith("未達成: かごに入った –")
    ref.update(1.0, Truth(0.7, 0.0, 0.0, 0.0, 0.0, full, {1: full, 2: 0.0, 3: 0.0}))       # かごに入った
    ref.update(2.0, Truth(0.7, 0.0, 0.0, math.pi, 0.0, 0.0, {1: 0.0, 2: 0.0, 3: 0.0}))     # 扉を向いた
    assert ref.summary() == "未達成: かごに入った ✓ → 扉の方を向いた ✓ → 行先階に着いた – → 行先階で降りた –"
    assert ref.result()["summary"] == ref.summary()
    ref.update(3.0, Truth(0.7, 0.0, 6.0, math.pi, 6.0, full, {1: 0.0, 2: 0.0, 3: full}))   # 3 階に着いた
    ref.update(4.0, Truth(-1.0, 0.0, 6.0, math.pi, 6.0, full, {1: 0.0, 2: 0.0, 3: full}))  # 降りた
    assert ref.summary() == "成功: 行先階で降りました"
    wrong = Referee(p, board_floor=1, dest_floor=3)
    wrong.update(1.0, Truth(0.7, 0.0, 0.0, 0.0, 0.0, full, {1: full, 2: 0.0, 3: 0.0}))
    wrong.update(2.0, Truth(-1.0, 0.0, 0.0, math.pi, 0.0, full, {1: full, 2: 0.0, 3: 0.0}))  # 乗った階で降りた
    assert wrong.summary() == "失敗: 行先階ではない階で降りました"
