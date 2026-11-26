"""ベンチ（Orin で、実機の代わりにダミーの世界をつないで通す）の部品の試験.

  * ダミーのカメラ: 真値に合う画像を samples/ から選ぶか
  * 点群: 壁と扉の点を、点群の座標系（cloud_link）で出すか
"""
import math
import os

import pytest

from elevator_sim.bench_cloud import CloudMount, world_points
from elevator_sim.elevator_logic import ElevatorConfig
from elevator_sim.pseudo_world import PseudoWorld
from elevator_sim.sample_camera import SampleCamera, classify
from elevator_sim.world_params import Truth, WorldParams

SAMPLES = os.environ.get("ELEVATOR_SAMPLES_DIR") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "samples")
WP = WorldParams()
OPEN = WP.door_travel


def truth(x=-1.0, y=0.0, yaw=0.0, floor=1, lift_floor=1, car_door=0.0, landing=None, lift=None, in_car=False):
    lift_pos = WP.floor_z(lift_floor) if lift is None else lift
    z = lift_pos if in_car else WP.floor_z(floor)
    land = {f: 0.0 for f in WP.floors()}
    land.update(landing or {})
    return Truth(x, y, z, yaw, lift_pos, car_door, land)


@pytest.fixture(scope="module")
def camera():
    if not os.path.isfile(os.path.join(SAMPLES, "labels.json")):
        pytest.skip(f"samples/ がありません: {SAMPLES}")
    return SampleCamera(SAMPLES)


def test_hall_closed_partial_open(camera):
    name, jpeg, view = camera.select(truth())
    assert view.place == "hall" and view.facing_door
    assert name.startswith("check_hall_f1_closed") and jpeg[:2] == b"\xff\xd8"
    name, _, _ = camera.select(truth(car_door=0.5 * OPEN, landing={1: 0.5 * OPEN}))
    assert name.startswith("check_hall_f1_partial4")
    name, _, _ = camera.select(truth(car_door=OPEN, landing={1: OPEN}))
    assert "hall_f1_open" in name or name.startswith("point_hall_f1")


def test_closing_door_is_not_shown_as_open(camera):
    # 9 割まで閉まり始めた扉に、全開の画像を出さない
    name, _, view = camera.select(truth(car_door=0.9 * OPEN, landing={1: 0.9 * OPEN}))
    assert view.door_ratio == pytest.approx(0.9)
    assert "partial" in name


def test_in_car_facing_door(camera):
    center = WP.car_center_x
    name, _, view = camera.select(truth(x=center, yaw=math.pi, lift_floor=1, car_door=OPEN, landing={1: OPEN},
                                        in_car=True))
    assert view.place == "car" and view.floor == 1
    assert name.startswith("check_car_f1_open")
    # 昇降中（階の間）は扉が閉じている
    name, _, view = camera.select(truth(x=center, yaw=math.pi, lift=4.0, in_car=True))
    assert view.floor == 0 and name.startswith("check_car_f0_closed")
    # 3 階で開きかけ
    name, _, _ = camera.select(truth(x=center, yaw=math.pi, lift_floor=3, car_door=0.57 * OPEN,
                                     landing={3: 0.57 * OPEN}, in_car=True))
    assert name.startswith("check_car_f3_partial5")


def test_closed_door_at_floor_without_closed_sample(camera):
    # 3 階に着いた直後（扉はまだ閉じている）. 3 階の「閉」の画像はないので、別の階の「閉」の画像を使う
    name, _, view = camera.select(truth(x=WP.car_center_x, yaw=math.pi, lift_floor=3, in_car=True))
    assert view.floor == 3 and view.door_ratio == 0.0
    assert "closed" in name


def test_in_car_near_door(camera):
    near_x = 0.55
    name, _, view = camera.select(truth(x=near_x, yaw=math.pi, lift_floor=3, car_door=OPEN, landing={3: OPEN},
                                        in_car=True))
    assert view.place == "car_near"
    assert "car_near_f3" in name


def test_views_without_sample_use_placeholder(camera):
    # かごに入った直後（奥の壁を向いている）
    name, jpeg, view = camera.select(truth(x=WP.car_center_x, yaw=0.0, in_car=True))
    assert name == "placeholder" and not view.facing_door and jpeg[:2] == b"\xff\xd8"
    # 降りた後の 3 階の乗場（かごに背を向けている）
    name, _, view = camera.select(truth(x=-1.0, yaw=math.pi, floor=3, lift_floor=3))
    assert name == "placeholder" and view.place == "hall" and view.floor == 3


def test_classify_floor_of_hall():
    view = classify(WP, truth(x=-1.0, yaw=0.0, floor=3, lift_floor=3))
    assert view.place == "hall" and view.floor == 3 and view.facing_door


def test_cloud_walls_and_door():
    world = PseudoWorld(ElevatorConfig())
    mount = CloudMount(x=0.1, y=0.0, z=0.4, yaw=0.0)
    pts = world_points(world.wp, world.truth(), world.rp, mount, max_range=5.0)
    assert len(pts) > 100
    # 扉が閉じている: 正面（ロボットの前、横のずれ 0.3 m 以内）に、敷居のあたり（前 0.9〜1.0 m）の点がある
    front = [p for p in pts if abs(p[1]) < 0.3 and 0.8 < p[0] + mount.x < 1.1]
    assert front, "閉じた扉の点がない"
    # 床（高さ 0.05 m より下）の点は出さない
    assert min(p[2] + mount.z for p in pts) > 0.05 - 1e-6


def test_cloud_door_open_shows_car():
    world = PseudoWorld(ElevatorConfig())
    world.car_door = OPEN
    world.landing[1] = OPEN
    mount = CloudMount(x=0.1, y=0.0, z=0.4, yaw=0.0)
    pts = world_points(world.wp, world.truth(), world.rp, mount, max_range=5.0)
    # 扉が開いている: 正面の出入口のあたりには点がなく、かごの奥の壁（前 2.4 m 付近）に点がある
    doorway = [p for p in pts if abs(p[1]) < 0.3 and 0.8 < p[0] + mount.x < 1.1]
    back = [p for p in pts if abs(p[1]) < 0.3 and 2.3 < p[0] + mount.x < 2.6]
    assert not doorway
    assert back


def test_cloud_mount_transform():
    world = PseudoWorld(ElevatorConfig())
    mount = CloudMount(x=0.1, y=0.05, z=0.4, yaw=math.pi)   # 後ろ向きに付けた LiDAR
    pts = world_points(world.wp, world.truth(), world.rp, mount, max_range=5.0)
    # cloud_link の点を base_link に戻すと、閉じた扉は前にある
    back = [(-p[0] + mount.x, -p[1] + mount.y) for p in pts]
    assert any(abs(y) < 0.3 and 0.8 < x < 1.1 for x, y in back)
