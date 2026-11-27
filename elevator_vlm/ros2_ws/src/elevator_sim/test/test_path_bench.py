"""道のベンチの部品（ROS に依存しない部分）の試験: 映像のフレームの読み込みと、正解からのモック VLM の答え."""
import os

import pytest

from elevator_task.core.geometry import point_in_polygon
from elevator_sim.frame_labels import FrameLabels, aim_point, frame_number, load_frames

HERE = os.path.dirname(os.path.abspath(__file__))
ATC = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(HERE))))),
                   "atc_frames")


def test_frame_number():
    assert frame_number("frame_00243.jpg") == 243
    assert frame_number("x.jpg") is None


def test_aim_point_is_on_the_path_and_off_obstacles():
    label = {"walkable": [[200, 1000], [400, 500], [600, 500], [800, 1000]],
             "avoid": [[[450, 600], [550, 600], [550, 750], [450, 750]]]}
    x, y = aim_point(label)
    assert point_in_polygon(label["walkable"], x, y)
    assert not point_in_polygon(label["avoid"][0], x, y)
    assert 500 < y < 800                         # 遠すぎず近すぎない


def test_labels_answers():
    data = {"frames": {
        "frame_00010.jpg": {"walkable": [[0, 1000], [0, 500], [1000, 500], [1000, 1000]]},
        "frame_00020.jpg": {"walkable": [[0, 1000], [0, 600], [300, 600], [300, 1000]]}},
        "statements": [{"text": "Doors ahead.", "yes": ["frame_00015.jpg", "frame_00018.jpg"], "no": ["frame_00010.jpg"]}]}
    labels = FrameLabels(data)
    near10 = labels.path_point(12)
    near20 = labels.path_point(19)
    assert near10 == labels.points[10] and near20 == labels.points[20] and near20[0] < 300
    assert labels.path_point(None) is None
    assert [labels.statement("Doors ahead.", n) for n in (14, 15, 17, 18, 19)] == [False, True, True, True, False]
    assert not labels.statement("Something else.", 16)


def test_load_frames_range_and_size(tmp_path):
    from PIL import Image

    for n in (1, 2, 3, 10):
        Image.new("RGB", (1920, 1080), (n, n, n)).save(tmp_path / f"frame_{n:05d}.jpg")
    frames = load_frames(str(tmp_path), 2, 3, 960)
    assert [f[1] for f in frames] == [2, 3] and (frames[0][3], frames[0][4]) == (960, 540)
    assert [f[1] for f in load_frames(str(tmp_path), 1, 0, 0)] == [1, 2, 3, 10]


@pytest.mark.skipif(not os.path.isfile(os.path.join(ATC, "labels.json")), reason="atc_frames がない（イメージの中など）")
def test_atc_labels_give_answers_on_the_path():
    import json

    with open(os.path.join(ATC, "labels.json"), encoding="utf-8") as f:
        data = json.load(f)
    labels = FrameLabels(data)
    assert len(labels.points) == len(data["frames"])
    for name, label in data["frames"].items():
        x, y = labels.points[frame_number(name)]
        assert point_in_polygon(label["walkable"], x, y), name
        assert not any(point_in_polygon(a, x, y) for a in label.get("avoid", [])), name
    # ベンチの通し試験（225 枚目から走り、エレベーターの扉の前で止まる）で使う文
    doors = "Closed elevator doors are directly in front of the robot."
    assert not labels.statement(doors, 242) and labels.statement(doors, 243) and labels.statement(doors, 249)


def test_task_configs_load():
    """シミュレーション・ベンチ・実機の実行系の設定が、知らない項目なしで読める."""
    from elevator_task.core.config import load_config

    sim_config = os.path.join(os.path.dirname(HERE), "config")
    paths = [os.path.join(sim_config, n) for n in sorted(os.listdir(sim_config)) if n.startswith("task_")]
    robot = os.path.join(os.path.dirname(ATC), "jetson", "robot", "task_robot.yaml")
    if os.path.isfile(robot):
        paths.append(robot)
    for path in paths:
        cfg = load_config(path, env={})
        assert cfg.vlm.base_url.endswith("/v1"), path
    bench = load_config(os.path.join(sim_config, "task_bench_path.yaml"), env={})
    assert not bench.safety.publish_when_idle and bench.skills.follow_path.step_m == 1.0
    if os.path.isfile(robot):
        prod = load_config(robot, env={})
        assert not prod.safety.publish_when_idle and prod.drive.require_arm
        assert prod.drive.command_topic == "/vlm_drive/command" and prod.camera.image_topic == "/camera/compressed"
