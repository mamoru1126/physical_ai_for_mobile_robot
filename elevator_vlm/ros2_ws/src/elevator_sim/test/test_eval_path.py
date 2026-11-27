"""道に沿って走る机上検証（scripts/inside/eval_path.py）と、ATC のフレームの正解（atc_frames/labels.json）の試験.

本物の VLM の代わりにモック VLM（HTTP）を使い、スクリプトが最後まで通って結果を保存することを確かめる.
正解のファイルは、形（多角形が画像の中にあるか、画像があるか）を確かめる.
"""
import json
import os
import sys
import threading

import pytest

from elevator_sim.mock_vlm import MockConfig, MockVlm, make_server
from elevator_sim.world_params import WorldParams

HERE = os.path.dirname(os.path.abspath(__file__))
VLM_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(HERE))))   # elevator_vlm
REPO = os.path.dirname(VLM_ROOT)
ATC = os.path.join(REPO, "atc_frames")


def load_eval():
    import importlib.util

    spec = importlib.util.spec_from_file_location("eval_path", os.path.join(VLM_ROOT, "scripts", "inside", "eval_path.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_point_judgement():
    ev = load_eval()
    label = {"walkable": [[0, 1000], [0, 500], [1000, 500], [1000, 1000]],
             "avoid": [[[400, 600], [600, 600], [600, 800], [400, 800]]]}
    assert ev.judge_point(label, 200, 900) == ("on_path", 100.0)          # 下端から 100
    verdict, margin = ev.judge_point(label, 500, 700)
    assert verdict == "on_obstacle" and margin < 0
    verdict, margin = ev.judge_point(label, 500, 300)
    assert verdict == "off_path" and margin == pytest.approx(-200.0)


@pytest.mark.skipif(not os.path.isfile(os.path.join(ATC, "labels.json")), reason="atc_frames がない（イメージの中など）")
def test_atc_labels_are_consistent():
    with open(os.path.join(ATC, "labels.json"), encoding="utf-8") as f:
        labels = json.load(f)
    assert len(labels["frames"]) >= 30
    for name, item in labels["frames"].items():
        assert os.path.isfile(os.path.join(ATC, name)), name
        for poly in [item["walkable"]] + item.get("avoid", []):
            assert len(poly) >= 3, name
            assert all(0 <= x <= 1000 and 0 <= y <= 1000 for x, y in poly), name
    for st in labels["statements"]:
        assert st["text"].endswith(".") and st["yes"] and st["no"]
        for name in st["yes"] + st["no"]:
            assert os.path.isfile(os.path.join(ATC, name)), name
        assert not set(st["yes"]) & set(st["no"])


def test_eval_script_runs_against_a_mock(tmp_path, monkeypatch):
    # 小さな正解の組（3 枚）を作り、モック VLM（道の点はいつも画面の中央下）で通す
    from PIL import Image

    frames = tmp_path / "frames"
    frames.mkdir()
    for i in range(3):
        Image.new("RGB", (1920, 1080), (90 + i, 90, 90)).save(frames / f"f{i}.jpg")
    labels = {"path": "the walkway", "frames": {
        "f0.jpg": {"scene": "道の上", "walkable": [[0, 1000], [0, 500], [1000, 500], [1000, 1000]]},
        "f1.jpg": {"scene": "人の上", "walkable": [[0, 1000], [0, 500], [1000, 500], [1000, 1000]],
                   "avoid": [[[400, 700], [600, 700], [600, 900], [400, 900]]]},
        "f2.jpg": {"scene": "道の外", "walkable": [[0, 1000], [0, 900], [300, 900], [300, 1000]]}},
        "statements": [{"text": "A thing is ahead.", "yes": ["f0.jpg"], "no": ["f1.jpg", "f2.jpg"]}]}
    (frames / "labels.json").write_text(json.dumps(labels), encoding="utf-8")
    mock = MockVlm(WorldParams(), MockConfig(latency_s=0.0), lambda: None, lambda: None,
                   path_fn=lambda: (500.0, 800.0), statement_fn=lambda st: True)
    server = make_server(mock, "127.0.0.1", 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        monkeypatch.setenv("VLM_BASE_URL", f"http://127.0.0.1:{server.server_address[1]}/v1")
        monkeypatch.setenv("VLM_MODEL", "mock-vlm")
        out = tmp_path / "runs"
        ev = load_eval()
        monkeypatch.setattr(sys, "argv", ["eval_path.py", "--frames", str(frames), "--out", str(out),
                                          "--config", os.path.join(VLM_ROOT, "ros2_ws", "src", "elevator_task",
                                                                   "config", "task.yaml")])
        assert ev.main() == 0
        run = next(out.iterdir())
        result = json.loads((run / "results.json").read_text(encoding="utf-8"))
        verdicts = {r["frame"]: r["verdict"] for r in result["points"]}
        assert verdicts == {"f0.jpg": "on_path", "f1.jpg": "on_obstacle", "f2.jpg": "off_path"}
        assert [r["answer"] for r in result["checks"]] == ["yes", "yes", "yes"]
        assert len(result["plans"]) == 3 and all(r["ok"] for r in result["plans"]), result["plans"]
        report = (run / "report.md").read_text(encoding="utf-8")
        assert "道の上: **1 / 3 枚" in report and "1 / 1" in report and "0 / 2" in report
        assert sorted(os.listdir(run / "overlays")) == ["f0.jpg", "f1.jpg", "f2.jpg"]
    finally:
        server.shutdown()
        server.server_close()
