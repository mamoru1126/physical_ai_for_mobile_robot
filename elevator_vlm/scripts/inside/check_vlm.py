#!/usr/bin/env python3
"""VLM サーバとのつながりを確かめる. 実行系と同じコード（判定、指差し、計画）で、合成した画像に問いかける.

確かめること:
  1. yes / no の判定が、構造化出力で正しく返るか. 応答時間
  2. 指差しの座標の約束（Qwen 系は 0〜1000 の相対座標）が設定と合っているか
  3. 3つの指示から、検査に通る計画が作れるか
  4. 机上評価: シミュレーションのカメラ画像（samples/、正解つき）で、扉の判定と指差しがどれだけ合うか

1〜3 は設定の確認で、NG があれば終了コードが 1 になる. 4 は VLM の実力の確認で、終了コードには影響しない.
"""
import io
import json
import math
import os
import sys

ROOT = os.environ.get("ELEVATOR_VLM_ROOT", "/workspace")
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "elevator_task"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "elevator_sim"))

from PIL import Image, ImageDraw  # noqa: E402

from elevator_task.core.config import load_config  # noqa: E402
from elevator_task.core.events import EventLog  # noqa: E402
from elevator_task.core.geometry import CameraModel, pixel_to_ground  # noqa: E402
from elevator_task.core.perception import Perception  # noqa: E402
from elevator_task.core.planner import make_plan  # noqa: E402
from elevator_task.core.robot_io import Frame, RobotIO, encode_jpeg  # noqa: E402
from elevator_task.core.vlm_client import VlmClient, VlmError  # noqa: E402

SAMPLES = os.path.join(ROOT, "samples")
W, H = 640, 480
SQUARE = (430, 300, 510, 380)          # 赤い四角（中心は 470, 340）


class StillImage(RobotIO):
    def __init__(self) -> None:
        img = Image.new("RGB", (W, H), (150, 150, 150))
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, W, 180], fill=(205, 205, 200))        # 壁
        draw.rectangle(SQUARE, fill=(220, 30, 30))
        self.frame = Frame(0.0, W, H, img.tobytes())

    def now(self) -> float:
        return 0.0

    def get_frame(self):
        return self.frame


def desk_eval(perception: Perception, cfg) -> None:
    """正解つきのシミュレーション画像で、扉の判定と指差しを評価する."""
    labels_path = os.path.join(SAMPLES, "labels.json")
    if not os.path.isfile(labels_path):
        print("\n[4] 机上評価: samples/labels.json がないので省略します")
        return
    with open(labels_path, encoding="utf-8") as f:
        labels = json.load(f)
    print(f"\n[4] 机上評価（シミュレーションのカメラ画像 {len(labels)} 枚. 正解はシミュレータの真値）")
    cam = None
    correct = total = 0
    latencies = []
    for item in labels:
        with open(os.path.join(SAMPLES, item["file"]), "rb") as f:
            img = Image.open(io.BytesIO(f.read())).convert("RGB")
        frame = Frame(0.0, img.width, img.height, img.tobytes())
        if cam is None:
            mount = cfg.camera
            focal = img.width / (2.0 * math.tan(math.radians(mount.hfov_deg) / 2.0))
            cam = CameraModel(focal, focal, img.width / 2.0, img.height / 2.0, img.width, img.height,
                              mount.height_m, math.radians(mount.pitch_deg), mount.x_offset_m)
        if item["kind"] == "check":
            res = perception.check(item["condition"], frame=frame)
            answer = "?" if res.answer is None else ("yes" if res.answer else "no")
            good = answer == item["expected"]
            total += 1
            correct += 1 if good else 0
            latencies.append(res.latency_s)
            print(f"  {'OK' if good else 'NG'}  {item['file']}: 正解 {item['expected']}、答え {answer}  {res.latency_s:.2f} 秒")
        else:
            res = perception.point(item["target"], frame=frame)
            if not res.ok:
                print(f"  NG  {item['file']}: 点を読めません（{res.error}）")
                continue
            latencies.append(res.latency_s)
            err_px = math.hypot(res.u - item["u"], res.v - item["v"])
            truth, got = pixel_to_ground(cam, item["u"], item["v"]), pixel_to_ground(cam, res.u, res.v)
            if truth is None or got is None:
                detail = "床に当たらない点です（地平線より上）"
            else:
                detail = (f"床の上で {math.hypot(got[0] - truth[0], got[1] - truth[1]) * 100:.0f} cm"
                          f"（前後 {(got[0] - truth[0]) * 100:+.0f} cm、左右 {(got[1] - truth[1]) * 100:+.0f} cm）")
            print(f"  --  {item['file']}:「{item['target']}」 正解 ({item['u']:.0f}, {item['v']:.0f})、"
                  f"答え ({res.u:.0f}, {res.v:.0f})、ずれ {err_px:.0f} 画素 = {detail}  {res.latency_s:.2f} 秒")
    if total:
        print(f"  判定の正解: {correct} / {total}")
    if latencies:
        print(f"  応答時間: 平均 {sum(latencies) / len(latencies):.2f} 秒、最大 {max(latencies):.2f} 秒")


def main() -> int:
    config = os.path.join(ROOT, "ros2_ws", "src", "elevator_sim", "config", "task_sim.yaml")
    cfg = load_config(config)
    print(f"VLM サーバ: {cfg.vlm.base_url}  モデル名: {cfg.vlm.model}")
    vlm = VlmClient(cfg.vlm)
    ok, info = vlm.ping()
    if not ok:
        print(f"NG: 接続できません（{info}）. scripts/vlm_up.sh で起動してください")
        return 1
    print(f"接続 OK（{info}）")
    robot = StillImage()
    perception = Perception(robot, vlm, cfg.perception, EventLog(None))
    failures = 0

    print("\n[1] 判定（yes / no）")
    for statement, expected in (("There is a red square in the image.", True),
                                ("There is a blue circle in the image.", False)):
        for _ in range(2):                             # 1回目はウォームアップを兼ねる
            res = perception.check(statement)
        good = res.answer is expected
        failures += 0 if good else 1
        print(f"  {'OK' if good else 'NG'}  「{statement}」→ {res.text!r}  {res.latency_s:.2f} 秒"
              + (f"  （{res.error}）" if res.error else ""))

    print("\n[2] 指差し（赤い四角の中心は u=470, v=340）")
    res = perception.point("the red square")
    if not res.ok:
        failures += 1
        print(f"  NG  点を読めません: {res.error}  返答: {res.text!r}")
    else:
        err = ((res.u - 470) ** 2 + (res.v - 340) ** 2) ** 0.5
        good = err < 45.0
        failures += 0 if good else 1
        print(f"  {'OK' if good else 'NG'}  返答 {res.text!r} → 画素 (u={res.u:.0f}, v={res.v:.0f})、ずれ {err:.0f} 画素、"
              f"{res.latency_s:.2f} 秒")
        if not good:
            print("      座標の約束が違う可能性があります. 設定の perception.point_scale / point_order を確かめてください")

    print("\n[3] 計画（3つの指示）")
    try:
        result = make_plan(vlm, cfg.instructions, encode_jpeg(robot.frame), cfg.limits, thinking=cfg.planner_thinking)
    except VlmError as e:
        result = None
        failures += 1
        print(f"  NG  {e}")
    if result is not None:
        if result.plan is None:
            failures += 1
            print(f"  NG  検査に通る計画になりません: {result.error}\n      返答: {result.text[:400]!r}")
        else:
            print(f"  OK  {result.latency_s:.2f} 秒、{result.attempts} 回目で通過")
            print(result.plan.to_json())

    desk_eval(perception, cfg)

    print("\n設定の確認（1〜3）: " + ("すべて OK" if failures == 0 else f"{failures} 件の NG"))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
