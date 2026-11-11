#!/usr/bin/env python3
"""VLM サーバとのつながりを確かめる. 実行系と同じコード（判定、指差し、計画）で、合成した画像に問いかける.

確かめること:
  1. yes / no の判定が、構造化出力で正しく返るか. 応答時間
  2. 指差しの座標の約束（Qwen 系は 0〜1000 の相対座標）が設定と合っているか
  3. 3つの指示から、検査に通る計画が作れるか
  4. 机上評価: シミュレーションのカメラ画像（samples/、正解つき）で、今の既定の言い回しでの
     扉の判定と指差しがどれだけ合うか（ほかの言い回しとの比較は scripts/vlm_phrases.sh）

1〜3 は設定の確認で、NG があれば終了コードが 1 になる. 4 は VLM の実力の確認で、終了コードには影響しない.
"""
import json
import os
import sys

ROOT = os.environ.get("ELEVATOR_VLM_ROOT", "/workspace")
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "elevator_task"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "elevator_sim"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PIL import Image, ImageDraw  # noqa: E402

from elevator_task.core.config import load_config  # noqa: E402
from elevator_task.core.events import EventLog  # noqa: E402
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


def desk_eval(cfg, vlm: VlmClient) -> None:
    """正解つきのシミュレーション画像で、今の既定の言い回し（samples/phrases.json の先頭）を評価する."""
    import eval_phrases

    samples = eval_phrases.load_samples(SAMPLES, [], 3)
    phrases_path = os.path.join(SAMPLES, "phrases.json")
    if not samples or not os.path.isfile(phrases_path):
        print("\n[4] 机上評価: samples/ に正解つきの画像がないので省略します")
        return
    with open(phrases_path, encoding="utf-8") as f:
        phrases = json.load(f)
    checks = [s for s in samples if s["kind"] == "check"]
    points = [s for s in samples if s["kind"] == "point"]
    frames = {s["path"]: eval_phrases.frame_of(s["path"]) for s in samples}
    targets = {place: list((phrases.get("targets") or {}).get(place, []))[:1]
               for place in eval_phrases.TARGET_PLACES}
    cond = eval_phrases.eval_conditions(cfg, vlm, list(phrases.get("conditions", []))[:1],
                                        [cfg.perception.check_prompt], checks, frames, 4)
    targ = eval_phrases.eval_targets(cfg, vlm, targets, points, frames, 4)
    print("\n[4] 机上評価（今の既定の言い回し. 正解はシミュレータの真値. ほかの候補との比較は scripts/vlm_phrases.sh）")
    print(eval_phrases.report(cfg.vlm.model, checks, points, cond, targ))


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

    desk_eval(cfg, vlm)

    print("\n設定の確認（1〜3）: " + ("すべて OK" if failures == 0 else f"{failures} 件の NG"))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
