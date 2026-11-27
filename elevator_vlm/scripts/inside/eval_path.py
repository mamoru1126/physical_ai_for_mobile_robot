#!/usr/bin/env python3
"""道に沿って走る（follow_path）の机上検証. 実写の画像（ATC のコースの動画のフレーム）と正解で、
本物の VLM が実行系と同じ問いかけにどう答えるかを測る（ロボットは動かさない）.

測ること:
  1. 道の指差し: 実行系と同じプロンプト（perception.path_system）で「次に進む道の上の点」を指させ、
     正解の「走れる範囲」（多角形）の中に入るか、避けるもの（人、コーン、柱）に乗っていないか、
     範囲の端からどれだけ内側か、応答時間.
  2. 止まる条件の判定: 条件の文（エレベーターの扉が目の前、コーンでふさがれている、など）に yes / no で
     答えさせ、正解と比べる.
  3. 計画: 「道に沿って走って」などの指示が、follow_path の手順（距離、止まる条件）になるか.

結果は <out>/path_eval_<日時>/ に保存する（report.md、results.json、overlays/ に点を描いた画像）.

使い方（コンテナの中）:
  python3 scripts/inside/eval_path.py --frames /atc_frames
  python3 scripts/inside/eval_path.py --frames /atc_frames --only point --limit 5      # 指差しを 5 枚だけ

環境変数:
  ELEVATOR_VLM_ROOT  elevator_vlm フォルダの場所（既定 /workspace）
  VLM_BASE_URL / VLM_MODEL  VLM サーバ（既定は設定ファイルの値）
"""
from __future__ import annotations

import argparse
import io
import json
import os
import statistics
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.environ.get("ELEVATOR_VLM_ROOT", "/workspace")
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "elevator_task"))

from PIL import Image, ImageDraw  # noqa: E402

from elevator_task.core.config import load_config  # noqa: E402
from elevator_task.core.events import EventLog  # noqa: E402
from elevator_task.core.geometry import point_in_polygon, polygon_edge_distance  # noqa: E402
from elevator_task.core.perception import Perception  # noqa: E402
from elevator_task.core.plan import SKILL_FOLLOW_PATH  # noqa: E402
from elevator_task.core.planner import make_plan  # noqa: E402
from elevator_task.core.robot_io import Frame, RobotIO  # noqa: E402
from elevator_task.core.http_util import urlopen  # noqa: E402
from elevator_task.core.vlm_client import VlmClient, VlmError  # noqa: E402

PLAN_CASES = [
    # (指示, 期待する distance_m（None は設定の既定値）, 止まる条件があるか)
    ("道に沿って走って", None, False),
    ("道に沿って20m走って", 20.0, False),
    ("エレベーターの扉の前まで道に沿って走って", 0.0, True),
]


class NoRobot(RobotIO):
    """画像は毎回渡すので、ロボットは時計だけ."""

    def now(self) -> float:
        return time.time()


def model_name(base_url: str, fallback: str) -> str:
    """サーバが動かしているモデルの名前（vLLM の root. 取れなければ fallback）."""
    try:
        with urlopen(base_url.rstrip("/") + "/models", timeout=5) as resp:
            data = json.loads(resp.read().decode("utf-8"))["data"][0]
        return str(data.get("root") or data.get("id") or fallback)
    except Exception:  # noqa: BLE001  表示用
        return fallback


# -- 幾何（0〜1000 の相対座標） ---------------------------------------------------
def judge_point(label: Dict[str, Any], x: float, y: float) -> Tuple[str, float]:
    """(結果, 余裕). 結果は on_path / on_obstacle / off_path. 余裕は範囲の端からの距離（外なら負）."""
    walkable = label["walkable"]
    for poly in label.get("avoid", []):
        if point_in_polygon(poly, x, y):
            return "on_obstacle", -polygon_edge_distance(poly, x, y)
    d = polygon_edge_distance(walkable, x, y)
    if point_in_polygon(walkable, x, y):
        return "on_path", d
    return "off_path", -d


# -- 画像 ---------------------------------------------------------------------
def load_frame(path: str, width: int) -> Tuple[Frame, Image.Image]:
    img = Image.open(path).convert("RGB")
    if width > 0 and img.width > width:
        img = img.resize((width, round(img.height * width / img.width)), Image.BILINEAR)
    return Frame(time.time(), img.width, img.height, img.tobytes()), img


def overlay(img: Image.Image, label: Optional[Dict[str, Any]], x: Optional[float], y: Optional[float],
            verdict: str, title: str) -> bytes:
    w, h = img.size
    out = img.convert("RGBA")
    layer = Image.new("RGBA", out.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    if label:
        d.polygon([(px * w / 1000, py * h / 1000) for px, py in label["walkable"]],
                  fill=(0, 255, 0, 55), outline=(0, 255, 0, 255))
        for poly in label.get("avoid", []):
            d.polygon([(px * w / 1000, py * h / 1000) for px, py in poly], fill=(255, 0, 0, 70),
                      outline=(255, 0, 0, 255))
    out = Image.alpha_composite(out, layer).convert("RGB")
    d = ImageDraw.Draw(out)
    if x is not None and y is not None:
        u, v = x * w / 1000, y * h / 1000
        color = {"on_path": (40, 220, 80), "on_obstacle": (255, 140, 0)}.get(verdict, (255, 40, 40))
        r = max(8, w // 80)
        d.ellipse([u - r - 3, v - r - 3, u + r + 3, v + r + 3], outline=(255, 255, 255), width=3)
        d.ellipse([u - r, v - r, u + r, v + r], outline=color, width=4)
        d.line([u - r, v, u + r, v], fill=color, width=2)
        d.line([u, v - r, u, v + r], fill=color, width=2)
    d.rectangle([0, 0, w, 22], fill=(0, 0, 0))
    d.text((6, 5), title, fill=(255, 255, 255))
    buf = io.BytesIO()
    out.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def latency_text(values: List[float]) -> str:
    if not values:
        return "-"
    vals = sorted(values)
    p90 = vals[min(len(vals) - 1, int(round(0.9 * (len(vals) - 1))))]
    return f"中央値 {statistics.median(vals):.2f} 秒、90% {p90:.2f} 秒、最大 {vals[-1]:.2f} 秒（{len(vals)} 回）"


# -- 1. 道の指差し ----------------------------------------------------------------
def eval_points(perception: Perception, frames_dir: str, labels: Dict[str, Any], path_text: str, width: int,
                names: List[str], workers: int, out_dir: str) -> List[Dict[str, Any]]:
    os.makedirs(os.path.join(out_dir, "overlays"), exist_ok=True)

    def ask(name: str) -> Dict[str, Any]:
        label = labels["frames"][name]
        frame, img = load_frame(os.path.join(frames_dir, name), width)
        res = perception.point_path(path_text, frame=frame, purpose="path_eval")
        row: Dict[str, Any] = {"frame": name, "scene": label.get("scene", ""), "latency_s": round(res.latency_s, 3),
                               "text": res.text[:200]}
        if not res.ok:
            row.update(verdict="no_answer", error=res.error)
            x = y = None
        else:
            x, y = res.u / frame.width * 1000.0, res.v / frame.height * 1000.0
            verdict, margin = judge_point(label, x, y)
            row.update(verdict=verdict, x=round(x), y=round(y), margin=round(margin, 1))
        title = f"{name}  {row['verdict']}  " + (f"margin {row.get('margin')}" if "margin" in row else row.get("error", ""))
        with open(os.path.join(out_dir, "overlays", name), "wb") as f:
            f.write(overlay(img, label, x, y, row["verdict"], title))
        return row

    with ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
        rows = list(pool.map(ask, names))
    return rows


# -- 2. 止まる条件の判定 ---------------------------------------------------------------
def eval_statements(perception: Perception, frames_dir: str, labels: Dict[str, Any], width: int,
                    workers: int, limit: int) -> List[Dict[str, Any]]:
    jobs: List[Tuple[str, str, bool]] = []
    for item in labels.get("statements", []):
        yes = list(item.get("yes", []))
        no = list(item.get("no", []))
        if limit > 0:
            yes, no = yes[:limit], no[:limit]
        jobs += [(item["text"], name, True) for name in yes] + [(item["text"], name, False) for name in no]

    def ask(job: Tuple[str, str, bool]) -> Dict[str, Any]:
        text, name, expected = job
        path = os.path.join(frames_dir, name)
        if not os.path.isfile(path):
            return {"statement": text, "frame": name, "expected": expected, "answer": None, "error": "画像がない"}
        frame, _img = load_frame(path, width)
        res = perception.check(text, frame=frame, purpose="stop_eval")
        return {"statement": text, "frame": name, "expected": "yes" if expected else "no",
                "answer": None if res.answer is None else ("yes" if res.answer else "no"),
                "latency_s": round(res.latency_s, 3), "error": res.error}

    with ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
        return list(pool.map(ask, jobs))


# -- 3. 計画 ---------------------------------------------------------------------
def eval_plans(vlm: VlmClient, cfg) -> List[Dict[str, Any]]:
    out = []
    lines = [c[0] for c in PLAN_CASES]
    try:
        result = make_plan(vlm, lines, None, cfg.limits, thinking=cfg.planner_thinking)
    except VlmError as e:
        return [{"instruction": "（全体）", "ok": False, "detail": f"VLM のエラー: {e}"}]
    if result.plan is None:
        return [{"instruction": "（全体）", "ok": False, "detail": f"検査に通る計画になりません: {result.error}",
                 "text": result.text[:500]}]
    for (line, want_dist, want_stop), step in zip(PLAN_CASES, result.plan.steps):
        do = step.do
        want = cfg.limits.default_path_distance_m if want_dist is None else want_dist
        ok = do.skill == SKILL_FOLLOW_PATH and abs(do.distance_m - want) < 1e-6 and bool(do.stop_when) == want_stop
        out.append({"instruction": line, "ok": ok, "latency_s": round(result.latency_s, 2),
                    "detail": json.dumps(do.to_dict(), ensure_ascii=False),
                    "expected": f"follow_path、distance_m={want:g}、止まる条件{'あり' if want_stop else 'なし'}"})
    return out


# -- まとめ ---------------------------------------------------------------------
def summarize(model: str, points: List[Dict[str, Any]], checks: List[Dict[str, Any]],
              plans: List[Dict[str, Any]], args) -> str:
    lines = [f"# 道に沿って走る: 机上検証（{time.strftime('%Y-%m-%d %H:%M')}）", "",
             f"- モデル: {model}", f"- 画像: {args.frames}（幅 {args.width} 画素に縮めて問いかけ）",
             f"- 沿うもの（path）: {args.path}", f"- 同時に問いかける数: {args.workers}", ""]
    if points:
        n = len(points)
        count = {k: sum(1 for r in points if r["verdict"] == k) for k in ("on_path", "on_obstacle", "off_path", "no_answer")}
        margins = [r["margin"] for r in points if r["verdict"] == "on_path"]
        lines += ["## 1. 道の指差し", "",
                  f"- 道の上: **{count['on_path']} / {n} 枚（{100.0 * count['on_path'] / n:.0f}%）**",
                  f"- 避けるもの（人、コーン、柱）の上: {count['on_obstacle']} 枚、道の外: {count['off_path']} 枚、"
                  f"答えが読めない: {count['no_answer']} 枚",
                  f"- 道の端からの余裕（道の上のとき、0〜1000 の相対）: "
                  + (f"中央値 {statistics.median(margins):.0f}、最小 {min(margins):.0f}" if margins else "-"),
                  f"- 指した高さ（y、0〜1000）: 中央値 "
                  + (f"{statistics.median([r['y'] for r in points if 'y' in r]):.0f}"
                     if any("y" in r for r in points) else "-")
                  + "（大きいほどロボットの近く）",
                  f"- 応答時間: {latency_text([r['latency_s'] for r in points if r.get('latency_s')])}", "",
                  "| 画像 | 場面 | 結果 | 点 (x, y) | 余裕 |", "| --- | --- | --- | --- | --- |"]
        names = {"on_path": "道の上", "on_obstacle": "避けるものの上", "off_path": "道の外", "no_answer": "読めない"}
        for r in points:
            lines.append(f"| {r['frame']} | {r['scene']} | {names[r['verdict']]} | "
                         + (f"({r['x']}, {r['y']})" if "x" in r else "-") + f" | {r.get('margin', '-')} |")
        lines.append("")
    if checks:
        lines += ["## 2. 止まる条件の判定", "", "| 条件の文 | yes が正解 | no が正解 | 応答時間（中央値） |",
                  "| --- | --- | --- | --- |"]
        by: Dict[str, List[Dict[str, Any]]] = {}
        for r in checks:
            by.setdefault(r["statement"], []).append(r)
        for text, rows in by.items():
            yes = [r for r in rows if r["expected"] == "yes"]
            no = [r for r in rows if r["expected"] == "no"]
            lat = [r["latency_s"] for r in rows if r.get("latency_s")]
            lines.append(f"| {text} | {sum(1 for r in yes if r['answer'] == 'yes')} / {len(yes)} | "
                         f"{sum(1 for r in no if r['answer'] == 'no')} / {len(no)} | "
                         + (f"{statistics.median(lat):.2f} 秒" if lat else "-") + " |")
        wrong = [r for r in checks if r["answer"] != r["expected"]]
        if wrong:
            lines += ["", "間違えたもの: " + "、".join(f"{r['frame']}（{r['statement'][:30]}… 正解 {r['expected']}、"
                                                     f"答え {r['answer'] or '読めない'}）" for r in wrong)]
        lines.append("")
    if plans:
        lines += ["## 3. 計画", "", "| 指示 | 結果 | 期待 | 作られた手順 |", "| --- | --- | --- | --- |"]
        for r in plans:
            lines.append(f"| {r['instruction']} | {'OK' if r['ok'] else 'NG'} | {r.get('expected', '-')} | "
                         f"`{r['detail']}` |")
        lines.append("")
    lines += ["## 読み方", "",
              "- 道の指差しは、走行中にくり返し問いかけるもの. 1回の答えで進むのは、その方向へ最大 "
              f"{args.step_m:g} m（設定の skills.follow_path.step_m）. 何回か外しても、次の答えで向きが直る.",
              "- 道の外や避けるものの上を指す割合が高い場面は、本番でその区間を VLM に任せない"
              "（元のシステムで走る）か、沿うもの（path）の言い回しを変えて試す（--path）.",
              "- 止まる条件は、yes が 2 回続いたら止まる（skills.follow_path.stop_confirm_count）. "
              "no が正解の画像で yes と答える（早く止まる）より、yes が正解の画像で no と答える（止まらない）方が危ない.",
              "- 正解の範囲は目で見て付けたおおよそのもの. 点を描いた画像（overlays/）で確かめる."]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="道に沿って走る（follow_path）の机上検証（実写の画像と正解で）")
    parser.add_argument("--frames", default="/atc_frames", help="画像と labels.json のあるディレクトリ")
    parser.add_argument("--labels", default="", help="正解のファイル（既定 <frames>/labels.json）")
    parser.add_argument("--config", default=os.path.join(ROOT, "ros2_ws", "src", "elevator_task", "config", "task.yaml"),
                        help="実行系の設定（VLM と、指差しのプロンプト）")
    parser.add_argument("--path", default="", help="沿うものの言い回し（既定は labels.json の path）")
    parser.add_argument("--width", type=int, default=960, help="問いかける前に、この幅（画素）まで縮める（0 で縮めない）")
    parser.add_argument("--workers", type=int, default=1, help="同時に問いかける数（応答時間を測るなら 1）")
    parser.add_argument("--only", choices=["all", "point", "check", "plan"], default="all")
    parser.add_argument("--limit", type=int, default=0, help="枚数を減らして試す（0 なら全部）")
    parser.add_argument("--out", default="/runs", help="結果の保存先")
    args = parser.parse_args()

    labels_path = args.labels or os.path.join(args.frames, "labels.json")
    try:
        with open(labels_path, encoding="utf-8") as f:
            labels = json.load(f)
    except (OSError, ValueError) as e:
        print(f"NG: 正解のファイルを読めません: {labels_path}（{e}）")
        return 1
    args.path = args.path or labels.get("path", "the path ahead")
    cfg = load_config(args.config)
    args.step_m = cfg.skills.follow_path.step_m
    vlm = VlmClient(cfg.vlm)
    ok, info = vlm.ping()
    print(f"VLM サーバ: {cfg.vlm.base_url}  モデル名: {cfg.vlm.model}  {'接続 OK' if ok else 'NG: ' + info}")
    if not ok:
        print("VLM のサーバを先に起動してください（3090: scripts/vlm_up.sh、Orin: bash jetson/vlm_up.sh）")
        return 1
    model = model_name(cfg.vlm.base_url, info)
    out_dir = os.path.join(args.out, "path_eval_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    perception = Perception(NoRobot(), vlm, cfg.perception, EventLog(None))
    # 1 回目は遅いことがある（ウォームアップ）. 結果には入れない
    first = sorted(labels["frames"])[0]
    perception.point_path(args.path, frame=load_frame(os.path.join(args.frames, first), args.width)[0])

    points: List[Dict[str, Any]] = []
    checks: List[Dict[str, Any]] = []
    plans: List[Dict[str, Any]] = []
    if args.only in ("all", "point"):
        names = [n for n in sorted(labels["frames"]) if os.path.isfile(os.path.join(args.frames, n))]
        names = names[:args.limit] if args.limit > 0 else names
        print(f"\n[1] 道の指差し（{len(names)} 枚）")
        points = eval_points(perception, args.frames, labels, args.path, args.width, names, args.workers, out_dir)
        for r in points:
            print(f"  {r['frame']}  {r['verdict']:<12} " + (f"({r['x']}, {r['y']}) 余裕 {r['margin']}" if "x" in r else
                                                          r.get("error", "")) + f"  {r['latency_s']:.2f} 秒")
    if args.only in ("all", "check"):
        print("\n[2] 止まる条件の判定")
        checks = eval_statements(perception, args.frames, labels, args.width, args.workers, args.limit)
        for r in checks:
            mark = "OK" if r["answer"] == r["expected"] else "NG"
            print(f"  {mark}  {r['frame']}  正解 {r['expected']} 答え {r['answer']}  {r['statement'][:50]}")
    if args.only in ("all", "plan"):
        print("\n[3] 計画")
        plans = eval_plans(vlm, cfg)
        for r in plans:
            print(f"  {'OK' if r['ok'] else 'NG'}  {r['instruction']} → {r['detail']}")

    report = summarize(model, points, checks, plans, args)
    with open(os.path.join(out_dir, "report.md"), "w", encoding="utf-8") as f:
        f.write(report)
    with open(os.path.join(out_dir, "results.json"), "w", encoding="utf-8") as f:
        json.dump({"model": model, "path": args.path, "width": args.width, "points": points, "checks": checks,
                   "plans": plans}, f, ensure_ascii=False, indent=1)
    print("\n" + report)
    print(f"保存しました: {out_dir}（report.md、results.json、overlays/）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
