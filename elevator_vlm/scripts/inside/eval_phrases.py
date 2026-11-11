#!/usr/bin/env python3
"""条件文と目標の言い回しを、正解つきの画像で比べる（VLM サーバが要る）.

  python3 scripts/inside/eval_phrases.py                    # samples/ の画像で比べる
  python3 scripts/inside/eval_phrases.py --latest-run       # 直近の実行の記録の画像も足す
  python3 scripts/inside/eval_phrases.py --run /runs/<名前>  # 指定した記録の画像も足す

比べるもの:
  判定: 候補の条件文 × システムプロンプト（guided / basic）ごとに、場所（乗場 / かごの中 / かごの中で扉の近く）× 扉の様子
        （全閉 / 途中 / 全開）の画像へ問いかけ、「全開のときだけ yes」と答えられた数を数える.
  指差し: 候補の目標ごとに、指差した点が床のどこに当たるかを出し、正解の場所と比べる.

候補は samples/phrases.json に書く. --run / --latest-run を付けると、その記録で VLM が作った計画の
言い回しも候補に足す. 結果は画面と /runs/phrases_<日時>.txt（と .json）に出す.
"""
import argparse
import dataclasses
import io
import json
import math
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.environ.get("ELEVATOR_VLM_ROOT", "/workspace")
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "elevator_task"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PIL import Image  # noqa: E402

from analyze_run import ENTER_BEYOND_M, EXIT_MIN_BEYOND_M, latest_run  # noqa: E402
from elevator_task.core.config import load_config  # noqa: E402
from elevator_task.core.events import EventLog  # noqa: E402
from elevator_task.core.geometry import CameraModel, pixel_to_ground  # noqa: E402
from elevator_task.core.perception import CHECK_SYSTEMS, Perception  # noqa: E402
from elevator_task.core.robot_io import Frame, RobotIO  # noqa: E402
from elevator_task.core.vlm_client import VlmClient  # noqa: E402
from export_samples import DOOR_JA, DOORS, PLACE_JA, PLACES, load_events, select  # noqa: E402

TARGET_PLACES = ("hall", "car")      # 目標の候補は、乗場から（乗る）と、かごの中から（降りる）の2組
PLACE_SHORT = {"hall": "乗場", "car": "かご", "car_near": "近く"}
DOOR_SHORT = {"closed": "閉", "partial": "途", "open": "開"}
LATERAL_TOL_M = 0.25                 # 目標の左右のずれの許容


class NoRobot(RobotIO):
    """画像は呼び出し側が渡すので、ロボットは要らない."""

    def now(self) -> float:
        return 0.0


def load_samples(samples_dir: str, runs: List[str], per_category: int) -> List[Dict[str, Any]]:
    """正解つきの画像の一覧. 各項目に path（画像ファイル）と source（出どころ）を足す."""
    items: List[Dict[str, Any]] = []
    labels_path = os.path.join(samples_dir, "labels.json")
    if os.path.isfile(labels_path):
        with open(labels_path, encoding="utf-8") as f:
            for item in json.load(f):
                if item.get("place") in PLACES:           # 古い形式（場所と扉の様子がない）は使わない
                    items.append(dict(item, path=os.path.join(samples_dir, item["file"]), source="samples"))
    for run in runs:
        for s in select(load_events(run), per_category):
            items.append(dict(s, path=os.path.join(run, s["image"]), file=os.path.basename(s["image"]),
                              source=os.path.basename(run.rstrip("/"))))
    return [i for i in items if os.path.isfile(i["path"])]


def plan_phrases(run: str) -> Tuple[List[str], Dict[str, List[str]]]:
    """記録の中の計画（VLM が作ったもの、読み込んだもの）から、条件文と目標を拾う."""
    conditions: List[str] = []
    targets: Dict[str, List[str]] = {p: [] for p in TARGET_PLACES}
    for e in load_events(run):
        if e.get("kind") not in ("plan_ready", "plan_loaded"):
            continue
        place = "hall"                                     # 最初の「目標へ向かう」手順は乗場から、次はかごの中から
        for step in (e.get("plan") or {}).get("steps", []):
            cond = (step.get("when") or {}).get("condition")
            if cond and cond not in conditions:
                conditions.append(cond)
            target = (step.get("do") or {}).get("target")
            if target:
                if target not in targets[place]:
                    targets[place].append(target)
                place = "car"
    return conditions, targets


def frame_of(path: str) -> Frame:
    with open(path, "rb") as f:
        img = Image.open(io.BytesIO(f.read())).convert("RGB")
    return Frame(0.0, img.width, img.height, img.tobytes())


def camera_of(cfg, frame: Frame) -> CameraModel:
    mount = cfg.camera
    focal = frame.width / (2.0 * math.tan(math.radians(mount.hfov_deg) / 2.0))
    return CameraModel(focal, focal, frame.width / 2.0, frame.height / 2.0, frame.width, frame.height,
                       mount.height_m, math.radians(mount.pitch_deg), mount.x_offset_m)


def eval_conditions(cfg, vlm: VlmClient, phrases: List[str], systems: List[str], checks: List[Dict[str, Any]],
                    frames: Dict[str, Frame], workers: int) -> List[Dict[str, Any]]:
    results = []
    for phrase in phrases:
        for system in systems:
            pcfg = dataclasses.replace(cfg.perception, check_prompt=system, check_system="")
            perception = Perception(NoRobot(), vlm, pcfg, EventLog(None))

            def ask(item: Dict[str, Any], perception=perception, phrase=phrase) -> Tuple[Optional[bool], float]:
                res = perception.check(phrase, frame=frames[item["path"]])
                return res.answer, res.latency_s

            with ThreadPoolExecutor(max_workers=workers) as pool:
                answers = list(pool.map(ask, checks))
            by: Dict[str, List[int]] = {}
            wrong = []
            for item, (answer, _) in zip(checks, answers):
                key = f"{item['place']}/{item['door']}"
                good = answer is (item["door"] == "open")
                cell = by.setdefault(key, [0, 0])
                cell[0] += 1 if good else 0
                cell[1] += 1
                if not good:
                    wrong.append({"file": item["file"], "source": item["source"], "place": item["place"],
                                  "door": item["door"], "door_ratio": item.get("door_ratio"),
                                  "answer": None if answer is None else ("yes" if answer else "no")})
            latencies = [lat for _, lat in answers if lat]
            results.append({"phrase": phrase, "system": system,
                            "correct": sum(c for c, _ in by.values()), "total": sum(t for _, t in by.values()),
                            "by_category": by, "wrong": wrong,
                            "latency_s": round(sum(latencies) / len(latencies), 3) if latencies else None})
    return results


def eval_targets(cfg, vlm: VlmClient, targets: Dict[str, List[str]], points: List[Dict[str, Any]],
                 frames: Dict[str, Frame], workers: int) -> List[Dict[str, Any]]:
    perception = Perception(NoRobot(), vlm, cfg.perception, EventLog(None))
    results = []
    for place in PLACES:
        items = [p for p in points if p["place"] == place]
        if not items:
            continue
        for phrase in targets.get("hall" if place == "hall" else "car", []):
            def ask(item: Dict[str, Any], phrase=phrase, place=place) -> Dict[str, Any]:
                frame = frames[item["path"]]
                res = perception.point(phrase, frame=frame)
                out: Dict[str, Any] = {"file": item["file"], "source": item["source"], "ok": False}
                if not res.ok:
                    out["error"] = res.error
                    return out
                ground = pixel_to_ground(camera_of(cfg, frame), res.u, res.v)
                out.update(u=round(res.u, 1), v=round(res.v, 1))
                if ground is None:
                    out["error"] = "床に当たらない点です（地平線より上）"
                    return out
                ahead, left = ground
                out.update(ahead_m=round(ahead, 3), left_m=round(left, 3),
                           d_ahead_m=round(ahead - item["goal_ahead_m"], 3),
                           d_left_m=round(left - item.get("goal_left_m", 0.0), 3))
                lateral_ok = abs(out["d_left_m"]) <= LATERAL_TOL_M
                if item.get("door_ahead_m") is None:
                    out["ok"] = lateral_ok and abs(out["d_ahead_m"]) <= 0.35
                    return out
                beyond = ahead - item["door_ahead_m"]
                out["beyond_m"] = round(beyond, 3)
                if place == "hall":
                    out["ok"] = lateral_ok and ENTER_BEYOND_M[0] <= beyond <= ENTER_BEYOND_M[1]
                else:
                    out["ok"] = (lateral_ok and beyond >= EXIT_MIN_BEYOND_M
                                 and math.hypot(ahead, left) <= cfg.skills.max_target_distance_m)
                return out

            with ThreadPoolExecutor(max_workers=workers) as pool:
                rows = list(pool.map(ask, items))
            results.append({"place": place, "phrase": phrase, "ok": sum(1 for r in rows if r["ok"]),
                            "total": len(rows), "results": rows})
    return results


def report(model: str, checks: List[Dict[str, Any]], points: List[Dict[str, Any]],
           cond: List[Dict[str, Any]], targ: List[Dict[str, Any]]) -> str:
    lines: List[str] = []
    counts: Dict[str, int] = {}
    for c in checks:
        key = f"{PLACE_JA[c['place']]}・{DOOR_JA[c['door']]}"
        counts[key] = counts.get(key, 0) + 1
    lines.append(f"言い回しの比較   VLM: {model}")
    lines.append(f"画像: 判定 {len(checks)} 枚（" + "、".join(f"{k} {v}" for k, v in counts.items())
                 + f"）、指差し {len(points)} 枚")
    if cond:
        lines.append("")
        lines.append("判定の条件文（正解は「全開のときだけ yes」. 数字は 正解 / 枚数. 閉 = 全閉、途 = 途中、開 = 全開、"
                     "近く = かごの中で扉の近く）")
        phrases = []
        for r in cond:
            if r["phrase"] not in phrases:
                phrases.append(r["phrase"])
        for i, phrase in enumerate(phrases):
            lines.append(f"[C{i + 1}] {phrase}")
            for r in (x for x in cond if x["phrase"] == phrase):
                cells = []
                for place in PLACES:
                    part = [f"{DOOR_SHORT[d]}{r['by_category'][f'{place}/{d}'][0]}/{r['by_category'][f'{place}/{d}'][1]}"
                            for d in DOORS if f"{place}/{d}" in r["by_category"]]
                    if part:
                        cells.append(f"{PLACE_SHORT[place]} " + " ".join(part))
                rate = 100.0 * r["correct"] / r["total"] if r["total"] else 0.0
                lat = f" {r['latency_s']:.2f}秒" if r["latency_s"] else ""
                lines.append(f"     {r['system']:<6} " + " ｜ ".join(cells)
                             + f" ｜ 計 {r['correct']}/{r['total']}（{rate:.0f}%）{lat}")
        ranked = sorted(cond, key=lambda r: (-r["correct"], r["latency_s"] or 0.0))
        lines.append("")
        lines.append("合計の高い順（上位 3）")
        for r in ranked[:3]:
            lines.append(f"  {r['correct']}/{r['total']}  [{r['system']}] {r['phrase']}")
        best = ranked[0]
        if best["wrong"]:
            lines.append("  1位が外した画像: " + "、".join(
                f"{w['file']}（{PLACE_JA[w['place']]}・{DOOR_JA[w['door']]}"
                + (f" {round(w['door_ratio'] * 100)}%" if w["door"] == "partial" and w.get("door_ratio") is not None else "")
                + f" → {w['answer'] or '?'}）" for w in best["wrong"][:8]))
    if targ:
        lines.append("")
        lines.append("指差しの目標（床の上の位置. OK の範囲: 乗るときは敷居の先 "
                     f"{ENTER_BEYOND_M[0]:.2f}〜{ENTER_BEYOND_M[1]:.2f} m、降りるときは {EXIT_MIN_BEYOND_M:.2f} m 以上、"
                     f"左右のずれ {LATERAL_TOL_M:.2f} m 以内）")
        number = 0
        for place in PLACES:
            rows = [r for r in targ if r["place"] == place]
            if not rows:
                continue
            lines.append(f" {PLACE_JA[place]}から（正解は{'かごの中央' if place == 'hall' else '乗場の 1 m 先'}）")
            for r in rows:
                number += 1
                lines.append(f" [T{number}] {r['phrase']}   → OK {r['ok']}/{r['total']}")
                for x in r["results"]:
                    if "ahead_m" not in x:
                        lines.append(f"       {x['file']}: {x.get('error', '点を読めません')}")
                        continue
                    beyond = f"、敷居の先 {x['beyond_m']:.2f} m" if "beyond_m" in x else ""
                    lines.append(f"       {x['file']}: 前 {x['ahead_m']:.2f} m・左 {x['left_m']:+.2f} m{beyond}"
                                 f"（正解とのずれ 前後 {x['d_ahead_m']:+.2f} m、左右 {x['d_left_m']:+.2f} m）"
                                 f" {'OK' if x['ok'] else 'NG'}")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="条件文と目標の言い回しを、正解つきの画像で比べる")
    parser.add_argument("--samples", default=os.path.join(ROOT, "samples"), help="正解つきの画像の組（既定 samples/）")
    parser.add_argument("--run", action="append", default=[], help="画像を足す記録のディレクトリ（複数可）")
    parser.add_argument("--latest-run", action="store_true", help="直近の実行の記録の画像を足す")
    parser.add_argument("--phrases", default="", help="候補のファイル（既定 <samples>/phrases.json）")
    parser.add_argument("--system", default="both", choices=["both"] + sorted(CHECK_SYSTEMS),
                        help="判定のシステムプロンプト（既定 both: 両方を比べる）")
    parser.add_argument("--per-category", type=int, default=3, help="記録から取る画像の、分類ごとの枚数（既定 3）")
    parser.add_argument("--workers", type=int, default=4, help="同時に問いかける数（既定 4）")
    parser.add_argument("--out", default="/runs", help="結果の保存先のディレクトリ（既定 /runs）")
    opts = parser.parse_args()

    config = os.path.join(ROOT, "ros2_ws", "src", "elevator_sim", "config", "task_sim.yaml")
    cfg = load_config(config)
    vlm = VlmClient(cfg.vlm)
    ok, info = vlm.ping()
    if not ok:
        print(f"NG: VLM サーバに接続できません（{info}）. scripts/vlm_up.sh で起動してください")
        return 1
    runs = list(opts.run)
    if opts.latest_run:
        run = latest_run("/runs")
        if run and run not in runs:
            runs.append(run)
    with open(opts.phrases or os.path.join(opts.samples, "phrases.json"), encoding="utf-8") as f:
        phrases = json.load(f)
    conditions: List[str] = list(phrases.get("conditions", []))
    targets: Dict[str, List[str]] = {p: list((phrases.get("targets") or {}).get(p, [])) for p in TARGET_PLACES}
    for run in runs:
        extra_conditions, extra_targets = plan_phrases(run)
        conditions += [c for c in extra_conditions if c not in conditions]
        for place in TARGET_PLACES:
            targets[place] += [t for t in extra_targets[place] if t not in targets[place]]

    samples = load_samples(opts.samples, runs, opts.per_category)
    checks = [s for s in samples if s["kind"] == "check"]
    points = [s for s in samples if s["kind"] == "point"]
    if not checks and not points:
        print("正解つきの画像がありません（samples/labels.json か、真値つきの記録が必要です）")
        return 1
    frames = {s["path"]: frame_of(s["path"]) for s in samples}
    systems = sorted(CHECK_SYSTEMS, reverse=True) if opts.system == "both" else [opts.system]
    calls = len(conditions) * len(systems) * len(checks) + sum(
        len(targets["hall" if x["place"] == "hall" else "car"]) for x in points)
    print(f"VLM サーバ: {cfg.vlm.base_url}（{info}）  問いかけ {calls} 回. しばらくかかります…", flush=True)
    t0 = time.monotonic()
    cond = eval_conditions(cfg, vlm, conditions, systems, checks, frames, opts.workers) if checks else []
    targ = eval_targets(cfg, vlm, targets, points, frames, opts.workers) if points else []
    text = report(cfg.vlm.model, checks, points, cond, targ)
    text += f"\n\n（{time.monotonic() - t0:.0f} 秒）"
    print(text)
    if os.path.isdir(opts.out):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(opts.out, f"phrases_{stamp}")
        try:
            with open(base + ".txt", "w", encoding="utf-8") as f:
                f.write(text + "\n")
            with open(base + ".json", "w", encoding="utf-8") as f:
                json.dump({"model": cfg.vlm.model, "runs": runs, "conditions": cond, "targets": targ},
                          f, ensure_ascii=False, indent=1)
                f.write("\n")
            print(f"保存しました: {base}.txt")
        except OSError as e:
            print(f"結果を保存できません: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
