#!/usr/bin/env python3
"""実行の記録から、正解つきの画像の組（机上評価用）を作る.

  python3 scripts/inside/export_samples.py <記録のディレクトリ> <出力先> [--per-category N] [--append]

正解は、記録に添えてあるシミュレータの真値（各記録の ctx）から決める. VLM の答えは使わないので、
モック VLM で通した記録でも、本物の VLM で動かした記録でも使える.

  判定: 場所（乗場 / かごの中 / かごの中で扉の近く）× 階 × 扉の様子（全閉 / 途中 / 全開）ごとに、数枚ずつ取る.
        正解は「全開のときだけ yes」. 切り替わりの前後の1枚は、画像と真値の時刻のずれがあるので取らない.
  指差し: 発進の前（ロボットが止まっていて、扉が開き切っているとき）の指差しを、場所 × 階ごとに取る.
        正解は、次に向かうはずの場所（乗場からはかごの中央、かごの中からは乗場の 1 m 先）.

--append を付けると、出力先に既にある組に足す（複数の記録から1つの組を作るとき）.
"""
import argparse
import json
import os
import shutil
import sys
from typing import Any, Dict, List, Optional, Tuple

# 場所の分類. かごの中は、扉までの近さで2つに分ける: 近くからは、開き切った扉が画面の外に出るので、見え方が違う
PLACE_JA = {"hall": "乗場", "car": "かごの中", "car_near": "かごの中（扉の近く）"}
PLACES = ("hall", "car", "car_near")
NEAR_DOOR_M = 0.65                   # ロボットの中心から敷居までがこれより近ければ「扉の近く」（かごの中央は 0.78 m）
DOOR_JA = {"closed": "全閉", "partial": "途中", "open": "全開"}
DOORS = ("closed", "partial", "open")


def load_events(run_dir: str) -> List[Dict[str, Any]]:
    events = []
    with open(os.path.join(run_dir, "events.jsonl"), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    events.append(json.loads(line))
                except ValueError:
                    continue                       # 書きかけの行
    return events


def truth_of(event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """記録に添えてある真値（無ければ None）. 扉の様子とロボットの場所が入っているものだけを返す."""
    ctx = (event.get("ctx") or {}).get("elevator")
    if not isinstance(ctx, dict) or ctx.get("door") not in DOORS or "robot_in_car" not in ctx:
        return None
    return ctx


def place_of(ctx: Dict[str, Any]) -> Tuple[str, int]:
    """（場所, 階）. かごの中にいて昇降中のときは、階を 0 とする."""
    if ctx.get("robot_in_car"):
        ahead = ctx.get("door_ahead_m")
        near = ahead is not None and 0.0 < float(ahead) < NEAR_DOOR_M
        return ("car_near" if near else "car"), int(ctx.get("floor") or 0)
    return "hall", int(ctx.get("robot_floor") or 0)


def _spread(items: List[Dict[str, Any]], count: int) -> List[Dict[str, Any]]:
    """並びの全体から、均等に count 個を取る."""
    if len(items) <= count:
        return list(items)
    if count == 1:
        return [items[len(items) // 2]]
    return [items[round(i * (len(items) - 1) / (count - 1))] for i in range(count)]


def labelled_checks(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """真値つきの判定の記録を、正解のラベルを付けて返す（切り替わりの前後は除く）."""
    checks = [e for e in events
              if e.get("kind") == "check" and e.get("purpose") in ("check", "recheck") and e.get("image")
              and truth_of(e) is not None]
    out = []
    for i, e in enumerate(checks):
        ctx = truth_of(e)
        place, floor = place_of(ctx)
        door, ratio = ctx["door"], float(ctx.get("door_ratio") or 0.0)
        if door == "partial":
            if not 0.15 <= ratio <= 0.85:
                continue                           # 全閉や全開と見分けにくい所は使わない
        else:
            near = [truth_of(checks[j])["door"] for j in (i - 1, i + 1) if 0 <= j < len(checks)]
            if any(d != door for d in near):
                continue
        out.append({"kind": "check", "image": e["image"], "place": place, "floor": floor, "door": door,
                    "door_ratio": round(ratio, 2), "answer": e.get("answer"), "condition": e.get("condition")})
    return out


def labelled_points(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """真値つきの、発進の前の指差しの記録を、正解の場所を付けて返す."""
    out = []
    for e in events:
        ctx = truth_of(e)
        if e.get("kind") != "point" or e.get("purpose") != "point" or not e.get("image") or ctx is None:
            continue
        if float(ctx.get("door_ratio") or 0.0) < 0.9 or "goal_ahead_m" not in ctx:
            continue
        place, floor = place_of(ctx)
        out.append({"kind": "point", "image": e["image"], "place": place, "floor": floor,
                    "goal_ahead_m": ctx["goal_ahead_m"], "goal_left_m": ctx.get("goal_left_m", 0.0),
                    "door_ahead_m": ctx.get("door_ahead_m"), "target": e.get("target")})
    return out


def select(events: List[Dict[str, Any]], per_category: int = 2) -> List[Dict[str, Any]]:
    """机上評価に使う画像を選ぶ. 分類ごとに per_category 枚まで."""
    samples: List[Dict[str, Any]] = []
    groups: Dict[Tuple[str, int, str], List[Dict[str, Any]]] = {}
    for s in labelled_checks(events):
        groups.setdefault((s["place"], s["floor"], s["door"]), []).append(s)
    for key in sorted(groups, key=lambda k: (PLACES.index(k[0]), k[1], DOORS.index(k[2]))):
        group = groups[key]
        if key[2] == "partial":
            group = sorted(group, key=lambda s: s["door_ratio"])
        samples.extend(_spread(group, per_category))
    points: Dict[Tuple[str, int], List[Dict[str, Any]]] = {}
    for s in labelled_points(events):
        points.setdefault((s["place"], s["floor"]), []).append(s)
    for key in sorted(points, key=lambda k: (PLACES.index(k[0]), k[1])):
        samples.extend(_spread(points[key], per_category))
    return samples


def file_name(sample: Dict[str, Any], index: int) -> str:
    base = f"{sample['kind']}_{sample['place']}_f{sample['floor']}"
    if sample["kind"] == "check":
        base += f"_{sample['door']}"
        if sample["door"] == "partial":
            base += f"{round(sample['door_ratio'] * 100):02d}"
    return f"{base}_{index:02d}.jpg"


def main() -> int:
    parser = argparse.ArgumentParser(description="実行の記録から、正解つきの画像の組（机上評価用）を作る")
    parser.add_argument("run_dir", help="記録のディレクトリ（events.jsonl と images/ がある所）")
    parser.add_argument("out_dir", help="出力先")
    parser.add_argument("--per-category", type=int, default=2, help="分類ごとの枚数（既定 2）")
    parser.add_argument("--append", action="store_true", help="出力先に既にある組に足す")
    opts = parser.parse_args()
    run_dir, out_dir, per_category = opts.run_dir, opts.out_dir, opts.per_category
    samples = select(load_events(run_dir), per_category)
    if not samples:
        print(f"真値つきの記録がありません（シミュレーションで動かした新しい記録が必要です）: {run_dir}")
        return 1
    os.makedirs(out_dir, exist_ok=True)
    labels_path = os.path.join(out_dir, "labels.json")
    labels: List[Dict[str, Any]] = []
    if opts.append and os.path.isfile(labels_path):
        with open(labels_path, encoding="utf-8") as f:
            labels = json.load(f)
    start = len(labels)
    for i, s in enumerate(samples):
        name = file_name(s, start + i)
        shutil.copyfile(os.path.join(run_dir, s["image"]), os.path.join(out_dir, name))
        item = {k: v for k, v in s.items() if k not in ("image", "answer", "condition", "target") and v is not None}
        item["file"] = name
        labels.append(item)
    with open(labels_path, "w", encoding="utf-8") as f:
        json.dump(labels, f, ensure_ascii=False, indent=2)
        f.write("\n")
    counts: Dict[str, int] = {}
    for item in labels:
        key = (f"{PLACE_JA[item['place']]}・{DOOR_JA[item['door']]}" if item["kind"] == "check"
               else f"指差し（{PLACE_JA[item['place']]}から）")
        counts[key] = counts.get(key, 0) + 1
    print(f"{len(samples)} 枚を書き出しました（合計 {len(labels)} 枚）: {out_dir}")
    print("  " + "、".join(f"{k} {v}" for k, v in counts.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
