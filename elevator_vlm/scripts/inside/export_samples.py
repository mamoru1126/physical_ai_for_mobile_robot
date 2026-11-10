#!/usr/bin/env python3
"""実行の記録から、正解つきの画像の組（机上評価用）を作る.

  python3 scripts/inside/export_samples.py <記録のディレクトリ> <出力先>

モック VLM で通した記録を使うこと. モック VLM の答えはシミュレータの真値なので、
その答えをそのまま正解として使える（扉が開いているか、目標の点が画像のどこか）.

  判定: no は、答えが安定している区間（3回以上続く所）の中ほどから取る.
        yes は、条件が確定したときの最後の1枚を取る（確定するとすぐ発進するので、yes は長く続かない）.
        扉が動いている最中の画像は、正解があいまいなので取らない.
  指差し: 発進の前（ロボットが止まっているとき）の指差しだけを取る.
"""
import json
import os
import shutil
import sys
from typing import Any, Dict, List


def load_events(run_dir: str) -> List[Dict[str, Any]]:
    events = []
    with open(os.path.join(run_dir, "events.jsonl"), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def select(events: List[Dict[str, Any]], per_run: int = 1) -> List[Dict[str, Any]]:
    samples: List[Dict[str, Any]] = []
    step = -1
    runs: List[List[Dict[str, Any]]] = []          # 同じ手順で同じ答えが続く区間
    for e in events:
        kind = e.get("kind")
        if kind == "step_start":
            step = int(e.get("step", -1))
            runs.append([])
        elif kind == "check" and e.get("purpose") == "check" and e.get("image") and e.get("answer") in ("yes", "no"):
            e = dict(e, step=step)
            if runs and runs[-1] and runs[-1][-1]["answer"] == e["answer"] and runs[-1][-1]["step"] == step:
                runs[-1].append(e)
            else:
                runs.append([e])
        elif kind == "point" and e.get("purpose") == "point" and e.get("ok") and e.get("image"):
            samples.append({"kind": "point", "image": e["image"], "target": e["target"], "step": step,
                            "u": e["u"], "v": e["v"]})
    for run in runs:
        if not run:
            continue
        if run[0]["answer"] == "yes":
            picked = run[-1:] if len(run) >= 2 else []
        elif len(run) >= 3:
            inner = run[1:-1]                       # 切り替わりの直前と直後は使わない
            stride = max(1, len(inner) // per_run)
            picked = inner[::stride][:per_run]
        else:
            picked = []
        for e in picked:
            samples.append({"kind": "check", "image": e["image"], "condition": e["condition"],
                            "expected": e["answer"], "step": e["step"]})
    return samples


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    run_dir, out_dir = sys.argv[1], sys.argv[2]
    samples = select(load_events(run_dir))
    os.makedirs(out_dir, exist_ok=True)
    labels = []
    for i, s in enumerate(samples):
        name = f"{i:02d}_{s['kind']}_step{s['step']}" + (f"_{s['expected']}" if s["kind"] == "check" else "") + ".jpg"
        shutil.copyfile(os.path.join(run_dir, s["image"]), os.path.join(out_dir, name))
        item = {k: v for k, v in s.items() if k != "image"}
        item["file"] = name
        labels.append(item)
    with open(os.path.join(out_dir, "labels.json"), "w", encoding="utf-8") as f:
        json.dump(labels, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(f"{len(labels)} 枚を書き出しました: {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
