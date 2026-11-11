#!/usr/bin/env python3
"""実行の記録（events.jsonl）を読み、VLM の答えをシミュレータの真値と突き合わせてまとめる.

  python3 scripts/inside/analyze_run.py [記録のディレクトリ]      # 省略すると runs/ の最新
  python3 scripts/inside/analyze_run.py [記録のディレクトリ] --key-images <出力先>   # 要所の画像もコピーする

出すもの:
  - VLM が作った計画（条件文と目標の言い回し）
  - 手順ごとの条件待ち: 判定の回数、答えの並び、真値（場所 × 扉の様子）ごとの yes / no の数、食い違い
    （扉の様子が切り替わる前後の1枚と、全開との境目は、食い違いに数えない）
  - 指差し: 床の上の位置（前・左）と、正解の場所とのずれ、敷居の先の距離
  - 終わり方: 実行系の報告と、真値による成否
  - 気づいた点（食い違いから機械的に拾ったもの）

標準ライブラリだけで動く（コンテナの外でも使える）.
"""
import argparse
import glob
import os
import shutil
import sys
from typing import Any, Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_samples import DOOR_JA, DOORS, PLACE_JA, PLACES, load_events, place_of, truth_of  # noqa: E402

# 指差しの目標が「通り抜けた」と言える、敷居の先の距離 [m]（ロボットの半分の長さ 0.25 m + 余裕）
EXIT_MIN_BEYOND_M = 0.45
ENTER_BEYOND_M = (0.40, 1.15)       # かごの中に収まる範囲（奥行き 1.35 m のかごに、長さ 0.5 m のロボット）
# 扉の開きがこれより大きい「途中」は、全開との境目とみなす（画像と真値の時刻のずれがあるので、どちらの答えでもよい）
NEARLY_OPEN_RATIO = 0.85


def expected_yes(truth: Dict[str, Any]) -> Optional[bool]:
    """真値から決まる正解. 全開なら yes、全閉と途中なら no、全開との境目なら None（どちらでもよい）."""
    if truth["door"] == "open":
        return True
    if truth["door"] == "partial" and float(truth.get("door_ratio") or 0.0) > NEARLY_OPEN_RATIO:
        return None
    return False


def latest_run(root: str) -> Optional[str]:
    runs = sorted(d for d in glob.glob(os.path.join(root, "20*")) if os.path.isfile(os.path.join(d, "events.jsonl")))
    return runs[-1] if runs else None


def run_lengths(answers: List[str]) -> str:
    out: List[str] = []
    prev, count = None, 0
    for a in answers + [None]:
        if a == prev:
            count += 1
            continue
        if prev is not None:
            out.append(f"{prev}×{count}")
        prev, count = a, 1
    return " ".join(out)


def pct(values: List[float]) -> str:
    values = sorted(values)
    if not values:
        return ""
    shown = values if len(values) <= 6 else values[:3] + values[-3:]
    text = ", ".join(f"{round(v * 100)}%" for v in shown[:3])
    if len(values) > 6:
        text += ", …, " + ", ".join(f"{round(v * 100)}%" for v in shown[3:])
    elif len(values) > 3:
        text += ", " + ", ".join(f"{round(v * 100)}%" for v in shown[3:])
    return text


class Analysis:
    def __init__(self, events: List[Dict[str, Any]]):
        self.events = events
        self.lines: List[str] = []
        self.findings: List[str] = []
        self.ground = {e["image"]: e for e in events if e.get("kind") == "point_ground" and e.get("image")}
        self.has_truth = any(truth_of(e) is not None for e in events if e.get("kind") in ("check", "point"))

    def out(self, text: str = "") -> None:
        self.lines.append(text)

    def note(self, text: str) -> None:
        if text not in self.findings:
            self.findings.append(text)

    # -- 全体 ---------------------------------------------------------------
    def header(self, name: str) -> None:
        start = next((e for e in self.events if e.get("kind") == "runtime_start"), {})
        checks = [e["latency_s"] for e in self.events if e.get("kind") == "check" and e.get("latency_s") is not None]
        points = [e["latency_s"] for e in self.events if e.get("kind") == "point" and e.get("latency_s") is not None]
        self.out(f"記録: {name}   VLM: {start.get('vlm_model', '?')}（{start.get('vlm_base_url', '?')}）")
        if checks:
            self.out(f"判定 {len(checks)} 回（平均 {sum(checks) / len(checks):.2f} 秒、最大 {max(checks):.2f} 秒）、"
                     + (f"指差し {len(points)} 回（平均 {sum(points) / len(points):.2f} 秒、最大 {max(points):.2f} 秒）"
                        if points else "指差し 0 回"))
        if not self.has_truth:
            self.out("この記録には真値が添えられていません（古い版の記録）. 答えの並びだけを出します")

    def plan(self, event: Dict[str, Any]) -> None:
        how = "VLM が作成" if event.get("kind") == "plan_ready" else "読み込み"
        extra = f"、{event['latency_s']:.1f} 秒" if event.get("latency_s") is not None else ""
        if event.get("attempts", 1) and event.get("attempts", 1) > 1:
            extra += f"、{event['attempts']} 回目で通過"
        self.out("")
        self.out(f"計画（{how}{extra}）")
        for i, step in enumerate((event.get("plan") or {}).get("steps", [])):
            when, do = step.get("when", {}), step.get("do", {})
            cond = f"[{when.get('type')}]" + (f"「{when['condition']}」" if when.get("condition") else "")
            args = {k: v for k, v in do.items() if k != "skill"}
            act = do.get("skill", "?") + ("「" + str(args["target"]) + "」" if "target" in args
                                           else " " + " ".join(f"{k}={v}" for k, v in args.items()))
            self.out(f"  {i + 1}. {step.get('instruction', '')}")
            self.out(f"     {cond} → {act}")

    # -- 条件待ち ------------------------------------------------------------
    def wait_block(self, step: Dict[str, Any], checks: List[Dict[str, Any]], marks: List[Dict[str, Any]]) -> None:
        when = step.get("when", {})
        waits = [c for c in checks if c.get("purpose") == "check"]
        if not waits:
            return
        confirmed = [m for m in marks if m["kind"] == "condition_confirmed"]
        seconds = (confirmed[-1]["t"] - step["t"]) if confirmed and "t" in step else None
        end = "確定" if confirmed else "確定せず"
        self.out(f"   条件「{when.get('condition', '')}」（{when.get('type')}）: "
                 + (f"{seconds:.1f} 秒待ち、" if seconds is not None else "")
                 + f"判定 {len(waits)} 回 → {end}")
        self.out("     答えの並び: " + run_lengths([c.get("answer") or "?" for c in waits]))
        truths = [(c, truth_of(c)) for c in waits]
        truths = [(c, t) for c, t in truths if t is not None]
        if not truths:
            return
        self.out("     真値との突き合わせ（正解は「全開のときだけ yes」）")
        table: Dict[Any, Dict[str, List[float]]] = {}
        wrongs: Dict[Any, List[float]] = {}
        for i, (c, t) in enumerate(truths):
            key = (place_of(t)[0], t["door"])
            answer = c.get("answer") or "?"
            ratio = float(t.get("door_ratio") or 0.0)
            table.setdefault(key, {"yes": [], "no": [], "?": []})[answer].append(ratio)
            # 扉の様子が切り替わる前後の1枚は、画像と真値の時刻のずれがあるので、食い違いに数えない
            near = [truths[j][1]["door"] for j in (i - 1, i + 1) if 0 <= j < len(truths)]
            expected = None if any(d != t["door"] for d in near) else expected_yes(t)
            if expected is not None and answer in ("yes", "no") and (answer == "yes") != expected:
                wrongs.setdefault(key, []).append(ratio)
        for key in sorted(table, key=lambda k: (PLACES.index(k[0]), DOORS.index(k[1]))):
            row = table[key]
            parts = []
            for ans in ("yes", "no", "?"):
                if row[ans]:
                    detail = f"（開き {pct(row[ans])}）" if key[1] == "partial" else ""
                    parts.append(f"{ans} {len(row[ans])}{detail}")
            wrong_ratios = wrongs.get(key, [])
            wrong = len(wrong_ratios)
            self.out(f"       {PLACE_JA[key[0]]}・{DOOR_JA[key[1]]}: " + "、".join(parts)
                     + (f"   ← 食い違い {wrong}" if wrong else ""))
            place = PLACE_JA[key[0]]
            if key[1] == "open" and wrong >= 2:
                self.note(f"{place}で、扉が全開なのに no と答えています（{wrong} 回）. 近くからは開いた扉が"
                          "画面の外に出るので、条件文が「扉が見えていること」を前提にしていないか確かめてください")
            if key[1] == "partial" and wrong:
                self.note(f"{place}で、扉が動いている途中（開き {pct(wrong_ratios)}）に yes と答えています. "
                          "閉まりかけの扉に向かって発進するおそれがあります. 条件文に「完全に開いている」ことを入れてください")
            if key[1] == "closed" and wrong:
                self.note(f"{place}で、扉が全閉なのに yes と答えています（{wrong} 回）")
        for m in marks:
            t = truth_of(m.get("_last_check") or {})
            if m["kind"] == "condition_confirmed" and t is not None:
                self.out(f"     確定したときの扉（真値）: {DOOR_JA[t['door']]}（開き {round(float(t['door_ratio']) * 100)}%）")
                if expected_yes(t) is False:
                    self.note(f"扉が開き切っていないとき（開き {round(float(t['door_ratio']) * 100)}%）に条件が確定しました")
            if m["kind"] == "condition_false_confirmed" and t is not None and t["door"] == "open":
                self.out("     「成り立たない」を確定したときの扉（真値）: 全開   ← 食い違い")
                self.note("扉が全開のままなのに「一度閉じた」と確定しました（「再び」の待ちが、最初から成り立ってしまいます）")
        for m in marks:
            if m["kind"] == "departure_recheck_failed":
                self.out(f"     発進の直前の確認が通らず、条件待ちに戻りました（t={m.get('t')}）")

    # -- 指差し --------------------------------------------------------------
    def point_block(self, events: List[Dict[str, Any]]) -> None:
        for e in events:
            if e.get("kind") == "point" and e.get("purpose") == "point":
                self.out("     " + self.point_line(e, "指差し"))
            elif e.get("kind") == "point_rejected":
                self.out(f"     指差しを却下: {e.get('reason', '')}")
        repoints = [e for e in events if e.get("kind") == "point" and e.get("purpose") == "repoint"]
        if repoints:
            adopted = sum(1 for e in events if e.get("kind") == "target_updated")
            rejected = sum(1 for e in events if e.get("kind") == "repoint_rejected")
            self.out(f"     走行中のやり直し: {len(repoints)} 回（採用 {adopted}、却下 {rejected}）")
            for e in repoints:
                self.out("       " + self.point_line(e, "やり直し", brief=True))

    def point_line(self, e: Dict[str, Any], label: str, brief: bool = False) -> str:
        if not e.get("ok"):
            return f"{label}「{e.get('target', '')}」: 点を読めません（{e.get('error', '')}）"
        g = self.ground.get(e.get("image"))
        t = truth_of(e)
        head = f"{label}: " if brief else f"{label}「{e.get('target', '')}」: "
        if g is None:
            # 古い記録（床の上の位置を残していない）か、地平線より上を指した（床に当たらない）
            return head + f"画素 (u={e.get('u')}, v={e.get('v')})" + ("（床に当たらない点）" if self.ground else "")
        text = head + f"前 {g['ahead_m']:.2f} m・左 {g['left_m']:+.2f} m"
        if t is None or "goal_ahead_m" not in t:
            return text
        place = place_of(t)[0]
        da, dl = g["ahead_m"] - t["goal_ahead_m"], g["left_m"] - t.get("goal_left_m", 0.0)
        goal = "かごの中央" if place == "hall" else "乗場の 1 m 先"
        text += f"。正解（{goal}）とのずれ: 前後 {da:+.2f} m、左右 {dl:+.2f} m"
        if t.get("door_ahead_m") is not None:
            beyond = g["ahead_m"] - t["door_ahead_m"]
            text += f"。敷居の先 {beyond:.2f} m"
            if not brief:
                if place != "hall" and beyond < EXIT_MIN_BEYOND_M:
                    text += "   ← 近すぎる"
                    self.note(f"降りるときの目標が敷居に近すぎます（敷居の先 {beyond:.2f} m. ロボットが出入口に残ります）. "
                              "目標の言い回しに「出入口の約 1 m 先」を入れてください")
                if place == "hall" and not ENTER_BEYOND_M[0] <= beyond <= ENTER_BEYOND_M[1]:
                    text += "   ← かごの中に収まらない"
                    self.note(f"乗るときの目標が、かごの中に収まる範囲の外です（敷居の先 {beyond:.2f} m）")
        return text

    # -- 実行1回ぶん ----------------------------------------------------------
    def task(self, number: int, events: List[Dict[str, Any]]) -> None:
        self.out("")
        self.out(f"実行 {number}（開始 t={events[0].get('t', '?')}）")
        steps: List[List[Dict[str, Any]]] = []
        for e in events[1:]:
            if e.get("kind") == "step_start":
                steps.append([e])
            elif steps:
                steps[-1].append(e)
        for group in steps:
            step = group[0]
            self.out(f" 手順 {step.get('step', 0) + 1}「{step.get('instruction', '')}」")
            buttons = [f"{e.get('name')}（{e.get('message', '')}）" for e in group if e.get("kind") == "button"]
            if buttons:
                self.out("   この間のボタン操作: " + " → ".join(buttons))
            checks = [e for e in group if e.get("kind") == "check"]
            marks = []
            last_check: Optional[Dict[str, Any]] = None
            for e in group:
                if e.get("kind") == "check" and e.get("purpose") == "check":
                    last_check = e
                elif e.get("kind") in ("condition_confirmed", "condition_false_confirmed", "departure_recheck_failed"):
                    marks.append(dict(e, _last_check=last_check))
            self.wait_block(step, checks, marks)
            do = step.get("do", {})
            starts = [e for e in group if e.get("kind") == "skill_start"]
            ends = [e for e in group if e.get("kind") == "skill_end"]
            if starts:
                args = {k: v for k, v in do.items() if k != "skill"}
                self.out(f"   動作 {do.get('skill')}"
                         + ("「" + str(args["target"]) + "」" if "target" in args
                            else " " + " ".join(f"{k}={v}" for k, v in args.items())))
                self.point_block(group)
            for s, e in zip(starts, ends):
                took = f"（{e['t'] - s['t']:.1f} 秒）" if "t" in e and "t" in s else ""
                self.out(f"     結果: {'' if e.get('ok') else '失敗 — '}{e.get('message', '')}{took}")
        ended = next((e for e in events if e.get("kind") in ("task_done", "task_failed")), None)
        status = next((e for e in events if e.get("kind") == "status"), None)
        report = "記録なし（中断）"
        if ended is not None:
            report = "完了" if ended["kind"] == "task_done" else f"中止（{ended.get('message', '')}）"
        line = f" 終わり方: 実行系の報告は「{report}」"
        referee = ((status or {}).get("status") or {}).get("referee") if status else None
        if isinstance(referee, dict):
            line += f"。真値: {referee.get('summary') or ('成功' if referee.get('success') else '成功していません')}"
            if ended is not None and ended["kind"] == "task_done" and not referee.get("success"):
                self.note("実行系は「完了」と報告しましたが、真値では成功していません")
        self.out(line)

    def run(self, name: str) -> str:
        self.header(name)
        last_plan = None
        number = 0
        current: Optional[List[Dict[str, Any]]] = None
        tasks: List[List[Dict[str, Any]]] = []
        for e in self.events:
            kind = e.get("kind")
            if kind in ("plan_ready", "plan_loaded"):
                last_plan = e
                current = None
            elif kind == "task_start":
                if last_plan is not None:
                    self.plan(last_plan)
                    last_plan = None
                current = [e]
                tasks.append(current)
            elif current is not None:
                current.append(e)
            if kind == "status":
                current = None
        if last_plan is not None:
            self.plan(last_plan)
        for events in tasks:
            number += 1
            self.task(number, events)
        if self.findings:
            self.out("")
            self.out("気づいた点")
            for f in self.findings:
                self.out(f"  - {f}")
        return "\n".join(self.lines)


def key_images(events: List[Dict[str, Any]]) -> List[str]:
    """記録と一緒に残す画像を選ぶ（全部は多いので、要所だけ）.

    指差しに使った画像は全部. 判定に使った画像は、同じ答えが続く区間の最初と最後と、10 枚おき.
    """
    picked: List[str] = []

    def add(e: Dict[str, Any]) -> None:
        if e.get("image") and e["image"] not in picked:
            picked.append(e["image"])

    def same(a: Dict[str, Any], b: Dict[str, Any]) -> bool:
        return a.get("answer") == b.get("answer") and a.get("condition") == b.get("condition")

    checks = [e for e in events if e.get("kind") == "check" and e.get("image")]
    index_in_run = 0
    for i, e in enumerate(checks):
        first = i == 0 or not same(checks[i - 1], e)
        last = i + 1 == len(checks) or not same(checks[i + 1], e)
        index_in_run = 0 if first else index_in_run + 1
        if first or last or index_in_run % 10 == 0 or e.get("purpose") != "check":
            add(e)
    for e in events:
        if e.get("kind") == "point":
            add(e)
    return picked


def main() -> int:
    parser = argparse.ArgumentParser(description="実行の記録を読み、VLM の答えを真値と突き合わせてまとめる")
    parser.add_argument("run_dir", nargs="?", default="", help="記録のディレクトリ（省略すると runs/ の最新）")
    parser.add_argument("--key-images", default="", metavar="DEST",
                        help="要所の画像を DEST/images/ にコピーする（記録を共有するとき用）")
    opts = parser.parse_args()
    run_dir = opts.run_dir
    if not run_dir:
        root = "/runs" if os.path.isdir("/runs") else os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "runs")
        run_dir = latest_run(root) or ""
        if not run_dir:
            print(f"記録がありません: {root}")
            return 1
    if not os.path.isfile(os.path.join(run_dir, "events.jsonl")):
        print(f"events.jsonl がありません: {run_dir}")
        return 1
    events = load_events(run_dir)
    print(Analysis(events).run(run_dir))
    if opts.key_images:
        copied = 0
        for image in key_images(events):
            src, dst = os.path.join(run_dir, image), os.path.join(opts.key_images, image)
            if os.path.isfile(src):
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copyfile(src, dst)
                copied += 1
        print(f"\n要所の画像 {copied} 枚をコピーしました: {os.path.join(opts.key_images, 'images')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
