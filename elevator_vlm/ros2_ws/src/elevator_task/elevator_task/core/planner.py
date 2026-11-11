"""指示文を計画（JSON）に変える. VLM を1回（直しが要れば2回）呼ぶ.

プロンプトには、使えるスキルと「いつ」の3種類、条件文と目標の書き方だけを書く. エレベーターの説明は入れない
（書き方の例も、別の作業のものにしてある）.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

from .plan import DEFAULT_LIMITS, PLAN_JSON_SCHEMA, Plan, PlanError, PlanLimits, parse_plan
from .vlm_client import VlmClient, extract_json, image_part

PLANNER_SYSTEM = """You are the task planner of a wheeled mobile robot that has one forward-facing camera and wheel odometry.
Convert the operator's instructions into an ordered list of steps. Each instruction becomes exactly one step, in the same order.

Each step has four fields.
- "instruction": the original instruction text, copied verbatim.
- "when": when the step may start. One of:
  - {"type": "now"}: start as soon as the previous step has finished. Use this when the instruction only depends on the previous action being completed (for example "after arriving, ..." right after a step that drives there).
  - {"type": "when_true", "condition": "..."}: wait until the condition is visibly true in the camera image, then start.
  - {"type": "after_false_then_true", "condition": "..."}: use this when the instruction says the event must happen AGAIN (for example "again", "once more", "再び", "もう一度"). The condition may already be true when the step begins, so the robot must first see it become false and then see it become true.
- "do": the action. One of the skills:
  - {"skill": "rotate", "angle_deg": A}: turn in place by A degrees (positive is counter-clockwise, to the left). Use when the instruction gives a turning angle.
  - {"skill": "move_straight", "distance_m": D}: drive straight by D metres (negative is backwards). Use ONLY when the instruction states the distance as a number.
  - {"skill": "go_to_visual_target", "target": "..."}: drive to a place on the floor that is found in the camera image. Use when the destination is described by the scene (for example "go inside", "go out", "move in front of the desk") and no distance is given.
  - {"skill": "wait", "seconds": S}: stay still for S seconds.
- "timeout_s": give up when the step has not finished after this many seconds. Use 120 for "when_true" steps, 300 for "after_false_then_true" steps and 30 for "now" steps.

Rules for the text you write:
- Write every "condition" and "target" in English, even when the instruction is in another language. Name the things you can recognise in the camera image.
- A condition is one short statement that can be judged true or false from a single camera image, without knowing what happened before. The camera is mounted low. It may be so close to an opening that the open door itself is outside the picture, and other doors may be visible farther away. So when the robot waits for a door, gate, shutter or barrier to open, do not write "The door is open."; describe the way directly in front of the robot and require it to be completely open, for example "The way directly in front of the robot is open: no door panel blocks any part of it.".
- A target is a short noun phrase for the spot on the floor where the robot should stop, so that it can be pointed at in the image. To go into a space, use the center of its floor, for example "center of the floor of the room beyond the open doorway". To leave a space or to pass through something, use a spot about 1 m beyond the opening and say so, for example "floor about 1 m beyond the open doorway", so that the whole robot ends up clear of the opening.

Example (a different task, to show the wording).
Instructions, in order:
1. ゲートが開いたら通り抜けて
2. 通り抜けたら左に90度回って
Reply:
{"steps": [{"instruction": "ゲートが開いたら通り抜けて", "when": {"type": "when_true", "condition": "The way directly in front of the robot is open: no part of the gate blocks it."}, "do": {"skill": "go_to_visual_target", "target": "ground about 1 m beyond the open gate"}, "timeout_s": 120}, {"instruction": "通り抜けたら左に90度回って", "when": {"type": "now"}, "do": {"skill": "rotate", "angle_deg": 90}, "timeout_s": 30}]}

Reply with JSON only: {"steps": [...]}"""


@dataclass
class PlanningResult:
    plan: Optional[Plan]
    text: str = ""
    latency_s: float = 0.0
    error: str = ""
    attempts: int = 0


def split_instructions(text: str) -> List[str]:
    """入力欄の文字列を、1行1指示のリストにする."""
    return [line.strip() for line in text.replace("\r", "").split("\n") if line.strip()]


def make_plan(vlm: VlmClient, instructions: Sequence[str], jpeg: Optional[bytes] = None,
              limits: PlanLimits = DEFAULT_LIMITS, thinking: bool = False,
              max_tokens: int = 1200) -> PlanningResult:
    """指示文のリストから計画を作る. 検査に通らなければ、理由を添えて1回だけ作り直させる."""
    if not instructions:
        return PlanningResult(None, error="指示文が空です")
    listing = "\n".join(f"{i + 1}. {s}" for i, s in enumerate(instructions))
    user: List[Dict[str, Any]] = []
    if jpeg:
        user.append(image_part(jpeg))
        user.append({"type": "text", "text": "The image is the robot's current camera view.\n"
                                             f"Instructions, in order:\n{listing}"})
    else:
        user.append({"type": "text", "text": f"Instructions, in order:\n{listing}"})
    messages: List[Dict[str, Any]] = [
        {"role": "system", "content": PLANNER_SYSTEM},
        {"role": "user", "content": user},
    ]
    total_latency = 0.0
    text = ""
    error = ""
    for attempt in (1, 2):
        reply = vlm.chat(messages, max_tokens=max_tokens, schema_name="task_plan",
                         schema=PLAN_JSON_SCHEMA, thinking=thinking)
        total_latency += reply.latency_s
        text = reply.text
        data = extract_json(text)
        try:
            if data is None:
                raise PlanError("返答に JSON がありません")
            plan = parse_plan(data, limits)
            if len(plan.steps) != len(instructions):
                raise PlanError(f"手順の数（{len(plan.steps)}）が指示の数（{len(instructions)}）と合いません")
            return PlanningResult(plan, text=text, latency_s=total_latency, attempts=attempt)
        except PlanError as e:
            error = str(e)
            messages.append({"role": "assistant", "content": text})
            messages.append({"role": "user", "content": [{
                "type": "text",
                "text": f"The plan was rejected by the validator: {error}\n"
                        "Fix the problem and reply with the corrected JSON only."}]})
    return PlanningResult(None, text=text, latency_s=total_latency, error=error, attempts=2)
