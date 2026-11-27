"""計画（手順の列）の形式と検査.

1手順は「いつ (when)」「何を (do)」「時間切れ (timeout_s)」の3つで表す.
検討資料の 5.1 節の JSON をそのまま読み書きできる.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

WHEN_NOW = "now"
WHEN_TRUE = "when_true"
WHEN_AFTER_FALSE_THEN_TRUE = "after_false_then_true"
WHEN_TYPES = (WHEN_NOW, WHEN_TRUE, WHEN_AFTER_FALSE_THEN_TRUE)

SKILL_ROTATE = "rotate"
SKILL_MOVE_STRAIGHT = "move_straight"
SKILL_GO_TO_VISUAL_TARGET = "go_to_visual_target"
SKILL_WAIT = "wait"
SKILL_FOLLOW_PATH = "follow_path"
SKILLS = (SKILL_ROTATE, SKILL_MOVE_STRAIGHT, SKILL_GO_TO_VISUAL_TARGET, SKILL_WAIT, SKILL_FOLLOW_PATH)


class PlanError(ValueError):
    """計画が形式か範囲の検査に通らなかった."""


@dataclass(frozen=True)
class PlanLimits:
    max_distance_m: float = 3.0
    max_angle_deg: float = 360.0
    max_wait_s: float = 600.0
    max_timeout_s: float = 1800.0
    max_steps: int = 12
    # 道に沿って走る（follow_path）1手順の距離の上限 [m]. distance_m が 0（距離を決めない）でもここで止まる
    max_path_distance_m: float = 100.0
    # 距離も止まる条件も言わない「道に沿って走って」の距離 [m]. 計画を作るプロンプトに書いて VLM に使わせる
    default_path_distance_m: float = 10.0


@dataclass(frozen=True)
class When:
    type: str
    condition: str = ""

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {"type": self.type}
        if self.type != WHEN_NOW:
            d["condition"] = self.condition
        return d


@dataclass(frozen=True)
class Action:
    skill: str
    angle_deg: float = 0.0
    distance_m: float = 0.0           # move_straight: 進む距離. follow_path: 道に沿って走る距離（0 は決めない）
    target: str = ""
    seconds: float = 0.0
    path: str = ""                    # follow_path: 沿って走るもの（英語. 例 "the paved walkway"）
    stop_when: str = ""               # follow_path: 成り立ったら止まる条件（英語. 空なら使わない）

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {"skill": self.skill}
        if self.skill == SKILL_ROTATE:
            d["angle_deg"] = self.angle_deg
        elif self.skill == SKILL_MOVE_STRAIGHT:
            d["distance_m"] = self.distance_m
        elif self.skill == SKILL_GO_TO_VISUAL_TARGET:
            d["target"] = self.target
        elif self.skill == SKILL_WAIT:
            d["seconds"] = self.seconds
        elif self.skill == SKILL_FOLLOW_PATH:
            d["path"] = self.path
            d["distance_m"] = self.distance_m
            d["stop_when"] = self.stop_when
        return d


@dataclass(frozen=True)
class Step:
    instruction: str
    when: When
    do: Action
    timeout_s: float

    def to_dict(self) -> Dict[str, Any]:
        return {"instruction": self.instruction, "when": self.when.to_dict(),
                "do": self.do.to_dict(), "timeout_s": self.timeout_s}


@dataclass(frozen=True)
class Plan:
    steps: List[Step] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {"steps": [s.to_dict() for s in self.steps]}

    def to_json(self, indent: Optional[int] = 2) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=indent)


# VLM の構造化出力に渡す JSON スキーマ. スキルごと、「いつ」の種類ごとに、必要な項目を決めてある
# （サーバが出力をこの形に縛るので、「旋回なのに角度がない」といった計画は出てこない）.
# 数値の範囲は parse_plan で検査する.
def _variant(key: str, values: List[str], extra: Dict[str, Any]) -> Dict[str, Any]:
    props: Dict[str, Any] = {key: {"type": "string", "enum": values}}
    props.update(extra)
    return {"type": "object", "properties": props, "required": list(props), "additionalProperties": False}


PLAN_JSON_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "properties": {
        "steps": {
            "type": "array",
            "minItems": 1,
            "maxItems": 12,
            "items": {
                "type": "object",
                "properties": {
                    "instruction": {"type": "string"},
                    "when": {"anyOf": [
                        _variant("type", [WHEN_NOW], {}),
                        _variant("type", [WHEN_TRUE, WHEN_AFTER_FALSE_THEN_TRUE], {"condition": {"type": "string"}}),
                    ]},
                    "do": {"anyOf": [
                        _variant("skill", [SKILL_ROTATE], {"angle_deg": {"type": "number"}}),
                        _variant("skill", [SKILL_MOVE_STRAIGHT], {"distance_m": {"type": "number"}}),
                        _variant("skill", [SKILL_GO_TO_VISUAL_TARGET], {"target": {"type": "string"}}),
                        _variant("skill", [SKILL_WAIT], {"seconds": {"type": "number"}}),
                        _variant("skill", [SKILL_FOLLOW_PATH], {"path": {"type": "string"},
                                                                "distance_m": {"type": "number"},
                                                                "stop_when": {"type": "string"}}),
                    ]},
                    "timeout_s": {"type": "number"},
                },
                "required": ["instruction", "when", "do", "timeout_s"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["steps"],
    "additionalProperties": False,
}


def _number(obj: Dict[str, Any], key: str, where: str) -> float:
    if key not in obj:
        raise PlanError(f"{where}: '{key}' がありません")
    val = obj[key]
    if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(float(val)):
        raise PlanError(f"{where}: '{key}' は数値で指定してください（{val!r}）")
    return float(val)


def _text(obj: Dict[str, Any], key: str, where: str) -> str:
    val = obj.get(key)
    if not isinstance(val, str) or not val.strip():
        raise PlanError(f"{where}: '{key}' は空でない文字列で指定してください")
    return val.strip()


DEFAULT_LIMITS = PlanLimits()


def parse_plan(data: Any, limits: PlanLimits = DEFAULT_LIMITS) -> Plan:
    """dict か JSON 文字列から計画を作る. 形式と範囲が合わなければ PlanError."""
    if isinstance(data, (str, bytes)):
        try:
            data = json.loads(data)
        except json.JSONDecodeError as e:
            raise PlanError(f"JSON として読めません: {e}") from e
    if not isinstance(data, dict) or not isinstance(data.get("steps"), list):
        raise PlanError("計画には 'steps' の配列が必要です")
    raw_steps = data["steps"]
    if not raw_steps:
        raise PlanError("手順が1つもありません")
    if len(raw_steps) > limits.max_steps:
        raise PlanError(f"手順が多すぎます（{len(raw_steps)} > {limits.max_steps}）")

    steps: List[Step] = []
    for i, raw in enumerate(raw_steps):
        where = f"手順{i + 1}"
        if not isinstance(raw, dict):
            raise PlanError(f"{where}: オブジェクトではありません")
        instruction = raw.get("instruction", "")
        if not isinstance(instruction, str):
            raise PlanError(f"{where}: 'instruction' は文字列で指定してください")

        raw_when = raw.get("when")
        if not isinstance(raw_when, dict):
            raise PlanError(f"{where}: 'when' がありません")
        wtype = raw_when.get("type")
        if wtype not in WHEN_TYPES:
            raise PlanError(f"{where}: when.type は {list(WHEN_TYPES)} のどれかです（{wtype!r}）")
        condition = "" if wtype == WHEN_NOW else _text(raw_when, "condition", f"{where} when")
        when = When(wtype, condition)

        raw_do = raw.get("do")
        if not isinstance(raw_do, dict):
            raise PlanError(f"{where}: 'do' がありません")
        skill = raw_do.get("skill")
        if skill not in SKILLS:
            raise PlanError(f"{where}: do.skill は {list(SKILLS)} のどれかです（{skill!r}）")
        dwhere = f"{where} do"
        if skill == SKILL_ROTATE:
            angle = _number(raw_do, "angle_deg", dwhere)
            if abs(angle) > limits.max_angle_deg or abs(angle) < 1e-3:
                raise PlanError(f"{dwhere}: angle_deg は 0 以外で ±{limits.max_angle_deg:g} 度以内です（{angle:g}）")
            action = Action(skill, angle_deg=angle)
        elif skill == SKILL_MOVE_STRAIGHT:
            dist = _number(raw_do, "distance_m", dwhere)
            if abs(dist) > limits.max_distance_m or abs(dist) < 1e-3:
                raise PlanError(f"{dwhere}: distance_m は 0 以外で ±{limits.max_distance_m:g} m 以内です（{dist:g}）")
            action = Action(skill, distance_m=dist)
        elif skill == SKILL_GO_TO_VISUAL_TARGET:
            action = Action(skill, target=_text(raw_do, "target", dwhere))
        elif skill == SKILL_FOLLOW_PATH:
            dist = _number(raw_do, "distance_m", dwhere)
            if not 0.0 <= dist <= limits.max_path_distance_m:
                raise PlanError(f"{dwhere}: distance_m は 0（距離を決めない）から {limits.max_path_distance_m:g} m "
                                f"までです（{dist:g}）")
            stop_when = raw_do.get("stop_when", "")
            if not isinstance(stop_when, str):
                raise PlanError(f"{dwhere}: 'stop_when' は文字列で指定してください（使わないなら空の文字列）")
            action = Action(skill, distance_m=dist, path=_text(raw_do, "path", dwhere), stop_when=stop_when.strip())
        else:
            sec = _number(raw_do, "seconds", dwhere)
            if not 0.0 < sec <= limits.max_wait_s:
                raise PlanError(f"{dwhere}: seconds は 0 より大きく {limits.max_wait_s:g} 秒以内です（{sec:g}）")
            action = Action(skill, seconds=sec)

        timeout = _number(raw, "timeout_s", where)
        if not 0.0 < timeout <= limits.max_timeout_s:
            raise PlanError(f"{where}: timeout_s は 0 より大きく {limits.max_timeout_s:g} 秒以内です（{timeout:g}）")
        steps.append(Step(instruction, when, action, timeout))
    return Plan(steps)
