"""VLM への2種類の問い: 条件の判定（yes / no）と、指差し（画像の上の点）.

どちらも最新の画像1枚だけを使う. エレベーターに固有の知識は持たない.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from .events import EventLog
from .robot_io import Frame, Marker, RobotIO, encode_jpeg
from .vlm_client import VlmClient, VlmError, image_part, parse_point, parse_yes_no, yes_probability

# 判定のシステムプロンプト. 2種類を用意し、設定（perception.check_prompt）で選ぶ.
#   basic:  短い. 条件文をそのまま画像と照らす.
#   guided: 「開いている」という条件文の見方を足したもの. 近すぎて開いた扉が画面の外に出ているときや、
#           扉が途中まで開いているときの答えをそろえるため（エレベーターに限らない、開口部一般の説明）.
CHECK_SYSTEM_BASIC = (
    "You are the visual perception module of a mobile robot. You receive one image from the robot's "
    "forward-facing camera and one statement about the scene. Decide whether the statement is true in "
    "this image, judging only from what is visible. "
    'Reply with JSON only: {"answer": "yes"} if the statement is true, {"answer": "no"} otherwise.'
)
CHECK_SYSTEM_GUIDED = (
    "You are the visual perception module of a mobile robot. You receive one image from the robot's "
    "forward-facing camera and one statement about the scene. The camera is mounted low and may be very "
    "close to what is in front of it. Decide whether the statement is true in this image, judging only "
    "from what is visible.\n"
    "When the statement says that something is open (a door, gate, shutter, barrier, or the way ahead), "
    "judge it like this:\n"
    "- It is open only when the way through is clear across its whole width, so that the floor and the "
    "space beyond are visible. The camera may be so close that the open door itself is outside the "
    "picture; an unobstructed view of the floor and space beyond still counts as open. Doors that are "
    "visible farther away do not matter.\n"
    "- If door panels or any other part of the barrier still cover part of the opening, it is only "
    "partly open, and that does not count as open.\n"
    "- If a flat surface such as door panels or a wall fills the view straight ahead, it is closed.\n"
    'Reply with JSON only: {"answer": "yes"} if the statement is true, {"answer": "no"} otherwise.'
)
CHECK_SYSTEMS: Dict[str, str] = {"basic": CHECK_SYSTEM_BASIC, "guided": CHECK_SYSTEM_GUIDED}
# 測った結果（log/20261111_144828、Qwen3.5-9B）: basic の方がよく合った（既定の条件文で 31/35、guided は 26/35）
CHECK_SYSTEM = CHECK_SYSTEM_BASIC
CHECK_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "properties": {"answer": {"type": "string", "enum": ["yes", "no"]}},
    "required": ["answer"],
    "additionalProperties": False,
}

POINT_SYSTEM = (
    "You are the visual perception module of a wheeled mobile robot. You receive one image from the "
    "robot's forward-facing camera. Point at the requested place with a single point on the floor "
    "surface where the robot could stand. When the place is described as a distance beyond something "
    "(for example 'about 1 m beyond the doorway'), point at the floor that far beyond it, not at the "
    "nearest visible floor. {coords} "
    'Reply with JSON only, like this: {{"point_2d": [x, y]}}'
)
POINT_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "properties": {"point_2d": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2}},
    "required": ["point_2d"],
    "additionalProperties": False,
}
MARKER_STATEMENT = ("The red circle marker is drawn on the floor (ground surface), "
                    "not on a wall, a door, a mirror or any object.")


@dataclass
class PerceptionConfig:
    jpeg_quality: int = 90
    max_image_age_s: float = 1.0
    use_logprobs: bool = False
    # 指差しの座標の約束. Qwen 系は画像サイズに対する 0〜1000 の相対座標、順序は x, y.
    # point_scale を 0 にすると画素の値そのままとして扱う. Gemma 系は順序が y, x.
    point_scale: float = 1000.0
    point_order: str = "xy"
    check_prompt: str = "basic"       # 判定のシステムプロンプト: basic / guided（上の説明を参照）
    check_system: str = ""            # 空でなければ、check_prompt の代わりにこの文をそのまま使う
    point_system: str = POINT_SYSTEM

    def check_system_text(self) -> str:
        return self.check_system or CHECK_SYSTEMS.get(self.check_prompt, CHECK_SYSTEM_BASIC)


@dataclass
class CheckResult:
    answer: Optional[bool]            # None は判定できなかった（通信の失敗、古い画像など）
    stamp: float = 0.0                # 使った画像の撮影時刻
    latency_s: float = 0.0
    p_yes: Optional[float] = None
    text: str = ""
    error: str = ""
    image: str = ""                   # 保存した画像の相対パス


@dataclass
class PointResult:
    ok: bool
    u: float = 0.0                    # 画素
    v: float = 0.0
    stamp: float = 0.0
    latency_s: float = 0.0
    text: str = ""
    error: str = ""
    image: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)


class Perception:
    def __init__(self, robot: RobotIO, vlm: VlmClient, cfg: PerceptionConfig, events: EventLog):
        self.robot = robot
        self.vlm = vlm
        self.cfg = cfg
        self.events = events
        self.last_check: Optional[CheckResult] = None
        self.last_point: Optional[PointResult] = None

    # -- 画像の取得 ---------------------------------------------------------
    def _fresh_frame(self) -> Optional[Frame]:
        frame = self.robot.get_frame()
        if frame is None:
            return None
        if self.robot.now() - frame.stamp > self.cfg.max_image_age_s:
            return None
        return frame

    # -- 条件の判定 ---------------------------------------------------------
    def check(self, condition: str, frame: Optional[Frame] = None,
              marker: Optional[Marker] = None, purpose: str = "check") -> CheckResult:
        """条件文が画像の中で成り立っているかを yes / no で得る."""
        if frame is None:
            frame = self._fresh_frame()
        if frame is None:
            res = CheckResult(None, error="新しいカメラ画像がありません")
            self.events.emit("check_error", purpose=purpose, condition=condition, error=res.error)
            self.last_check = res
            return res
        ctx = self.events.context()
        jpeg = encode_jpeg(frame, self.cfg.jpeg_quality, markers=[marker] if marker else ())
        image_path = self.events.save_image(purpose, jpeg)
        messages = [
            {"role": "system", "content": self.cfg.check_system_text()},
            {"role": "user", "content": [
                image_part(jpeg),
                {"type": "text", "text": f'Statement: "{condition}"\nIs the statement true in this image?'},
            ]},
        ]
        try:
            reply = self.vlm.chat(messages, max_tokens=16, schema_name="condition_check",
                                  schema=CHECK_SCHEMA, logprobs=self.cfg.use_logprobs)
        except VlmError as e:
            res = CheckResult(None, stamp=frame.stamp, error=str(e), image=image_path)
            self.events.emit("check_error", purpose=purpose, condition=condition, error=res.error)
            self.last_check = res
            return res
        answer = parse_yes_no(reply.text)
        res = CheckResult(answer, stamp=frame.stamp, latency_s=reply.latency_s,
                          p_yes=yes_probability(reply.logprobs), text=reply.text, image=image_path,
                          error="" if answer is not None else "返答から yes / no を読めません")
        self.events.emit("check", purpose=purpose, condition=condition,
                         answer=None if answer is None else ("yes" if answer else "no"),
                         p_yes=res.p_yes, latency_s=round(res.latency_s, 3), stamp=round(frame.stamp, 3),
                         text=reply.text[:200], image=image_path, ctx=ctx)
        self.last_check = res
        return res

    # -- 指差し -------------------------------------------------------------
    def point(self, target: str, frame: Optional[Frame] = None, purpose: str = "point") -> PointResult:
        """対象の記述に当たる床の上の点を、画素の座標で得る."""
        if frame is None:
            frame = self._fresh_frame()
        if frame is None:
            res = PointResult(False, error="新しいカメラ画像がありません")
            self.events.emit("point_error", purpose=purpose, target=target, error=res.error)
            self.last_point = res
            return res
        ctx = self.events.context()
        jpeg = encode_jpeg(frame, self.cfg.jpeg_quality)
        image_path = self.events.save_image(purpose, jpeg)
        if self.cfg.point_scale > 0:
            coords = (f"Coordinates are relative to the image size and range from 0 to {self.cfg.point_scale:g} "
                      "(x from left to right, y from top to bottom).")
        else:
            coords = (f"Coordinates are in pixels of the {frame.width}x{frame.height} image "
                      "(x from left to right, y from top to bottom).")
        if self.cfg.point_order == "yx":
            coords += " Give the point as [y, x]."
        messages = [
            {"role": "system", "content": self.cfg.point_system.format(coords=coords)},
            {"role": "user", "content": [
                image_part(jpeg),
                {"type": "text", "text": f"Locate this place with one point on the floor: {target}. "
                                         "Output the point coordinates in JSON format."},
            ]},
        ]
        try:
            reply = self.vlm.chat(messages, max_tokens=48, schema_name="point_target", schema=POINT_SCHEMA)
        except VlmError as e:
            res = PointResult(False, stamp=frame.stamp, error=str(e), image=image_path)
            self.events.emit("point_error", purpose=purpose, target=target, error=res.error)
            self.last_point = res
            return res
        raw = parse_point(reply.text)
        if raw is None:
            res = PointResult(False, stamp=frame.stamp, latency_s=reply.latency_s, text=reply.text,
                              error="返答から点の座標を読めません", image=image_path)
        else:
            a, b = raw
            x, y = (b, a) if self.cfg.point_order == "yx" else (a, b)
            if self.cfg.point_scale > 0:
                u = x / self.cfg.point_scale * frame.width
                v = y / self.cfg.point_scale * frame.height
            else:
                u, v = x, y
            res = PointResult(True, u=u, v=v, stamp=frame.stamp, latency_s=reply.latency_s,
                              text=reply.text, image=image_path, extra={"raw": [a, b]})
        self.events.emit("point", purpose=purpose, target=target, ok=res.ok,
                         u=round(res.u, 1), v=round(res.v, 1), latency_s=round(res.latency_s, 3),
                         stamp=round(frame.stamp, 3), text=reply.text[:200], error=res.error, image=image_path,
                         ctx=ctx)
        self.last_point = res
        return res

    def verify_floor(self, frame: Frame, u: float, v: float) -> CheckResult:
        """指差した点が実際の床の上にあるかを、印を重ねた画像で VLM に確かめる（鏡や光沢面への対策）."""
        return self.check(MARKER_STATEMENT, frame=frame, marker=Marker(u, v), purpose="verify_point")
