"""ダミーのカメラ. 擬似世界の真値（ロボットの場所と向き、扉の開き、かごの階）に合う画像を、
正解つきの画像の組（elevator_vlm/samples/、Gazebo で撮ったもの）から選ぶ. ROS に依存しない.

画像は、決まった位置から撮ったものしかない. 次の位置と向きのときに、それらしい画像になる.
  * 乗場の待機位置（敷居の 1 m 手前）から扉を向いている（hall）
  * かごの中央から扉を向いている（car）. 扉の近く（敷居の約 0.55 m 奥）なら car_near
合う画像がない位置と向き（かごの奥の壁を向いている、降りた後の乗場、など）では、
「この向きの画像はありません」と書いた画像を出す. この間は VLM に問わない手順なので、通しには困らない.

ロボットが動いても、同じ場所と状態なら同じ画像のままになる. そのため、走行中に指差しをやり直す
設定（skills.repoint_period_s）は 0 にして使う.
"""
from __future__ import annotations

import io
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .world_params import RobotParams, Truth, WorldParams, door_label, door_ratio_at_robot, robot_in_car, to_robot_frame

FACING_TOL_DEG = 35.0          # 扉の方を向いているとみなす向きのずれ
CAR_NEAR_DOOR_AHEAD_M = 0.64   # かごの中で、敷居までがこれより近ければ car_near の画像を使う


@dataclass(frozen=True)
class Sample:
    kind: str           # check（判定の画像）か point（指差しの画像）
    place: str          # hall、car、car_near
    floor: int          # 乗場の階. かごの中なら着床している階（0 は昇降中）
    door_ratio: float   # 扉の開き具合（0 = 全閉、1 = 全開）
    file: str
    jpeg: bytes


@dataclass(frozen=True)
class View:
    """今のロボットから見えるはずのもの（画像を選ぶための分類）."""

    place: str                  # hall、car、car_near
    floor: int
    door_ratio: float
    facing_door: bool

    def describe(self) -> str:
        where = {"hall": "乗場", "car": "かごの中央", "car_near": "かごの扉の近く"}.get(self.place, self.place)
        return (f"{where}、{self.floor} 階、扉の開き {self.door_ratio:.2f}、"
                f"{'扉を向いている' if self.facing_door else '扉を向いていない'}")


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def classify(wp: WorldParams, truth: Truth) -> View:
    """真値から、今のロボットの見え方を分類する."""
    ratio = door_ratio_at_robot(wp, truth)
    if robot_in_car(wp, truth):
        floor = wp.nearest_floor(truth.lift_pos, 0.02) or 0
        facing = abs(_wrap(truth.robot_yaw - math.pi)) < math.radians(FACING_TOL_DEG)
        door_ahead = to_robot_frame(truth, 0.0, 0.0)[0]
        place = "car_near" if facing and door_ahead < CAR_NEAR_DOOR_AHEAD_M else "car"
        return View(place, floor, ratio, facing)
    floor = wp.nearest_floor(truth.robot_z) or 0
    facing = abs(_wrap(truth.robot_yaw)) < math.radians(FACING_TOL_DEG)
    return View("hall", floor, ratio, facing)


def placeholder_jpeg(width: int, height: int, lines: List[str]) -> bytes:
    """合う画像がないときに出す画像（灰色に文字）."""
    from PIL import Image, ImageDraw  # 遅延 import

    img = Image.new("RGB", (width, height), (70, 74, 80))
    draw = ImageDraw.Draw(img)
    y = height // 2 - 10 * len(lines)
    for text in lines:
        draw.text((24, y), text, fill=(235, 235, 235))
        y += 20
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


class SampleCamera:
    """正解つきの画像の組から、今の見え方に合う画像を選ぶ."""

    def __init__(self, samples_dir: str, wp: Optional[WorldParams] = None, rp: Optional[RobotParams] = None):
        self.wp = wp or WorldParams()
        self.rp = rp or RobotParams()
        self.samples_dir = samples_dir
        with open(os.path.join(samples_dir, "labels.json"), "r", encoding="utf-8") as f:
            labels = json.load(f)
        self.samples: List[Sample] = []
        for item in labels:
            path = os.path.join(samples_dir, item["file"])
            if not os.path.isfile(path):
                continue
            with open(path, "rb") as f:
                jpeg = f.read()
            # 指差しの画像は、扉が開いているときに撮ったもの
            ratio = float(item.get("door_ratio", 1.0 if item.get("kind") == "point" else 0.0))
            self.samples.append(Sample(str(item.get("kind", "check")), str(item["place"]), int(item["floor"]),
                                       ratio, str(item["file"]), jpeg))
        if not self.samples:
            raise ValueError(f"画像がありません: {samples_dir}")
        self._placeholders: Dict[str, bytes] = {}

    def _placeholder(self, view: View) -> Tuple[str, bytes]:
        if view.place == "hall":
            reason = f"hall (floor {view.floor}), not facing the elevator door"
        else:
            reason = "in the car, facing the back wall" if not view.facing_door else "in the car"
        key = reason
        if key not in self._placeholders:
            self._placeholders[key] = placeholder_jpeg(
                self.rp.image_width, self.rp.image_height,
                ["DUMMY CAMERA: no sample image for this view", f"({reason})"])
        return "placeholder", self._placeholders[key]

    def select(self, truth: Truth) -> Tuple[str, bytes, View]:
        """今の見え方に合う画像の (名前, JPEG, 分類). 合う画像がなければ名前は placeholder."""
        view = classify(self.wp, truth)
        if not view.facing_door:
            name, jpeg = self._placeholder(view)
            return name, jpeg, view
        same_place = [s for s in self.samples if s.place == view.place]
        if not same_place and view.place == "car_near":
            same_place = [s for s in self.samples if s.place == "car"]
        if not same_place:
            name, jpeg = self._placeholder(view)
            return name, jpeg, view
        # 全閉・途中・全開の区別を先にそろえる（閉まり始めた扉に全開の画像を出さないため）.
        # 同じ階に同じ区別の画像がなければ、別の階のものを使う（閉じた扉は、階によらず同じに見える）
        label = door_label(view.door_ratio)
        same_label = [s for s in same_place if door_label(s.door_ratio) == label]
        pool = ([s for s in same_label if s.floor == view.floor] or same_label
                or [s for s in same_place if s.floor == view.floor] or same_place)
        best = min(pool, key=lambda s: (abs(s.door_ratio - view.door_ratio), s.floor != view.floor,
                                        s.kind != "check", s.file))
        return best.file, best.jpeg, view
