"""実写のフレームに付けた正解（atc_frames/labels.json）から、モック VLM の答えを作る. ROS に依存しない.

道のベンチ（bench_path_node）では、ロボットが走った道のりに合わせて映像のフレームを流す. モック VLM は
画像を見ないので、今のフレームの正解から答える:
  * 道の指差し: 正解の「走れる範囲」の中で、避けるものから離れた、少し先の点. 正解のないフレームでは、
    いちばん近い番号の正解のあるフレームの点.
  * 止まる条件: 正解の文のうち、yes のフレームの番号の範囲（最初から最後まで）に今のフレームが入れば yes.
    正解にない文は no.
"""
from __future__ import annotations

import io
import json
import os
import re
from typing import Any, Dict, List, Optional, Tuple

from elevator_task.core.geometry import point_in_polygon, polygon_edge_distance

_NUMBER = re.compile(r"(\d+)")


def frame_number(name: str) -> Optional[int]:
    """ファイル名の最後の数字（frame_00243.jpg → 243）."""
    found = _NUMBER.findall(name)
    return int(found[-1]) if found else None


def aim_point(label: Dict[str, Any], step: float = 20.0) -> Optional[Tuple[float, float]]:
    """走れる範囲の中の、避けるものから離れた、少し先の点（0〜1000 の相対座標）."""
    walkable = label["walkable"]
    avoid = label.get("avoid", [])
    xs = [p[0] for p in walkable]
    ys = [p[1] for p in walkable]
    top, bottom = min(ys), max(ys)
    y_goal = top + 0.35 * (bottom - top)          # 範囲の上（遠く）から 35% あたり: 数 m 先
    best: Optional[Tuple[float, Tuple[float, float]]] = None
    y = top
    while y <= bottom:
        x = min(xs)
        while x <= max(xs):
            if point_in_polygon(walkable, x, y) and not any(point_in_polygon(a, x, y) for a in avoid):
                clear = min([polygon_edge_distance(walkable, x, y)]
                            + [polygon_edge_distance(a, x, y) for a in avoid])
                score = min(clear, 120.0) - 0.4 * abs(y - y_goal)
                if best is None or score > best[0]:
                    best = (score, (x, y))
            x += step
        y += step
    return None if best is None else best[1]


class FrameLabels:
    def __init__(self, data: Dict[str, Any]):
        self.path = str(data.get("path", "the path ahead"))
        self.points: Dict[int, Tuple[float, float]] = {}
        for name, label in (data.get("frames") or {}).items():
            number = frame_number(name)
            point = aim_point(label)
            if number is not None and point is not None:
                self.points[number] = point
        self.statements: Dict[str, Tuple[int, int]] = {}
        for item in data.get("statements") or []:
            numbers = [n for n in (frame_number(x) for x in item.get("yes", [])) if n is not None]
            if numbers:
                self.statements[str(item["text"]).strip()] = (min(numbers), max(numbers))

    @classmethod
    def load(cls, path: str) -> "FrameLabels":
        with open(path, encoding="utf-8") as f:
            return cls(json.load(f))

    def path_point(self, number: Optional[int]) -> Optional[Tuple[float, float]]:
        """そのフレームでの道の指差しの答え（正解のあるいちばん近いフレームの点）."""
        if number is None or not self.points:
            return None
        nearest = min(self.points, key=lambda n: (abs(n - number), n))
        return self.points[nearest]

    def statement(self, text: str, number: Optional[int]) -> bool:
        span = self.statements.get(text.strip())
        return span is not None and number is not None and span[0] <= number <= span[1]

    def names(self) -> List[str]:
        return sorted(self.statements)


def load_frames(frames_dir: str, first: int, last: int, width: int) -> List[Tuple[str, int, bytes, int, int]]:
    """映像のフレームを読み、幅 width まで縮めた JPEG にする. (名前, 番号, JPEG, 幅, 高さ) の並び（番号の順）.

    first〜last の番号のものだけ（last が 0 なら最後まで）.
    """
    from PIL import Image

    out = []
    names = [n for n in os.listdir(frames_dir) if n.lower().endswith((".jpg", ".jpeg", ".png"))]
    for name in sorted(names, key=lambda n: (frame_number(n) or 0, n)):
        number = frame_number(name) or 0
        if number < first or (last > 0 and number > last):
            continue
        img = Image.open(os.path.join(frames_dir, name)).convert("RGB")
        if width > 0 and img.width > width:
            img = img.resize((width, round(img.height * width / img.width)), Image.BILINEAR)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        out.append((name, number, buf.getvalue(), img.width, img.height))
    return out
