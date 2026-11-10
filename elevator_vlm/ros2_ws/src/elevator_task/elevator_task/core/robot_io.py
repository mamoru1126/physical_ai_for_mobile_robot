"""ロボット本体との接点（カメラ画像、オドメトリ、速度指令、時計）の型.

実行系はこのインターフェースだけに依存する. ROS 2 のノード（task_node.py）と、
テスト用の擬似世界（test/fake_world.py）がそれぞれ実装する.
"""
from __future__ import annotations

import io
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .geometry import CameraModel, Pose2D


@dataclass(frozen=True)
class Frame:
    """カメラ画像1枚. rgb は行優先の RGB8（幅×高さ×3 バイト）."""

    stamp: float
    width: int
    height: int
    rgb: bytes


@dataclass(frozen=True)
class Marker:
    """画像に重ねる印（指差した点の確認や画面表示に使う）."""

    u: float
    v: float
    color: Tuple[int, int, int] = (255, 0, 0)
    radius: int = 9
    label: str = ""


def encode_jpeg(frame: Frame, quality: int = 90, markers: Sequence[Marker] = (),
                lines: Sequence[str] = ()) -> bytes:
    """画像を JPEG にする. 印と文字を重ねることもできる."""
    from PIL import Image, ImageDraw  # 遅延 import（幾何だけ使うテストでは不要）

    img = Image.frombytes("RGB", (frame.width, frame.height), frame.rgb)
    if markers or lines:
        draw = ImageDraw.Draw(img)
        for m in markers:
            r = m.radius
            draw.ellipse([m.u - r - 2, m.v - r - 2, m.u + r + 2, m.v + r + 2], outline=(255, 255, 255), width=2)
            draw.ellipse([m.u - r, m.v - r, m.u + r, m.v + r], outline=m.color, width=3)
            draw.line([m.u - r, m.v, m.u + r, m.v], fill=m.color, width=1)
            draw.line([m.u, m.v - r, m.u, m.v + r], fill=m.color, width=1)
            if m.label:
                draw.text((m.u + r + 4, m.v - r), m.label, fill=m.color)
        y = 4
        for text in lines:
            draw.rectangle([2, y - 1, 6 + 6 * len(text), y + 11], fill=(0, 0, 0))
            draw.text((4, y), text, fill=(255, 255, 255))
            y += 14
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=int(quality))
    return buf.getvalue()


class RobotIO:
    """ロボット本体との接点. 実装側ですべて上書きする."""

    def now(self) -> float:
        """現在時刻 [s]. シミュレーションではシミュレーション時刻."""
        raise NotImplementedError

    def sleep(self, seconds: float) -> None:
        """seconds 秒待つ."""
        raise NotImplementedError

    def get_frame(self) -> Optional[Frame]:
        """最新のカメラ画像. まだ無ければ None."""
        raise NotImplementedError

    def get_pose(self) -> Optional[Pose2D]:
        """最新のオドメトリ. まだ無ければ None."""
        raise NotImplementedError

    def pose_at(self, stamp: float) -> Optional[Pose2D]:
        """時刻 stamp のオドメトリ（履歴から補間）."""
        raise NotImplementedError

    def get_camera(self) -> Optional[CameraModel]:
        """カメラのモデル. カメラ情報をまだ受け取っていなければ None."""
        raise NotImplementedError

    def send_cmd(self, linear: float, angular: float) -> None:
        """速度指令（並進 [m/s]、旋回 [rad/s]）を出す."""
        raise NotImplementedError

    def status(self) -> List[str]:
        """画面表示用の状態（不足している入力など）."""
        return []
