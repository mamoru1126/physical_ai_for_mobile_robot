"""道のある擬似世界（道に沿って走る試験用）. ROS に依存しない.

平面の上に道（中心線の折れ線と幅）を置き、差動二輪のロボットを走らせる. モック VLM が答えるための
真値（道の中心線の少し先の点が、カメラのどこに写るか. 道の終わりの目印が前にあるか）も出す.

使うところ:
  * テスト（test/fake_road.py）: 擬似時刻で、道に沿って走るスキルを通しで確かめる.
  * 道のベンチ（bench_path_node.py）: 実時間で回し、/odom を出す（画像は ATC の映像のフレーム）.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from elevator_task.core.geometry import CameraModel, ground_to_pixel, wrap_angle


@dataclass
class RoadParams:
    # 道の中心線（ワールド座標の折れ線 [m]）. ロボットは最初の点から、最初の区間の向きで走り出す
    points: List[Tuple[float, float]] = field(default_factory=lambda: [(0.0, 0.0), (60.0, 0.0)])
    width_m: float = 2.0
    lookahead_m: float = 2.5          # モック VLM が指す点: 中心線のこれだけ先
    landmark_s: Optional[float] = None  # 道の終わりの目印（中心線に沿った位置 [m]）. None なら置かない
    landmark_radius_m: float = 2.0    # 目印がこれより近く、前にあれば「目の前にある」


def curve_points(straight1: float = 8.0, radius: float = 4.0, angle_deg: float = 90.0, straight2: float = 10.0,
                 step_deg: float = 5.0) -> List[Tuple[float, float]]:
    """直線 → 円弧（左へ曲がる）→ 直線 の中心線."""
    pts = [(0.0, 0.0), (straight1, 0.0)]
    cx, cy = straight1, radius
    n = max(1, int(round(angle_deg / step_deg)))
    for i in range(1, n + 1):
        a = math.radians(angle_deg) * i / n
        pts.append((cx + radius * math.sin(a), cy - radius * math.cos(a)))
    end = math.radians(angle_deg)
    x, y = pts[-1]
    pts.append((x + straight2 * math.cos(end), y + straight2 * math.sin(end)))
    return pts


class Polyline:
    """折れ線の上の位置の計算（中心線に沿った位置 s と、横のずれ）."""

    def __init__(self, points: List[Tuple[float, float]]):
        if len(points) < 2:
            raise ValueError("道の中心線には 2 点以上が要ります")
        self.points = [(float(x), float(y)) for x, y in points]
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.cum[-1]

    def project(self, x: float, y: float) -> Tuple[float, float]:
        """点を中心線に落とした位置 s と、横のずれ（進む向きの左が正）."""
        best = (float("inf"), 0.0, 0.0)
        for i, ((x0, y0), (x1, y1)) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = x1 - x0, y1 - y0
            seg = math.hypot(dx, dy)
            if seg <= 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (seg * seg)))
            px, py = x0 + t * dx, y0 + t * dy
            d = math.hypot(x - px, y - py)
            if d < best[0]:
                side = (dx * (y - y0) - dy * (x - x0)) / seg
                best = (d, self.cum[i] + t * seg, side)
        return best[1], best[2]

    def at(self, s: float) -> Tuple[float, float, float]:
        """中心線に沿った位置 s の点と、そこでの向き."""
        s = max(0.0, min(self.length, s))
        for i in range(len(self.points) - 1):
            if s <= self.cum[i + 1] or i == len(self.points) - 2:
                (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
                seg = self.cum[i + 1] - self.cum[i]
                t = 0.0 if seg <= 1e-9 else (s - self.cum[i]) / seg
                return x0 + t * (x1 - x0), y0 + t * (y1 - y0), math.atan2(y1 - y0, x1 - x0)
        x, y = self.points[-1]
        return x, y, 0.0


class RoadWorld:
    """道のある平面とロボット. 時刻は step(dt) を呼んだ分だけ進む."""

    def __init__(self, road: Optional[RoadParams] = None, cmd_timeout_s: float = 0.5, bounded: bool = True,
                 record: bool = True):
        self.road = road or RoadParams()
        self.record = record            # 速度指令と道のりの記録を残すか（試験用. 長く動かすベンチでは残さない）
        self.line = Polyline(self.road.points)
        self.cmd_timeout_s = cmd_timeout_s
        self.bounded = bounded          # False なら道の外にも出られる（記録だけする）
        self._lock = threading.RLock()
        self.t = 0.0
        x, y, yaw = self.line.at(0.0)
        self.x, self.y, self.yaw = x, y, yaw
        self._origin = (x, y, yaw)
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_at = -1.0
        self.ox = self.oy = self.oyaw = 0.0
        self.travelled = 0.0            # 真の道のり
        self.max_lateral = 0.0          # 中心線からの横のずれの最大
        self.left_road = False
        self.cmd_log: List[Tuple[float, float, float]] = []
        self.trace: List[Tuple[float, float]] = []      # (時刻, 道のり)

    # -- 真値 ---------------------------------------------------------------
    def odom(self) -> Tuple[float, float, float]:
        with self._lock:
            return self.ox, self.oy, self.oyaw

    def velocity(self) -> Tuple[float, float]:
        with self._lock:
            return self.v, self.w

    def along(self) -> Tuple[float, float]:
        """中心線に沿った位置と、横のずれ."""
        with self._lock:
            return self.line.project(self.x, self.y)

    def to_robot(self, wx: float, wy: float) -> Tuple[float, float]:
        with self._lock:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            dx, dy = wx - self.x, wy - self.y
            return c * dx + s * dy, -s * dx + c * dy

    def path_pixel(self, cam: CameraModel) -> Optional[Tuple[float, float]]:
        """モック VLM の答え: 中心線の lookahead_m 先の点が写る画素. 画像の外や後ろなら、写る範囲に寄せる."""
        s, _lat = self.along()
        gx, gy, _ = self.line.at(s + self.road.lookahead_m)
        rx, ry = self.to_robot(gx, gy)
        if rx < 0.3:
            return None                 # 道の終わりを過ぎた、か、後ろ
        uv = ground_to_pixel(cam, rx, ry)
        if uv is None:
            return None
        u = max(2.0, min(cam.width - 3.0, uv[0]))
        v = max(cam.horizon_v() + 4.0, min(cam.height - 3.0, uv[1]))
        return u, v

    def landmark_ahead(self) -> bool:
        """道の終わりの目印が、前の近く（landmark_radius_m より近く）にあるか."""
        if self.road.landmark_s is None:
            return False
        lx, ly, _ = self.line.at(self.road.landmark_s)
        rx, ry = self.to_robot(lx, ly)
        return rx > 0.0 and math.hypot(rx, ry) < self.road.landmark_radius_m

    # -- 操作 ---------------------------------------------------------------
    def set_cmd(self, linear: float, angular: float) -> None:
        with self._lock:
            self.cmd = (float(linear), float(angular))
            self.cmd_at = self.t
            if self.record:
                self.cmd_log.append((self.t, float(linear), float(angular)))

    def step(self, dt: float) -> None:
        with self._lock:
            self.t += dt
            cv, cw = self.cmd if self.t - self.cmd_at <= self.cmd_timeout_s else (0.0, 0.0)
            self.v += max(-1.5 * dt, min(1.5 * dt, cv - self.v))
            self.w += max(-4.0 * dt, min(4.0 * dt, cw - self.w))
            nx = self.x + self.v * math.cos(self.yaw) * dt
            ny = self.y + self.v * math.sin(self.yaw) * dt
            _s, lat = self.line.project(nx, ny)
            if abs(lat) > self.road.width_m / 2.0:
                self.left_road = True
                if self.bounded:        # 道の端（縁石）に当たって進めない
                    nx, ny = self.x, self.y
                    self.v = 0.0
            self.max_lateral = max(self.max_lateral, abs(lat))
            self.travelled += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
            self.yaw = wrap_angle(self.yaw + self.w * dt)
            self.ox += self.v * math.cos(self.oyaw) * dt
            self.oy += self.v * math.sin(self.oyaw) * dt
            self.oyaw = wrap_angle(self.oyaw + self.w * dt)
            if self.record:
                self.trace.append((self.t, self.travelled))
