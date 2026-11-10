"""カメラの幾何（画素と床面の点の変換）と、オドメトリ座標の計算.

座標の約束:
  * ロボット座標: X は前方、Y は左、原点は駆動輪の中心の床面.
  * 画像座標: u は右、v は下 [画素].
  * カメラはロボットの前後 x_offset_m、床からの高さ height_m にあり、
    水平から pitch_rad だけ下を向く（下向きが正）.

検討資料の 5.3 節の式をそのまま実装している.
"""
from __future__ import annotations

import bisect
import math
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple


def wrap_angle(a: float) -> float:
    """角度を (-pi, pi] に丸める."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


@dataclass(frozen=True)
class Pose2D:
    """オドメトリ座標での位置と向き."""

    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CameraModel:
    """ピンホールカメラと取り付け位置."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    height_m: float
    pitch_rad: float
    x_offset_m: float = 0.0
    y_offset_m: float = 0.0

    def horizon_v(self) -> float:
        """地平線が写る v [画素]. これより下（v が大きい側）だけが床に当たる."""
        return self.cy - self.fy * math.tan(self.pitch_rad)

    def nearest_ground_m(self) -> float:
        """画像の下端に写る床までの前方距離 [m]."""
        p = pixel_to_ground(self, self.cx, self.height - 1)
        return p[0] if p else float("inf")


def pixel_to_ground(cam: CameraModel, u: float, v: float) -> Optional[Tuple[float, float]]:
    """画素 (u, v) を通る視線が床に当たる点をロボット座標 (X, Y) で返す.

    地平線より上の画素（床に当たらない）は None.
    """
    a = (u - cam.cx) / cam.fx
    b = (v - cam.cy) / cam.fy
    sin_t, cos_t = math.sin(cam.pitch_rad), math.cos(cam.pitch_rad)
    denom = sin_t + b * cos_t
    if denom <= 1e-6:
        return None
    s = cam.height_m / denom
    x = cam.x_offset_m + s * (cos_t - b * sin_t)
    y = cam.y_offset_m - s * a
    return x, y


def ground_to_pixel(cam: CameraModel, x: float, y: float, z: float = 0.0) -> Optional[Tuple[float, float]]:
    """ロボット座標の点 (X, Y, Z) が写る画素 (u, v) を返す. カメラの後ろは None.

    画像の外に出る場合もそのまま返す（範囲の判定は呼ぶ側で行う）.
    """
    sin_t, cos_t = math.sin(cam.pitch_rad), math.cos(cam.pitch_rad)
    dx = x - cam.x_offset_m
    dy = y - cam.y_offset_m
    dz = z - cam.height_m
    depth = dx * cos_t - dz * sin_t          # 光軸方向
    right = -dy                               # 画像の右方向
    down = -dx * sin_t - dz * cos_t           # 画像の下方向
    if depth <= 1e-6:
        return None
    return cam.cx + cam.fx * right / depth, cam.cy + cam.fy * down / depth


def in_image(cam: CameraModel, u: float, v: float, margin: float = 0.0) -> bool:
    return (margin <= u <= cam.width - 1 - margin) and (margin <= v <= cam.height - 1 - margin)


def robot_to_odom(pose: Pose2D, x: float, y: float) -> Tuple[float, float]:
    """ロボット座標の点をオドメトリ座標へ."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def odom_to_robot(pose: Pose2D, xo: float, yo: float) -> Tuple[float, float]:
    """オドメトリ座標の点をロボット座標へ."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    dx, dy = xo - pose.x, yo - pose.y
    return c * dx + s * dy, -s * dx + c * dy


class PoseHistory:
    """オドメトリの短い履歴. 画像の撮影時刻でのロボットの位置を引くために使う."""

    def __init__(self, horizon_s: float = 10.0):
        self._horizon = horizon_s
        self._stamps: List[float] = []
        self._poses: List[Pose2D] = []
        self._lock = threading.Lock()

    def add(self, pose: Pose2D) -> None:
        with self._lock:
            if self._stamps and pose.stamp < self._stamps[-1]:
                # 時刻が巻き戻った（シミュレーションのリセットなど）ら履歴を捨てる
                self._stamps.clear()
                self._poses.clear()
            self._stamps.append(pose.stamp)
            self._poses.append(pose)
            cut = bisect.bisect_left(self._stamps, pose.stamp - self._horizon)
            if cut > 0:
                del self._stamps[:cut]
                del self._poses[:cut]

    def latest(self) -> Optional[Pose2D]:
        with self._lock:
            return self._poses[-1] if self._poses else None

    def at(self, stamp: float) -> Optional[Pose2D]:
        """時刻 stamp の位置を線形補間で返す. 履歴の外は端の値で代用する."""
        with self._lock:
            if not self._poses:
                return None
            i = bisect.bisect_left(self._stamps, stamp)
            if i <= 0:
                return self._poses[0]
            if i >= len(self._poses):
                return self._poses[-1]
            p0, p1 = self._poses[i - 1], self._poses[i]
            span = p1.stamp - p0.stamp
            r = 0.0 if span <= 0.0 else (stamp - p0.stamp) / span
            yaw = p0.yaw + r * wrap_angle(p1.yaw - p0.yaw)
            return Pose2D(stamp, p0.x + r * (p1.x - p0.x), p0.y + r * (p1.y - p0.y), wrap_angle(yaw))
