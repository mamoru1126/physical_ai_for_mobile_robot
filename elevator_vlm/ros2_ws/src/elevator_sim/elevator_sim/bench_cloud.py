"""ダミーの点群. 擬似世界の壁と扉の面に点を並べ、LiDAR の座標系（cloud_link）で返す. ROS に依存しない.

見えるかどうか（ほかの壁の陰になるか）は考えない. 安全ゲートが「壁や扉に近づく動きを止めるか」を
確かめるためのもので、本物の LiDAR の見え方を真似るものではない.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from .world_params import RobotParams, Truth, WorldParams

HEIGHTS = (0.10, 0.30, 0.50, 0.70, 0.90, 1.20)   # 床からの高さ [m]
SPACING = 0.05                                     # 面に並べる点の間隔 [m]

Point = Tuple[float, float, float]
Segment = Tuple[float, float, float, float]      # (x0, y0, x1, y1) ワールド座標


@dataclass(frozen=True)
class CloudMount:
    """base_link から見た cloud_link の位置と向き（ヨーだけ）."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.4
    yaw: float = 0.0


def _hall_segments(wp: WorldParams, landing_open: float) -> List[Segment]:
    """乗場（1つの階）の壁と、乗場の扉のパネル."""
    half_hall = wp.hall_width / 2.0
    half_door = wp.door_width / 2.0
    x_wall = wp.hall_wall_x0
    segs: List[Segment] = [
        (x_wall, half_door, x_wall, half_hall),          # 出入口のある壁（左）
        (x_wall, -half_hall, x_wall, -half_door),        # 出入口のある壁（右）
        (x_wall, half_hall, -wp.hall_depth, half_hall),  # 横の壁
        (x_wall, -half_hall, -wp.hall_depth, -half_hall),
        (-wp.hall_depth, -half_hall, -wp.hall_depth, half_hall),   # 奥の壁
    ]
    o = max(0.0, min(landing_open, half_door))
    if o < half_door - 1e-3:   # 扉のパネルの、出入口に出ている部分
        segs.append((wp.landing_door_x, o, wp.landing_door_x, half_door))
        segs.append((wp.landing_door_x, -half_door, wp.landing_door_x, -o))
    return segs


def _car_segments(wp: WorldParams, car_open: float) -> List[Segment]:
    """かごの壁と、かごの扉のパネル."""
    half_car = wp.car_width / 2.0
    half_door = wp.door_width / 2.0
    x0, x1 = wp.car_x0, wp.car_x1
    segs: List[Segment] = [
        (x1, -half_car, x1, half_car),                   # 奥の壁
        (x0, half_car, x1, half_car),                    # 横の壁
        (x0, -half_car, x1, -half_car),
        (x0, half_door, x0, half_car),                   # 出入口のある壁
        (x0, -half_car, x0, -half_door),
    ]
    o = max(0.0, min(car_open, half_door))
    if o < half_door - 1e-3:
        segs.append((wp.car_door_x, o, wp.car_door_x, half_door))
        segs.append((wp.car_door_x, -half_door, wp.car_door_x, -o))
    return segs


def world_points(wp: WorldParams, truth: Truth, rp: RobotParams, mount: CloudMount,
                 max_range: float = 5.0) -> List[Point]:
    """今のロボットのまわりの壁と扉の点を、cloud_link の座標で返す."""
    robot_level = truth.robot_z
    car_here = abs(truth.lift_pos - robot_level) < 0.05
    in_car = truth.robot_x > wp.car_front_x0 and car_here
    segs: List[Segment] = []
    floor = wp.nearest_floor(robot_level, 0.05)
    if floor is not None and (not in_car or truth.car_door > 0.02):
        segs += _hall_segments(wp, truth.landing_open(floor))
    if car_here:
        segs += _car_segments(wp, truth.car_door)

    c, s = math.cos(truth.robot_yaw), math.sin(truth.robot_yaw)
    cm, sm = math.cos(mount.yaw), math.sin(mount.yaw)
    out: List[Point] = []
    for x0, y0, x1, y1 in segs:
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(length / SPACING))
        for i in range(n + 1):
            t = i / n
            wx, wy = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            dx, dy = wx - truth.robot_x, wy - truth.robot_y
            if math.hypot(dx, dy) > max_range:
                continue
            bx, by = c * dx + s * dy, -s * dx + c * dy            # base_link（前, 左）
            px, py = bx - mount.x, by - mount.y
            lx, ly = cm * px + sm * py, -sm * px + cm * py        # cloud_link
            for h in HEIGHTS:
                out.append((lx, ly, h - mount.z))
    return out
