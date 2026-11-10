"""シミュレーションの世界の寸法と名前. 世界の生成、エレベーターの制御、真値を使う部品が同じ値を使う.

座標の約束（ワールド座標）:
  * 出入口の敷居が x = 0. 乗場（廊下）は x < 0、かごは x > 0.
  * y は、乗場からかごを向いて左が正. z は上が正で、1階の床が z = 0.
かごは 13 人乗りの規格寸法（幅 1.6 m、奥行き 1.35 m、出入口の幅 0.9 m）に合わせている.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass(frozen=True)
class WorldParams:
    world_name: str = "elevator_world"
    building_model: str = "elevator_building"
    robot_model: str = "elevator_robot"

    num_floors: int = 3
    floor_height: float = 3.0          # 階高（床から上の階の床まで）

    # 出入口と扉（両開き、片側のパネルが半分ずつ動く）
    door_width: float = 0.90
    door_height: float = 2.10
    panel_thickness: float = 0.03
    panel_overlap: float = 0.01
    door_speed: float = 0.25           # パネル1枚の速さ [m/s]. 0.45 m を 1.8 秒で動く

    # 乗場
    hall_depth: float = 4.0
    hall_width: float = 4.0
    hall_wall: float = 0.10
    slab_thickness: float = 0.25
    sill_gap: float = 0.02             # 乗場の床とかごの床のすき間

    # かご
    car_width: float = 1.60
    car_depth: float = 1.35
    car_height: float = 2.30
    car_wall: float = 0.04
    car_floor_thickness: float = 0.10
    lift_speed: float = 1.0            # [m/s]
    lift_accel: float = 0.8            # [m/s^2]

    # 奥行き方向（x）の配置
    hall_wall_x0: float = -0.20        # 乗場の壁（出入口のある壁）
    hall_wall_x1: float = -0.10
    landing_door_x: float = -0.07      # 乗場の扉のパネルの中心
    car_door_x: float = 0.035          # かごの扉のパネルの中心
    car_front_x0: float = 0.06         # かごの前の壁（出入口のある壁）
    car_front_x1: float = 0.10

    # ロボット
    start_distance: float = 1.0        # 待機位置（ロボット中心）から敷居まで
    robot_radius: float = 0.30         # 出入口をふさいでいるかの判定に使う半径

    # -- 寸法から決まる値 ----------------------------------------------------
    @property
    def door_travel(self) -> float:
        return self.door_width / 2.0

    @property
    def panel_width(self) -> float:
        return self.door_width / 2.0 + self.panel_overlap

    @property
    def car_x0(self) -> float:
        """かごの室内の手前の端."""
        return self.car_front_x1

    @property
    def car_x1(self) -> float:
        """かごの室内の奥の端."""
        return self.car_front_x1 + self.car_depth

    @property
    def car_center_x(self) -> float:
        return 0.5 * (self.car_x0 + self.car_x1)

    @property
    def lift_max(self) -> float:
        return (self.num_floors - 1) * self.floor_height

    @property
    def wall_height(self) -> float:
        """乗場の壁の高さ（床から上の階の床版の下面まで）."""
        return self.floor_height - self.slab_thickness

    def floors(self) -> List[int]:
        return list(range(1, self.num_floors + 1))

    def floor_z(self, floor: int) -> float:
        return (floor - 1) * self.floor_height

    def nearest_floor(self, z: float, tol: float = 0.3) -> Optional[int]:
        """高さ z に最も近い階. どの階の床からも tol より離れていれば None."""
        floor = int(round(z / self.floor_height)) + 1
        if 1 <= floor <= self.num_floors and abs(z - self.floor_z(floor)) <= tol:
            return floor
        return None

    def start_pose(self, floor: int) -> Tuple[float, float, float, float]:
        """待機位置（x, y, z, 向き）. 扉の正面で、扉を向く."""
        return -self.start_distance, 0.0, self.floor_z(floor), 0.0

    def in_car(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.car_x0 + margin <= x <= self.car_x1 - margin) and abs(y) <= self.car_width / 2.0 - margin

    def doorway_blocked(self, x: float, y: float) -> bool:
        """ロボット（中心 x, y）が出入口の扉の通り道にかかっているか."""
        r = self.robot_radius
        return (self.hall_wall_x0 - r) < x < (self.car_front_x1 + r) and abs(y) < self.door_width / 2.0 + r

    # -- 関節とトピックの名前 --------------------------------------------------
    lift_joint: str = "lift_joint"
    car_door_joints: Tuple[str, str] = ("car_door_left_joint", "car_door_right_joint")
    lift_cmd_topic: str = "/elevator/lift/cmd_vel"
    car_door_cmd_topic: str = "/elevator/car_door/cmd_pos"
    joint_state_topic: str = "/elevator/joint_states"

    def landing_door_joints(self, floor: int) -> Tuple[str, str]:
        return f"landing_door_{floor}_left_joint", f"landing_door_{floor}_right_joint"

    def landing_door_cmd_topic(self, floor: int) -> str:
        return f"/elevator/landing_door_{floor}/cmd_pos"

    # 外から様子を見るための固定カメラ（人が見るためのもの. 実行系は使わない）
    def observer_names(self) -> List[str]:
        return [f"floor_{f}" for f in self.floors()] + ["car"]

    def observer_topic(self, name: str) -> str:
        return f"/observer/{name}/image_raw"


@dataclass(frozen=True)
class RobotParams:
    """シミュレーションのロボット（差動二輪、前向きの単眼カメラ）の寸法.

    モデルの原点は、駆動輪の車軸の中心の真下の床面.
    カメラの取り付け（高さ、俯角、前後の位置）を変えたら、実行系の設定（task.yaml の camera）も合わせること.
    """

    base_length: float = 0.50
    base_width: float = 0.36
    base_height: float = 0.16
    base_clearance: float = 0.06
    base_mass: float = 15.0
    wheel_radius: float = 0.08
    wheel_width: float = 0.04
    wheel_separation: float = 0.40
    wheel_mass: float = 1.0
    caster_radius: float = 0.04
    caster_x: float = 0.19
    camera_x: float = 0.20             # ロボット中心からカメラまでの前後の距離
    camera_z: float = 0.50             # 床からの高さ
    camera_pitch_deg: float = 15.0     # 俯角（下向きが正）
    camera_hfov: float = 1.5708        # 水平画角 [rad]（90 度）. 640 画素で焦点距離 320 画素
    image_width: int = 640
    image_height: int = 480
    camera_rate: float = 10.0

    @property
    def half_length(self) -> float:
        return self.base_length / 2.0

    @property
    def half_width(self) -> float:
        return self.wheel_separation / 2.0 + self.wheel_width / 2.0

    @property
    def focal_px(self) -> float:
        return self.image_width / (2.0 * math.tan(self.camera_hfov / 2.0))


@dataclass
class Truth:
    """真値のまとめ（モック VLM と審判が使う）."""

    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_z: float = 0.0
    robot_yaw: float = 0.0
    lift_pos: float = 0.0
    car_door: float = 0.0                   # かごの扉の開き [m]（2枚のうち小さい方）
    landing: Optional[Dict[int, float]] = None   # 階 → 乗場の扉の開き [m]

    def landing_open(self, floor: int) -> float:
        return (self.landing or {}).get(floor, 0.0)


def door_fully_open(params: WorldParams, truth: Truth, ratio: float = 0.95, level_tol: float = 0.02) -> bool:
    """ロボットから見える扉が全開か.

    かごがロボットのいる階に着床していて、かごの扉とその階の乗場の扉がどちらも開き切っているとき True.
    """
    car_floor = params.nearest_floor(truth.lift_pos, level_tol)
    if car_floor is None:
        return False
    need = ratio * params.door_travel
    if truth.car_door < need or truth.landing_open(car_floor) < need:
        return False
    if truth.robot_x >= 0.0 and params.in_car(truth.robot_x, truth.robot_y):
        return True
    robot_floor = params.nearest_floor(truth.robot_z)
    return robot_floor == car_floor


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
