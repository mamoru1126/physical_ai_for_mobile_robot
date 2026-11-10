"""シミュレーションの世界（SDF）、ロボットのモデル、テクスチャ、ブリッジの設定を生成する.

  python3 -m elevator_sim.world_gen --pkg-dir <elevator_sim パッケージのソース>

寸法は world_params.py の WorldParams / RobotParams で決まる. 生成物はリポジトリに入れてあるので、
寸法を変えたときだけ実行すればよい（--no-textures でテクスチャの再生成を省ける）.

建物は1つのモデルにまとめてある（同じモデルの中のリンクどうしは衝突しないので、
かごや扉が壁とこすれて動かなくなることがない）:
  base        … 各階の乗場（床、壁）. ワールドに固定
  car         … かご. base と直動関節 lift_joint（上下）でつながる
  car_door_*  … かごの扉（2枚）. car と直動関節（左右）でつながる
  landing_door_<階>_* … 各階の乗場の扉（2枚）. base と直動関節でつながる
"""
from __future__ import annotations

import argparse
import math
import os
from typing import List, Optional, Sequence, Tuple

from .world_params import RobotParams, WorldParams

TEX_MODEL = "elevator_textures"
TEX_URI = f"model://{TEX_MODEL}/materials/textures"
HEADER = "<!-- elevator_sim/world_gen.py が生成したファイル. 手で編集せず、生成し直すこと. -->"

Color = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# SDF の部品
# ---------------------------------------------------------------------------
def _f(v: float) -> str:
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> str:
    return f"<pose>{_f(x)} {_f(y)} {_f(z)} {_f(roll)} {_f(pitch)} {_f(yaw)}</pose>"


def material(color: Color = (0.8, 0.8, 0.8), tex: Optional[str] = None, roughness: float = 0.8,
             metalness: float = 0.0, emissive: Optional[Color] = None, specular: float = 0.1) -> str:
    r, g, b = color
    s = (f"<material><ambient>{_f(r)} {_f(g)} {_f(b)} 1</ambient><diffuse>{_f(r)} {_f(g)} {_f(b)} 1</diffuse>"
         f"<specular>{_f(specular)} {_f(specular)} {_f(specular)} 1</specular>")
    if emissive is not None:
        s += f"<emissive>{_f(emissive[0])} {_f(emissive[1])} {_f(emissive[2])} 1</emissive>"
    if tex is not None:
        s += (f"<pbr><metal><albedo_map>{TEX_URI}/{tex}.png</albedo_map>"
              f"<roughness>{_f(roughness)}</roughness><metalness>{_f(metalness)}</metalness></metal></pbr>")
    return s + "</material>"


def box_geom(sx: float, sy: float, sz: float) -> str:
    return f"<geometry><box><size>{_f(sx)} {_f(sy)} {_f(sz)}</size></box></geometry>"


def box_inertial(mass: float, sx: float, sy: float, sz: float, at: Optional[Tuple[float, float, float]] = None) -> str:
    ixx = mass / 12.0 * (sy * sy + sz * sz)
    iyy = mass / 12.0 * (sx * sx + sz * sz)
    izz = mass / 12.0 * (sx * sx + sy * sy)
    p = pose(*at) if at else ""
    return (f"<inertial>{p}<mass>{_f(mass)}</mass><inertia><ixx>{ixx:.6g}</ixx><ixy>0</ixy><ixz>0</ixz>"
            f"<iyy>{iyy:.6g}</iyy><iyz>0</iyz><izz>{izz:.6g}</izz></inertia></inertial>")


class Link:
    """リンク1つ分の衝突形状と見た目を集める."""

    def __init__(self, name: str, at: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 inertial: str = "", gravity: bool = True):
        self.name = name
        self.at = at
        self.inertial = inertial
        self.gravity = gravity
        self.items: List[str] = []
        self._n = 0

    def box(self, name: str, lo: Sequence[float], hi: Sequence[float], mat: Optional[str],
            collide: bool = True, mu: Optional[float] = None) -> None:
        """ワールド座標の2つの隅 lo, hi で直方体を置く（リンクの位置を引いて書き出す）."""
        size = [hi[i] - lo[i] for i in range(3)]
        if min(size) <= 0.0:
            raise ValueError(f"{self.name}/{name}: 大きさが正ではありません {size}")
        center = [0.5 * (lo[i] + hi[i]) - self.at[i] for i in range(3)]
        p = pose(*center)
        g = box_geom(*size)
        self._n += 1
        if collide:
            surf = ""
            if mu is not None:
                surf = f"<surface><friction><ode><mu>{_f(mu)}</mu><mu2>{_f(mu)}</mu2></ode></friction></surface>"
            self.items.append(f'<collision name="{name}_c">{p}{g}{surf}</collision>')
        if mat is not None:
            self.items.append(f'<visual name="{name}_v">{p}{g}{mat}</visual>')

    def raw(self, xml: str) -> None:
        self.items.append(xml)

    def sdf(self) -> str:
        grav = "" if self.gravity else "<gravity>false</gravity>"
        body = "\n        ".join(self.items)
        return (f'      <link name="{self.name}">\n        {pose(*self.at)}{grav}\n        {self.inertial}\n'
                f"        {body}\n      </link>")


def prismatic(name: str, parent: str, child: str, axis: Tuple[int, int, int], lower: float, upper: float,
              effort: float, velocity: float) -> str:
    return (f'      <joint name="{name}" type="prismatic"><parent>{parent}</parent><child>{child}</child>'
            f"<axis><xyz>{axis[0]} {axis[1]} {axis[2]}</xyz><limit><lower>{_f(lower)}</lower><upper>{_f(upper)}</upper>"
            f"<effort>{_f(effort)}</effort><velocity>{_f(velocity)}</velocity></limit>"
            f"<dynamics><damping>0</damping><friction>0</friction></dynamics></axis></joint>")


# ---------------------------------------------------------------------------
# 建物（乗場、かご、扉）
# ---------------------------------------------------------------------------
WALL_COLORS = [(0.93, 0.91, 0.86), (0.88, 0.89, 0.90), (0.84, 0.89, 0.93)]
ACCENT_COLORS = [(0.80, 0.33, 0.20), (0.25, 0.55, 0.35), (0.20, 0.38, 0.70)]
STEEL = material((0.86, 0.87, 0.88), tex="door_steel", roughness=0.45, metalness=0.0, specular=0.3)
FRAME = material((0.30, 0.31, 0.33), specular=0.2)
DARK = material((0.10, 0.10, 0.11))
WHITE = material((0.95, 0.95, 0.95))


def _hall(base: Link, p: WorldParams, floor: int) -> None:
    """1つの階の乗場（床、壁、出入口の枠、表示、小物）を base に足す."""
    z0 = p.floor_z(floor)
    zc = z0 + p.wall_height                    # 天井（上の階の床版の下面）
    hw = p.hall_width / 2.0
    x_back = -p.hall_depth
    x_edge = -p.sill_gap / 2.0
    dw = p.door_width / 2.0
    ci = (floor - 1) % len(WALL_COLORS)
    wall = material(WALL_COLORS[ci])
    accent = material(ACCENT_COLORS[ci])
    n = f"f{floor}"

    # 床版（上面が床）. 乗場の扉の敷居まで続く
    base.box(f"{n}_slab", (x_back - p.hall_wall, -hw - p.hall_wall, z0 - p.slab_thickness), (x_edge, hw + p.hall_wall, z0),
             material((1, 1, 1), tex=f"floor_{floor}f", roughness=0.9), mu=1.0)
    # 出入口のある壁（左、右、上）
    base.box(f"{n}_front_l", (p.hall_wall_x0, dw, z0), (p.hall_wall_x1, hw, zc), wall)
    base.box(f"{n}_front_r", (p.hall_wall_x0, -hw, z0), (p.hall_wall_x1, -dw, zc), wall)
    base.box(f"{n}_front_top", (p.hall_wall_x0, -dw, z0 + p.door_height), (p.hall_wall_x1, dw, zc), wall)
    # 側面と奥の壁
    base.box(f"{n}_side_l", (x_back, hw, z0), (p.hall_wall_x0, hw + p.hall_wall, zc), wall)
    base.box(f"{n}_side_r", (x_back, -hw - p.hall_wall, z0), (p.hall_wall_x0, -hw, zc), wall)
    base.box(f"{n}_back", (x_back - p.hall_wall, -hw - p.hall_wall, z0), (x_back, hw + p.hall_wall, zc), wall)
    # 巾木（階ごとに色を変える）
    base.box(f"{n}_skirt_l", (x_back, hw - 0.012, z0), (p.hall_wall_x0, hw, z0 + 0.10), accent, collide=False)
    base.box(f"{n}_skirt_r", (x_back, -hw, z0), (p.hall_wall_x0, -hw + 0.012, z0 + 0.10), accent, collide=False)
    base.box(f"{n}_skirt_b", (x_back, -hw, z0), (x_back + 0.012, hw, z0 + 0.10), accent, collide=False)

    # 出入口の枠（三方枠）
    # 壁の面と重ならないよう、開口の内側と壁の前後に数 mm はみ出させる
    fx0, fx1 = p.hall_wall_x0 - 0.012, p.hall_wall_x1 + 0.004
    ji, zt = dw - 0.004, z0 + p.door_height
    base.box(f"{n}_jamb_l", (fx0, ji, z0), (fx1, dw + 0.06, zt + 0.06), FRAME, collide=False)
    base.box(f"{n}_jamb_r", (fx0, -dw - 0.06, z0), (fx1, -ji, zt + 0.06), FRAME, collide=False)
    base.box(f"{n}_jamb_t", (fx0, -ji, zt - 0.004), (fx1, ji, zt + 0.06), FRAME, collide=False)
    # 敷居（床とほぼ同じ高さの金属の帯）
    base.box(f"{n}_sill", (p.hall_wall_x0, -dw, z0), (x_edge, dw, z0 + 0.002), material((0.55, 0.56, 0.58)), collide=False)

    xs = p.hall_wall_x0 - 0.008                 # 壁の乗場側の面から少し手前
    # 階数の表示（扉の上）、呼びボタン（扉の右）、階の札（扉の左）
    base.box(f"{n}_indicator", (xs - 0.01, -0.16, z0 + p.door_height + 0.14), (p.hall_wall_x0, 0.16, z0 + p.door_height + 0.28),
             material((1, 1, 1), tex=f"indicator_{floor}f", emissive=(0.35, 0.35, 0.35)), collide=False)
    base.box(f"{n}_callpanel", (xs, -dw - 0.33, z0 + 1.00), (p.hall_wall_x0, -dw - 0.21, z0 + 1.28),
             material((1, 1, 1), tex="call_panel"), collide=False)
    base.box(f"{n}_sign", (xs, dw + 0.22, z0 + 1.40), (p.hall_wall_x0, dw + 0.58, z0 + 1.76),
             material((1, 1, 1), tex=f"sign_{floor}f"), collide=False)
    # 奥の壁: 大きな階の札と、部屋の扉に見える板
    xb = x_back + 0.008
    base.box(f"{n}_bigsign", (x_back, -0.45, z0 + 1.25), (xb, 0.45, z0 + 2.15),
             material((1, 1, 1), tex=f"sign_{floor}f"), collide=False)
    base.box(f"{n}_roomdoor", (x_back, 1.00, z0), (xb, 1.80, z0 + 2.00), material((0.45, 0.30, 0.18)), collide=False)
    base.box(f"{n}_roomknob", (x_back, 1.08, z0 + 0.98), (xb + 0.03, 1.13, z0 + 1.03), material((0.75, 0.72, 0.55)),
             collide=False)
    # 天井の照明（見た目だけ）
    for i, lx in enumerate((-1.2, -2.9)):
        base.box(f"{n}_lamp{i}", (lx - 0.5, -0.15, zc - 0.02), (lx + 0.5, 0.15, zc),
                 material((1, 1, 1), emissive=(1.0, 1.0, 0.95)), collide=False)

    # 小物（通り道の外）. 階ごとに変える
    if floor % 3 == 1:
        base.box(f"{n}_bench_seat", (-2.6, -hw + 0.05, z0 + 0.36), (-1.4, -hw + 0.45, z0 + 0.42), material((0.50, 0.34, 0.20)))
        base.box(f"{n}_bench_leg0", (-2.55, -hw + 0.08, z0), (-2.47, -hw + 0.42, z0 + 0.36), DARK)
        base.box(f"{n}_bench_leg1", (-1.53, -hw + 0.08, z0), (-1.45, -hw + 0.42, z0 + 0.36), DARK)
        _plant(base, f"{n}_plant", -3.5, hw - 0.4, z0)
    elif floor % 3 == 2:
        base.box(f"{n}_cabinet", (-3.2, -hw + 0.02, z0), (-2.2, -hw + 0.45, z0 + 0.95), material((0.62, 0.64, 0.66)))
        base.box(f"{n}_cabinet_top", (-3.22, -hw + 0.02, z0 + 0.95), (-2.18, -hw + 0.47, z0 + 0.98), DARK)
    else:
        base.box(f"{n}_vending", (-3.7, -hw + 0.02, z0), (-2.9, -hw + 0.70, z0 + 1.80), material((0.80, 0.12, 0.12)))
        base.box(f"{n}_vending_win", (-3.62, -hw + 0.70, z0 + 0.85), (-2.98, -hw + 0.705, z0 + 1.65),
                 material((0.75, 0.88, 0.95), emissive=(0.25, 0.3, 0.33)), collide=False)
        _plant(base, f"{n}_plant", -3.5, hw - 0.4, z0)


def _plant(base: Link, name: str, x: float, y: float, z0: float) -> None:
    pot = "<geometry><cylinder><radius>0.16</radius><length>0.36</length></cylinder></geometry>"
    base.raw(f'<collision name="{name}_pot_c">{pose(x, y, z0 + 0.18)}{pot}</collision>')
    base.raw(f'<visual name="{name}_pot_v">{pose(x, y, z0 + 0.18)}{pot}{material((0.55, 0.36, 0.26))}</visual>')
    for i, (dx, dy, dz, r) in enumerate(((0.0, 0.0, 0.66, 0.26), (0.08, 0.05, 0.95, 0.20), (-0.07, -0.04, 1.15, 0.15))):
        base.raw(f'<visual name="{name}_leaf{i}_v">{pose(x + dx, y + dy, z0 + dz)}'
                 f"<geometry><sphere><radius>{_f(r)}</radius></sphere></geometry>{material((0.16, 0.45, 0.20))}</visual>")


def _door_panel(name: str, p: WorldParams, x_center: float, side: int, z0: float) -> Link:
    """扉のパネル1枚. side は +1 で左（y が正の側）、-1 で右."""
    half_t = p.panel_thickness / 2.0
    y_in, y_out = 0.001 * side, p.panel_width * side      # 合わせ目に 2 mm のすき間
    lo_y, hi_y = min(y_in, y_out), max(y_in, y_out)
    z_lo, z_hi = z0 + 0.006, z0 + p.door_height + 0.03
    center = (x_center, 0.5 * (lo_y + hi_y), 0.5 * (z_lo + z_hi))
    link = Link(name, center, box_inertial(15.0, p.panel_thickness, hi_y - lo_y, z_hi - z_lo), gravity=False)
    link.box("panel", (x_center - half_t, lo_y, z_lo), (x_center + half_t, hi_y, z_hi), STEEL)
    # 合わせ目の縁を濃くして、2枚の扉であることが分かるようにする
    e_lo, e_hi = (lo_y, lo_y + 0.012) if side > 0 else (hi_y - 0.012, hi_y)
    link.box("edge", (x_center - half_t - 0.001, e_lo, z_lo), (x_center + half_t + 0.001, e_hi, z_hi),
             material((0.38, 0.39, 0.41)), collide=False)
    return link


def _car(p: WorldParams) -> Link:
    """かご（床の上面がリンクの原点の高さ）."""
    hw = p.car_width / 2.0
    w = p.car_wall
    x_floor0 = p.sill_gap / 2.0
    x_in0, x_in1 = p.car_x0, p.car_x1
    x_out1 = x_in1 + w
    dw = p.door_width / 2.0
    h = p.car_height
    car = Link("car", (0.0, 0.0, 0.0),
               box_inertial(600.0, p.car_depth, p.car_width, h, at=(p.car_center_x, 0.0, h / 2.0)), gravity=False)
    panel = material((0.80, 0.81, 0.82), tex="car_wall", roughness=0.5, specular=0.25)
    car.box("floor", (x_floor0, -hw - w, -p.car_floor_thickness), (x_out1, hw + w, 0.0),
            material((1, 1, 1), tex="car_floor", roughness=0.85), mu=1.0)
    car.box("sill", (x_floor0, -dw, 0.0), (p.car_front_x1, dw, 0.002), material((0.55, 0.56, 0.58)), collide=False)
    car.box("wall_l", (p.car_front_x0, hw, 0.0), (x_out1, hw + w, h), panel)
    car.box("wall_r", (p.car_front_x0, -hw - w, 0.0), (x_out1, -hw, h), panel)
    car.box("wall_b", (x_in1, -hw, 0.0), (x_out1, hw, h), panel)
    # 前の壁（袖壁と、出入口の上）
    car.box("front_l", (p.car_front_x0, dw, 0.0), (p.car_front_x1, hw, h), panel)
    car.box("front_r", (p.car_front_x0, -hw, 0.0), (p.car_front_x1, -dw, h), panel)
    car.box("front_top", (p.car_front_x0, -dw, p.door_height), (p.car_front_x1, dw, h), panel)
    car.box("ceiling", (p.car_front_x0, -hw - w, h), (x_out1, hw + w, h + 0.04), WHITE, collide=False)
    car.box("lamp", (x_in0 + 0.25, -0.45, h - 0.015), (x_in1 - 0.25, 0.45, h),
            material((1, 1, 1), emissive=(1.0, 1.0, 0.96)), collide=False)
    # 手すり、操作盤、階数の表示（見た目だけ）
    for side, name in ((1, "l"), (-1, "r")):
        y0, y1 = (hw - 0.05, hw - 0.02) if side > 0 else (-hw + 0.02, -hw + 0.05)
        car.box(f"rail_{name}", (x_in0 + 0.20, y0, 0.84), (x_in1 - 0.15, y1, 0.88), material((0.70, 0.71, 0.73)), collide=False)
    car.box("rail_b", (x_in1 - 0.05, -hw + 0.15, 0.84), (x_in1 - 0.02, hw - 0.15, 0.88), material((0.70, 0.71, 0.73)),
            collide=False)
    car.box("cop", (p.car_front_x1, -hw + 0.06, 0.95), (p.car_front_x1 + 0.008, -dw - 0.06, 1.45),
            material((1, 1, 1), tex="cop_panel"), collide=False)
    car.box("car_indicator", (p.car_front_x1, -0.14, p.door_height + 0.05), (p.car_front_x1 + 0.008, 0.14, p.door_height + 0.15),
            material((0.05, 0.05, 0.05), emissive=(0.30, 0.16, 0.0)), collide=False)
    # 奥の壁の腰の高さに色の帯（かごの中だと分かりやすくする）
    car.box("band_b", (x_in1 - 0.004, -hw, 0.95), (x_in1, hw, 1.05), material((0.35, 0.36, 0.40)), collide=False)
    return car


def observer_sensor(name: str, topic: str, hfov: float, pose_xml: str = "") -> str:
    return (f'<sensor name="{name}" type="camera">{pose_xml}<always_on>true</always_on><update_rate>2</update_rate>'
            f"<visualize>false</visualize><topic>{topic}</topic><camera><horizontal_fov>{_f(hfov)}</horizontal_fov>"
            "<image><width>640</width><height>480</height><format>R8G8B8</format></image>"
            "<clip><near>0.05</near><far>60</far></clip></camera></sensor>")


def observer_models(p: WorldParams) -> str:
    """各階の乗場を斜め上から見る固定カメラ（様子を見るためのもの）."""
    out = []
    for floor in p.floors():
        z0 = p.floor_z(floor)
        cx, cy, cz = -2.7, -1.6, z0 + 2.3
        tx, ty, tz = -0.3, 0.0, z0 + 1.0
        yaw = math.atan2(ty - cy, tx - cx)
        pitch = math.atan2(cz - tz, math.hypot(tx - cx, ty - cy))
        name = f"floor_{floor}"
        out.append(
            f'    <model name="observer_{name}"><static>true</static>{pose(cx, cy, cz, 0, pitch, yaw)}'
            f'<link name="link">{observer_sensor("camera", p.observer_topic(name), 1.4)}</link></model>')
    return "\n".join(out)


def building_sdf(p: WorldParams) -> str:
    hw = p.hall_width / 2.0
    base = Link("base", (0.0, 0.0, 0.0), box_inertial(20000.0, 4.0, 4.0, 3.0, at=(-2.0, 0.0, 1.5)))
    for floor in p.floors():
        _hall(base, p, floor)
    # 最上階の天井
    z_roof = p.floor_z(p.num_floors) + p.floor_height
    base.box("roof", (-p.hall_depth - p.hall_wall, -hw - p.hall_wall, z_roof - p.slab_thickness),
             (-p.sill_gap / 2.0, hw + p.hall_wall, z_roof), material((0.80, 0.80, 0.80)), collide=False)

    links: List[str] = [base.sdf()]
    joints: List[str] = [
        '      <joint name="fix_base" type="fixed"><parent>world</parent><child>base</child></joint>']
    plugins: List[str] = []

    car = _car(p)
    # かごの中を奥の上の隅から見るカメラ（様子を見るためのもの）
    cx, cy, cz = p.car_x1 - 0.06, p.car_width / 2.0 - 0.08, p.car_height - 0.12
    tx, ty, tz = p.car_x0 + 0.25, 0.0, 0.3
    car.raw(observer_sensor(
        "observer_car", p.observer_topic("car"), 1.9,
        pose(cx, cy, cz, 0, math.atan2(cz - tz, math.hypot(tx - cx, ty - cy)), math.atan2(ty - cy, tx - cx))))
    links.append(car.sdf())
    joints.append(prismatic(p.lift_joint, "base", "car", (0, 0, 1), -0.02, p.lift_max + 0.02, 60000.0, 2.0))
    plugins.append(
        '      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">'
        f"<joint_name>{p.lift_joint}</joint_name><topic>{p.lift_cmd_topic}</topic><initial_velocity>0</initial_velocity></plugin>")

    def door_pair(prefix: str, parent: str, x_center: float, z0: float, joint_names: Tuple[str, str], topic: str) -> None:
        for side, tag, joint in ((1, "left", joint_names[0]), (-1, "right", joint_names[1])):
            link_name = f"{prefix}_{tag}"
            links.append(_door_panel(link_name, p, x_center, side, z0).sdf())
            joints.append(prismatic(joint, parent, link_name, (0, side, 0), 0.0, p.door_travel, 150.0, 1.0))
            plugins.append(
                '      <plugin filename="gz-sim-joint-position-controller-system" '
                'name="gz::sim::systems::JointPositionController">'
                f"<joint_name>{joint}</joint_name><topic>{topic}</topic><use_velocity_commands>true</use_velocity_commands>"
                f"<cmd_max>{_f(p.door_speed)}</cmd_max><cmd_min>{_f(-p.door_speed)}</cmd_min>"
                "<initial_position>0</initial_position></plugin>")

    door_pair("car_door", "car", p.car_door_x, 0.0, p.car_door_joints, p.car_door_cmd_topic)
    for floor in p.floors():
        door_pair(f"landing_door_{floor}", "base", p.landing_door_x, p.floor_z(floor),
                  p.landing_door_joints(floor), p.landing_door_cmd_topic(floor))
    plugins.append(
        '      <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">'
        f"<topic>{p.joint_state_topic}</topic><update_rate>50</update_rate></plugin>")

    body = "\n".join(links + joints + plugins)
    return (f'    <model name="{p.building_model}">\n      <static>false</static>\n      <self_collide>false</self_collide>\n'
            f"{body}\n    </model>")


def world_sdf(p: WorldParams, board_floor: int = 1) -> str:
    x, y, z, yaw = p.start_pose(board_floor)
    lights = []
    for name, direction, level in (("light_fwd", (0.5, 0.2, -0.85), 0.45), ("light_back", (-0.5, -0.2, -0.85), 0.45),
                                   ("light_left", (0.1, 0.6, -0.8), 0.25), ("light_right", (-0.1, -0.6, -0.8), 0.25)):
        lights.append(
            f'    <light type="directional" name="{name}"><cast_shadows>false</cast_shadows>{pose(0, 0, 30)}'
            f"<diffuse>{level} {level} {level} 1</diffuse><specular>0.05 0.05 0.05 1</specular>"
            f"<direction>{direction[0]} {direction[1]} {direction[2]}</direction></light>")
    ground_z = -p.slab_thickness - 0.02
    return f"""<?xml version="1.0" ?>
{HEADER}
<sdf version="1.9">
  <world name="{p.world_name}">
    <physics name="2ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.78 0.83 0.88 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
{chr(10).join(lights)}

    <model name="ground_plane">
      <static>true</static>
      {pose(0, 0, ground_z)}
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>{material((0.42, 0.44, 0.42))}</visual>
      </link>
    </model>

{building_sdf(p)}

{observer_models(p)}

    <include>
      <uri>model://{p.robot_model}</uri>
      <name>{p.robot_model}</name>
      {pose(x, y, z + 0.01, 0, 0, yaw)}
    </include>
  </world>
</sdf>
"""


# ---------------------------------------------------------------------------
# ロボット
# ---------------------------------------------------------------------------
def robot_sdf(p: WorldParams, r: RobotParams) -> str:
    bz = r.base_clearance + r.base_height / 2.0
    wheel_y = r.wheel_separation / 2.0
    wr, ww = r.wheel_radius, r.wheel_width
    w_ixx = r.wheel_mass / 12.0 * (3 * wr * wr + ww * ww)
    w_izz = 0.5 * r.wheel_mass * wr * wr
    cr = r.caster_radius
    pitch = math.radians(r.camera_pitch_deg)
    body = material((0.15, 0.35, 0.75), specular=0.3)
    dark = material((0.08, 0.08, 0.09))
    tower_h = r.camera_z - 0.03 - (r.base_clearance + r.base_height)
    tower_z = r.base_clearance + r.base_height + tower_h / 2.0

    def wheel(name: str, y: float) -> str:
        geom = f"<geometry><cylinder><radius>{_f(wr)}</radius><length>{_f(ww)}</length></cylinder></geometry>"
        return f"""    <link name="{name}">
      {pose(0, y, wr, -math.pi / 2, 0, 0)}
      <inertial><mass>{_f(r.wheel_mass)}</mass><inertia><ixx>{w_ixx:.6g}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{w_ixx:.6g}</iyy><iyz>0</iyz><izz>{w_izz:.6g}</izz></inertia></inertial>
      <collision name="collision">{geom}<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="visual">{geom}{dark}</visual>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent><child>{name}</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit></axis>
    </joint>"""

    def caster(name: str, x: float) -> str:
        geom = f"<geometry><sphere><radius>{_f(cr)}</radius></sphere></geometry>"
        return (f'      <collision name="{name}_c">{pose(x, 0, cr)}{geom}'
                "<surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface></collision>\n"
                f'      <visual name="{name}_v">{pose(x, 0, cr)}{geom}{dark}</visual>')

    return f"""<?xml version="1.0"?>
{HEADER}
<!--
  差動二輪の移動ロボット（Gazebo Harmonic）. センサは前向きの単眼カメラと車輪オドメトリだけ.
    カメラ:        /camera/image_raw（{r.image_width}x{r.image_height}、{_f(r.camera_rate)} Hz、水平画角 {_f(r.camera_hfov)} rad、
                   床から {_f(r.camera_z)} m、俯角 {_f(r.camera_pitch_deg)} 度、車軸から前へ {_f(r.camera_x)} m）
    速度指令:      /cmd_vel
    車輪オドメトリ: /odom（車輪の回転から計算. 起動時の位置が原点）
    真値:          /ground_truth/odom（ワールド座標. 評価とモック VLM だけが使う）
-->
<sdf version="1.9">
  <model name="{p.robot_model}">
    <link name="base_link">
      {box_inertial(r.base_mass, r.base_length, r.base_width, r.base_height, at=(0, 0, bz))}
      <collision name="body_c">{pose(0, 0, bz)}{box_geom(r.base_length, r.base_width, r.base_height)}</collision>
      <visual name="body_v">{pose(0, 0, bz)}{box_geom(r.base_length, r.base_width, r.base_height)}{body}</visual>
      <collision name="tower_c">{pose(0.08, 0, tower_z)}{box_geom(0.20, 0.24, tower_h)}</collision>
      <visual name="tower_v">{pose(0.08, 0, tower_z)}{box_geom(0.20, 0.24, tower_h)}{material((0.85, 0.86, 0.88))}</visual>
{caster("caster_front", r.caster_x)}
{caster("caster_rear", -r.caster_x)}
    </link>

    <link name="camera_link">
      {pose(r.camera_x, 0, r.camera_z, 0, pitch, 0)}
      <inertial><mass>0.05</mass><inertia><ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz></inertia></inertial>
      <visual name="visual">{pose(-0.025, 0, 0)}{box_geom(0.04, 0.08, 0.04)}{dark}</visual>
      <sensor name="front_camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{_f(r.camera_rate)}</update_rate>
        <visualize>false</visualize>
        <topic>/camera/image_raw</topic>
        <camera name="front">
          <horizontal_fov>{_f(r.camera_hfov)}</horizontal_fov>
          <image><width>{r.image_width}</width><height>{r.image_height}</height><format>R8G8B8</format></image>
          <clip><near>0.03</near><far>60</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>

{wheel("left_wheel", wheel_y)}
{wheel("right_wheel", -wheel_y)}

    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>{_f(r.wheel_separation)}</wheel_separation>
      <wheel_radius>{_f(wr)}</wheel_radius>
      <topic>/cmd_vel</topic>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/wheel_tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>50</odom_publish_frequency>
      <max_linear_acceleration>1.5</max_linear_acceleration>
      <min_linear_acceleration>-1.5</min_linear_acceleration>
      <max_angular_acceleration>4.0</max_angular_acceleration>
      <min_angular_acceleration>-4.0</min_angular_acceleration>
      <max_linear_velocity>1.0</max_linear_velocity>
      <min_linear_velocity>-1.0</min_linear_velocity>
      <max_angular_velocity>2.0</max_angular_velocity>
      <min_angular_velocity>-2.0</min_angular_velocity>
    </plugin>

    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>/ground_truth/odom</odom_topic>
      <tf_topic>/ground_truth/tf</tf_topic>
      <odom_publish_frequency>50</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
"""


def model_config(name: str, description: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""


def texture_model_sdf() -> str:
    return f"""<?xml version="1.0"?>
{HEADER}
<sdf version="1.9">
  <model name="{TEX_MODEL}_sample">
    <static>true</static>
    <link name="link">
      <visual name="visual">{box_geom(1, 1, 1)}{material((1, 1, 1), tex="door_steel")}</visual>
    </link>
  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# ブリッジの設定
# ---------------------------------------------------------------------------
def bridge_yaml(p: WorldParams) -> str:
    def entry(topic: str, ros_type: str, gz_type: str, direction: str) -> str:
        return (f"- ros_topic_name: {topic}\n  gz_topic_name: {topic}\n  ros_type_name: {ros_type}\n"
                f"  gz_type_name: {gz_type}\n  direction: {direction}\n")

    items = [
        entry("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
        entry("/camera/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
        entry("/camera/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "GZ_TO_ROS"),
        "# 車輪オドメトリ（ロボットが使ってよいセンサ）\n"
        + entry("/odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        entry("/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
        "# 真値（評価、エレベーターの扉のセンサ、モック VLM だけが使う. 実行系は使わない）\n"
        + entry("/ground_truth/odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        "# エレベーター（扉とかごの関節）\n"
        + entry(p.joint_state_topic, "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS"),
        entry(p.lift_cmd_topic, "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
        entry(p.car_door_cmd_topic, "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
    ]
    for floor in p.floors():
        items.append(entry(p.landing_door_cmd_topic(floor), "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"))
    items.append("# 外から様子を見るための固定カメラ（人が見るためのもの. 実行系は使わない）\n"
                 + "\n".join(entry(p.observer_topic(name), "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS")
                             for name in p.observer_names()))
    return ("# ros_gz_bridge (parameter_bridge) の設定: Gazebo Harmonic <-> ROS 2\n"
            "# elevator_sim/world_gen.py が生成したファイル. 手で編集せず、生成し直すこと.\n" + "\n".join(items))


# ---------------------------------------------------------------------------
# テクスチャ（PIL で描く. 外部の素材は使わない）
# ---------------------------------------------------------------------------
def generate_textures(out_dir: str, p: WorldParams) -> List[str]:
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    os.makedirs(out_dir, exist_ok=True)
    rng = np.random.default_rng(0)
    written: List[str] = []

    def save(img: "Image.Image", name: str) -> None:
        path = os.path.join(out_dir, f"{name}.png")
        img.save(path)
        written.append(path)

    def font(size: int):
        try:
            return ImageFont.load_default(size=size)
        except TypeError:           # Pillow 10.1 より前は、既定のフォントの大きさを指定できない
            for name in ("DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
                try:
                    return ImageFont.truetype(name, size)
                except OSError:
                    continue
            raise RuntimeError("テクスチャの生成には Pillow 10.1 以降か DejaVu フォント（fonts-dejavu-core）が必要です") from None

    def noisy(size: int, color: Sequence[int], amp: float) -> "np.ndarray":
        base = np.ones((size, size, 3), dtype=np.float32) * np.array(color, dtype=np.float32)
        return base + rng.normal(0.0, amp, (size, size, 1)).astype(np.float32)

    def to_img(arr: "np.ndarray") -> "Image.Image":
        return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8), "RGB")

    def tiles(name: str, color: Sequence[int], grout: Sequence[int], n: int, amp: float) -> None:
        size = 512
        arr = noisy(size, color, amp)
        step = size // n
        for i in range(n):
            for j in range(n):
                arr[i * step:(i + 1) * step, j * step:(j + 1) * step] += rng.normal(0.0, 5.0)
        for k in range(0, size, step):
            arr[max(k - 1, 0):k + 2, :, :] = grout
            arr[:, max(k - 1, 0):k + 2, :] = grout
        save(to_img(arr), name)

    # 乗場の床（階ごとに変える）とかごの床
    tiles("floor_1f", (196, 186, 166), (120, 114, 104), 8, 4.0)     # ベージュの石調タイル
    tiles("floor_2f", (150, 152, 154), (96, 98, 100), 8, 5.0)       # グレーのタイル
    tiles("floor_3f", (78, 100, 138), (58, 74, 102), 8, 9.0)        # 青のタイルカーペット
    for extra in range(4, p.num_floors + 1):
        tiles(f"floor_{extra}f", (170, 160, 150), (110, 104, 98), 8, 5.0)
    save(to_img(noisy(256, (72, 74, 78), 10.0)), "car_floor")

    # ステンレス（縦のヘアライン）
    steel = np.ones((256, 256, 3), dtype=np.float32) * 205.0
    steel += rng.normal(0.0, 7.0, (1, 256, 1)).astype(np.float32)
    steel += rng.normal(0.0, 1.5, (256, 256, 1)).astype(np.float32)
    save(to_img(steel), "door_steel")
    wall = np.ones((256, 256, 3), dtype=np.float32) * 200.0
    wall += rng.normal(0.0, 4.0, (1, 256, 1)).astype(np.float32)
    for k in (0, 85, 170, 255):      # パネルの目地
        wall[:, max(k - 1, 0):k + 1, :] = 150.0
    save(to_img(wall), "car_wall")

    # 階の札と、扉の上の階数表示
    for floor in p.floors():
        img = Image.new("RGB", (512, 512), (24, 44, 92))
        d = ImageDraw.Draw(img)
        d.rectangle([14, 14, 497, 497], outline=(255, 255, 255), width=8)
        d.text((256, 236), f"{floor}F", fill=(255, 255, 255), font=font(250), anchor="mm")
        d.text((256, 430), "ELEVATOR", fill=(255, 255, 255), font=font(54), anchor="mm")
        save(img, f"sign_{floor}f")
        ind = Image.new("RGB", (256, 112), (12, 12, 12))
        d = ImageDraw.Draw(ind)
        d.text((128, 54), f"{floor}", fill=(255, 150, 20), font=font(92), anchor="mm")
        d.polygon([(34, 74), (58, 74), (46, 34)], fill=(255, 150, 20))
        save(ind, f"indicator_{floor}f")

    # 乗場の呼びボタンと、かごの操作盤
    call = to_img(noisy(256, (198, 200, 202), 3.0)).resize((120, 280))
    d = ImageDraw.Draw(call)
    for cy, up in ((95, True), (190, False)):
        d.ellipse([34, cy - 26, 86, cy + 26], fill=(40, 40, 44), outline=(120, 120, 124), width=3)
        tri = [(60, cy - 14), (46, cy + 10), (74, cy + 10)] if up else [(60, cy + 14), (46, cy - 10), (74, cy - 10)]
        d.polygon(tri, fill=(240, 240, 240))
    save(call, "call_panel")
    cop = to_img(noisy(256, (198, 200, 202), 3.0)).resize((180, 500))
    d = ImageDraw.Draw(cop)
    d.rectangle([30, 26, 150, 96], fill=(12, 12, 12))
    for i in range(6):
        cx, cy = 60 + 60 * (i % 2), 160 + 70 * (i // 2)
        d.ellipse([cx - 20, cy - 20, cx + 20, cy + 20], fill=(235, 235, 235), outline=(90, 90, 94), width=3)
        d.text((cx, cy), str(i + 1), fill=(30, 30, 30), font=font(22), anchor="mm")
    for cx, label in ((60, "<>"), (120, "><")):
        d.ellipse([cx - 20, 400 - 20, cx + 20, 400 + 20], fill=(235, 235, 235), outline=(90, 90, 94), width=3)
        d.text((cx, 400), label, fill=(30, 30, 30), font=font(18), anchor="mm")
    save(cop, "cop_panel")
    return written


# ---------------------------------------------------------------------------
def generate(pkg_dir: str, p: Optional[WorldParams] = None, r: Optional[RobotParams] = None,
             textures: bool = True) -> List[str]:
    """パッケージのソースの下に、世界・モデル・設定を書き出す. 書いたファイルの一覧を返す."""
    p = p or WorldParams()
    r = r or RobotParams()
    written: List[str] = []

    def write(rel: str, text: str) -> None:
        path = os.path.join(pkg_dir, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        written.append(path)

    write(os.path.join("worlds", "elevator.sdf"), world_sdf(p))
    write(os.path.join("models", p.robot_model, "model.sdf"), robot_sdf(p, r))
    write(os.path.join("models", p.robot_model, "model.config"),
          model_config(p.robot_model, "Differential-drive robot with one forward RGB camera and wheel odometry."))
    write(os.path.join("models", TEX_MODEL, "model.sdf"), texture_model_sdf())
    write(os.path.join("models", TEX_MODEL, "model.config"),
          model_config(TEX_MODEL, "Procedurally generated textures for the elevator world."))
    write(os.path.join("config", "bridge.yaml"), bridge_yaml(p))
    if textures:
        written += generate_textures(os.path.join(pkg_dir, "models", TEX_MODEL, "materials", "textures"), p)
    return written


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="エレベーターの世界（SDF）、ロボット、テクスチャ、ブリッジ設定を生成する")
    ap.add_argument("--pkg-dir", default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    help="elevator_sim パッケージのソースのディレクトリ")
    ap.add_argument("--no-textures", action="store_true", help="テクスチャを作り直さない")
    args = ap.parse_args(argv)
    for path in generate(args.pkg_dir, textures=not args.no_textures):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
