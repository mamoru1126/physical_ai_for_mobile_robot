#!/usr/bin/env python3
"""Orin から見た、つながりの確かめ. jetson/robot/check_link.sh が、実行系のイメージの中で動かす.

受けるだけで、速度の指令は出さない（--zero のときだけ、安全ゲートの入力に速度 0 を 3 秒出す）.
最終の速度指令のトピック（/cmd_vel）は購読しない. 橋渡しが dynamic_bridge だと、ROS 1 の側の
/cmd_vel を ROS 2 に引き込んでしまうため（誰が出し、誰が受けているかだけを見る）.
"""
import argparse
import json
import math
import os
import statistics
import sys
import threading
import time

import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

ME = "link_check"
RELAY_TOPIC = "/vlm_cmd_vel"
RESULTS = []


def report(level, title, detail="", hints=()):
    """level: OK / NG / 注意 / --（確かめなかった）"""
    RESULTS.append(level)
    mark = {"OK": "[OK]  ", "NG": "[NG]  ", "注意": "[注意]", "--": "[--]  "}[level]
    print(f"{mark} {title}" + (f": {detail}" if detail else ""))
    for h in hints:
        print(f"         → {h}")


def load_yaml(path):
    try:
        with open(path, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


def section(cfg, name):
    v = cfg.get(name) if isinstance(cfg, dict) else None
    return v if isinstance(v, dict) else {}


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Probe:
    """1 つのトピックの受け具合（この PC の時計で数える）."""

    def __init__(self, topic):
        self.topic = topic
        self.times = []
        self.offsets = []
        self.last = None
        self.lock = threading.Lock()

    def add(self, msg, stamp=None):
        now = time.time()
        with self.lock:
            self.times.append(now)
            self.last = msg
            if stamp is not None and stamp > 0:
                self.offsets.append(now - stamp)

    def count(self):
        with self.lock:
            return len(self.times)

    def rate(self):
        with self.lock:
            if len(self.times) < 2:
                return 0.0
            span = self.times[-1] - self.times[0]
            return (len(self.times) - 1) / span if span > 0 else 0.0


class LinkCheck(Node):
    def __init__(self, args):
        super().__init__(ME)
        task = load_yaml(args.task)
        gate = load_yaml(args.gate)
        cam = section(task, "camera")
        drive = section(task, "drive")
        topics = section(gate, "topics")
        frames = section(gate, "frames")
        self.image_topic = str(cam.get("image_topic", "/camera/compressed"))
        self.info_topic = str(cam.get("info_topic", "/camera/camera_info"))
        self.hfov = float(cam.get("hfov_deg", 90.0))
        self.odom_topic = args.odom
        self.cloud_topic = str(topics.get("cloud", "/cloud"))
        self.cmd_in = str(topics.get("cmd_in", RELAY_TOPIC))
        self.cmd_out = str(topics.get("cmd_out", "/cmd_vel"))
        self.gate_state_topic = str(topics.get("state", "/safety_gate/state"))
        self.gate_debug_topic = str(topics.get("debug", "/safety_gate/debug"))
        self.base_frame = str(frames.get("base", "base_link") or "base_link")
        self.cloud_frame_cfg = str(frames.get("cloud", "") or "")
        self.publish_static = bool(section(gate, "tf").get("publish_static", False))
        self.drive_cmd = str(drive.get("command_topic", "/vlm_drive/command") or "")
        self.drive_state = str(drive.get("state_topic", "/vlm_drive/state") or "")
        self.drive_active = str(drive.get("active_topic", "/vlm_drive/active") or "")

        self.p_image = Probe(self.image_topic)
        self.p_info = Probe(self.info_topic)
        self.p_odom = Probe(self.odom_topic)
        self.p_cloud = Probe(self.cloud_topic)
        self.p_tf = Probe("/tf")
        self.p_tf_static = Probe("/tf_static")
        self.p_gate_state = Probe(self.gate_state_topic)
        self.p_gate_debug = Probe(self.gate_debug_topic)
        self.p_drive = Probe(self.drive_state)
        self.tf_pairs = {}
        self.cloud_frames = set()
        self.cloud_points = 0

        sensor = qos_profile_sensor_data
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tf_static_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                   reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.image_topic.rstrip("/").endswith("compressed"):
            self.create_subscription(CompressedImage, self.image_topic,
                                     lambda m: self.p_image.add(m, stamp_sec(m.header)), sensor)
        else:
            self.create_subscription(Image, self.image_topic,
                                     lambda m: self.p_image.add(m, stamp_sec(m.header)), sensor)
        self.create_subscription(CameraInfo, self.info_topic, lambda m: self.p_info.add(m), sensor)
        self.create_subscription(Odometry, self.odom_topic,
                                 lambda m: self.p_odom.add(m, stamp_sec(m.header)), sensor)
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, sensor)
        self.create_subscription(TFMessage, "/tf", lambda m: self._on_tf(m, self.p_tf), 100)
        self.create_subscription(TFMessage, "/tf_static", lambda m: self._on_tf(m, self.p_tf_static), tf_static_qos)
        self.create_subscription(String, self.gate_state_topic, lambda m: self.p_gate_state.add(m), 10)
        self.create_subscription(String, self.gate_debug_topic, lambda m: self.p_gate_debug.add(m), 10)
        if self.drive_state:
            self.create_subscription(String, self.drive_state, lambda m: self.p_drive.add(m), latched)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_cloud(self, msg):
        self.p_cloud.add(msg, stamp_sec(msg.header))
        self.cloud_frames.add(msg.header.frame_id.lstrip("/"))
        self.cloud_points = int(msg.width) * int(msg.height)

    def _on_tf(self, msg, probe):
        probe.add(msg)
        for t in msg.transforms:
            self.tf_pairs[t.child_frame_id.lstrip("/")] = t.header.frame_id.lstrip("/")

    # -- グラフ（誰が出し、誰が受けているか） ------------------------------------------------
    def endpoints(self, topic, kind):
        try:
            infos = (self.get_publishers_info_by_topic(topic) if kind == "pub"
                     else self.get_subscriptions_info_by_topic(topic))
        except Exception:  # noqa: BLE001
            return []
        return sorted(i.node_name for i in infos if i.node_name != ME)


def names(items):
    return "、".join(items) if items else "なし"


def check_rate(probe, title, min_hz, ng_hints, node, extra=""):
    pubs = node.endpoints(probe.topic, "pub")
    if probe.count() == 0:
        report("NG", f"{title} {probe.topic}", "届いていません（出しているもの: " + names(pubs) + "）", ng_hints)
        return False
    hz = probe.rate()
    detail = f"{hz:.1f} Hz（出しているもの: {names(pubs)}）" + extra
    if len(pubs) > 1:
        report("注意", f"{title} {probe.topic}", detail,
               ["出しているものが 2 つ以上あります. 橋渡しが 2 つ動いていないか、同じ名前で出すノードがないか"])
    elif hz < min_hz:
        report("注意", f"{title} {probe.topic}", detail + f"（{min_hz:g} Hz より遅い）")
    else:
        report("OK", f"{title} {probe.topic}", detail)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--gate", required=True)
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--zero", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = LinkCheck(args)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    print(f"ROS 2 のドメイン {domain} で {args.seconds:.0f} 秒受けます"
          "（橋渡しが dynamic_bridge なら、こちらが購読してから橋渡しが始まるので少しかかります）", flush=True)
    end = time.time() + args.seconds
    while time.time() < end:
        time.sleep(1.0)
        print(".", end="", flush=True)
    print("\n")

    # ---- 1. Orin の中（カメラ） ----
    print("== カメラ（Orin の中）==")
    check_rate(node.p_image, "カメラ画像", 5.0,
               ["カメラのサービス: systemctl status usb-camera（「カメラがつながるのを待っています」なら USB を挿し直す）",
                "映像を見る: ブラウザで http://<OrinのIP>:9000/",
                "名前が違えば /etc/elevator-task/task.yaml の camera.image_topic と /etc/usb-camera/camera.env をそろえる"],
               node)
    info = node.p_info.last
    if info is None:
        report("注意", f"カメラ情報 {node.info_topic}", f"届いていません. 画角は設定の hfov_deg（{node.hfov:g} 度）を使います",
               ["/etc/elevator-task/task.yaml の camera.hfov_deg を、カメラの水平画角に合わせる"])
    elif float(info.k[0]) <= 0.0:
        report("注意", f"カメラ情報 {node.info_topic}",
               f"{info.width}x{info.height}、校正なし. 画角は設定の hfov_deg（{node.hfov:g} 度）を使います",
               ["/etc/elevator-task/task.yaml の camera.hfov_deg を、カメラの水平画角に合わせる（仕様書の値）"])
    else:
        hfov = math.degrees(2.0 * math.atan(info.width / (2.0 * float(info.k[0]))))
        report("OK", f"カメラ情報 {node.info_topic}", f"{info.width}x{info.height}、校正あり（水平画角 {hfov:.0f} 度）")

    # ---- 2. ロボットの PC から（橋渡し） ----
    print("\n== ロボットの PC から（ROS 1 → 橋渡し → Orin）==")
    if check_rate(node.p_odom, "オドメトリ", 5.0,
                  ["ロボットの PC で: rostopic hz " + node.odom_topic,
                   "名前が違えば /etc/elevator-task/robot.env の ODOM_TOPIC（橋渡しは名前を変えられない）",
                   "ROS 1 では出ているのに届かないなら、橋渡し（jetson/robot/link_check.md の 2〜4）"], node):
        m = node.p_odom.last
        off = statistics.median(node.p_odom.offsets) if node.p_odom.offsets else 0.0
        print(f"         座標系 {m.header.frame_id or '（空）'} → {m.child_frame_id or '（空）'}、"
              f"時計のずれ（Orin − ロボットの PC）{off:+.2f} 秒")
        if abs(off) > 0.5:
            report("注意", "時計", f"ロボットの PC と {off:+.2f} 秒ずれています",
                   ["実行系はずれを直して使うので動きますが、両方の時計を合わせるのが確実（chrony など）"])
    if check_rate(node.p_cloud, "点群", 2.0,
                  ["ロボットの PC で: rostopic hz " + node.cloud_topic,
                   "名前が違えば /etc/safety-gate/config.yaml の topics.cloud（そのあと sudo systemctl restart safety-gate）"],
                  node):
        print(f"         座標系 {names(sorted(node.cloud_frames))}、{node.cloud_points} 点")
    tf_n, st_n = node.p_tf.count(), node.p_tf_static.count()
    if tf_n == 0 and st_n == 0:
        report("注意", "TF（/tf、/tf_static）", "届いていません")
    else:
        pairs = [f"{p}→{c}" for c, p in sorted(node.tf_pairs.items())]
        more = f" ほか {len(pairs) - 12}" if len(pairs) > 12 else ""
        report("OK", "TF（/tf、/tf_static）", f"/tf {tf_n} 回、/tf_static {st_n} 回. " + "、".join(pairs[:12]) + more)
    cloud_frame = node.cloud_frame_cfg or (sorted(node.cloud_frames)[0] if node.cloud_frames else "")
    if not cloud_frame:
        report("--", f"TF {node.base_frame} ← 点群の座標系", "点群が届かないので確かめられません")
    else:
        try:
            t = node.tf_buffer.lookup_transform(node.base_frame, cloud_frame, Time())
            tr, q = t.transform.translation, t.transform.rotation
            roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            report("OK", f"TF {node.base_frame} ← {cloud_frame}",
                   f"x {tr.x:.2f} y {tr.y:.2f} z {tr.z:.2f} [m]、roll {roll:.0f} pitch {pitch:.0f} yaw {yaw:.0f} [度]"
                   + ("（安全ゲートが自分で出している: tf.publish_static: true）" if node.publish_static else ""))
        except Exception as e:  # noqa: BLE001
            report("NG", f"TF {node.base_frame} ← {cloud_frame}", f"取れません（{str(e)[:100]}）",
                   [f"ロボットの PC で: rosrun tf tf_echo {node.base_frame} {cloud_frame}",
                    "ROS 1 に TF があるのに届かないなら橋渡しの TF（/tf、/tf_static）",
                    "どこにも TF がないなら /etc/safety-gate/config.yaml の tf: に publish_static: true と、"
                    "取り付けの xyz・rpy を書く（そのあと sudo systemctl restart safety-gate）"])

    # ---- 3. 安全ゲート ----
    print("\n== 安全ゲート（safety-gate）==")
    state = node.p_gate_state.last
    if state is None:
        report("NG", f"安全ゲートの状態 {node.gate_state_topic}", "届いていません",
               ["systemctl status safety-gate、journalctl -u safety-gate -n 30",
                "ROS 2 のドメインが 3 か所でそろっているか（bash jetson/robot/set_domain.sh）"])
    else:
        report("OK", f"安全ゲートの状態 {node.gate_state_topic}", state.data[:80])
    dbg = None
    if node.p_gate_debug.last is not None:
        try:
            dbg = json.loads(node.p_gate_debug.last.data)
        except ValueError:
            dbg = None
    if dbg is not None:
        if dbg.get("age_cloud") is None:
            report("NG", "安全ゲートから見た点群", "使えていません（点群が来ないか、TF がない）. このままでは走りません",
                   ["上の「点群」と「TF」を直す. 理由は journalctl -u safety-gate -n 30"])
        else:
            fp = dbg.get("fp") or []
            xs = [p[0] for p in fp] or [0.0]
            ys = [p[1] for p in fp] or [0.0]
            report("OK", "安全ゲートから見た点群",
                   f"{dbg.get('nvox', 0)} 個の障害物のます、座標系 {' ← '.join(dbg.get('frames') or [])}")
            print(f"         足形: 前 {max(xs):+.2f} 後 {min(xs):+.2f} 左 {max(ys):+.2f} 右 {min(ys):+.2f} [m]"
                  f"（余裕 {dbg.get('margin', 0):.2f} m を足して見る）")

    # ---- 4. 速度の通り道 ----
    print("\n== 速度の通り道（実行系 → 安全ゲート → 橋渡し → ロボット）==")
    in_subs, in_pubs = node.endpoints(node.cmd_in, "sub"), node.endpoints(node.cmd_in, "pub")
    if node.cmd_in != RELAY_TOPIC:
        report("NG", f"安全ゲートの入力 {node.cmd_in}", f"実行系は {RELAY_TOPIC} に出します. 安全ゲートの topics.cmd_in を {RELAY_TOPIC} に")
    elif not in_subs:
        report("NG", f"安全ゲートの入力 {node.cmd_in}", "受けているものがありません（安全ゲートが動いていない）")
    else:
        report("OK", f"安全ゲートの入力 {node.cmd_in}", f"受ける: {names(in_subs)}、出す: {names(in_pubs)}")
    out_subs, out_pubs = node.endpoints(node.cmd_out, "sub"), node.endpoints(node.cmd_out, "pub")
    detail = f"受ける: {names(out_subs)}、出す: {names(out_pubs)}"
    if not out_subs:
        report("NG", f"⚠ 最終の速度指令 {node.cmd_out}", detail + ". ロボットへ届きません",
               ["ロボットの PC で: rostopic info " + node.cmd_out + "（Subscribers に台車のドライバがいるか）",
                "台車が別の名前で受けるなら /etc/safety-gate/config.yaml の topics.cmd_out をその名前に",
                "dynamic_bridge は、ROS 1 の側に受けるものがいるときだけ橋渡しします"])
    elif [p for p in out_pubs if p != "safety_gate_node"]:
        report("注意", f"⚠ 最終の速度指令 {node.cmd_out}", detail,
               ["安全ゲートのほかにも出しているものがあります. 安全ゲートを通らない指令が混ざらないか確かめる"])
    else:
        report("OK", f"⚠ 最終の速度指令 {node.cmd_out}", detail)

    # ---- 5. 実行系と、ROS1システムとの合図 ----
    print("\n== 実行系と、ROS1システムとの合図 ==")
    task_running = bool(node.drive_state and node.endpoints(node.drive_state, "pub"))
    if not task_running:
        report("--", "実行系（elevator-task）", "動いていないので確かめません. bash jetson/robot/robot_up.sh の後でもう一度")
    else:
        d = node.p_drive.last
        if d is None:
            report("NG", f"知らせ {node.drive_state}", "届いていません（実行系のログ: docker logs --tail 40 elevator-task）")
        else:
            try:
                s = json.loads(d.data)
                text = f"task_state {s.get('task_state')}、承認 {s.get('armed')}、走っている {s.get('active')}"
                last = s.get("last_command")
                if last:
                    text += f"、最後の合図 {last.get('command')}（{last.get('message')}）"
            except ValueError:
                text = d.data[:80]
            subs = node.endpoints(node.drive_state, "sub")
            report("OK", f"知らせ {node.drive_state}", text)
            print(f"         受けているもの: {names(subs)}（dynamic_bridge なら、ROS 1 の側で受けるときだけ橋渡しが受ける）")
        cmd_subs, cmd_pubs = node.endpoints(node.drive_cmd, "sub"), node.endpoints(node.drive_cmd, "pub")
        if "elevator_task" not in cmd_subs:
            report("NG", f"合図 {node.drive_cmd}", f"実行系が受けていません（受ける: {names(cmd_subs)}）")
        else:
            report("OK", f"合図 {node.drive_cmd}", f"受ける: {names(cmd_subs)}、出す: {names(cmd_pubs)}")
            print("         ROS 1 の側から届くかは、ロボットの PC で python3 vlm_drive.py ping（link_check.md の 6）")

    # ---- 6. 橋渡し ----
    print("\n== 橋渡し（ROS 2 の側から見えるもの）==")
    all_nodes = [n for n, _ in node.get_node_names_and_namespaces() if n != ME]
    bridges = [n for n in all_nodes if "bridge" in n]      # 同じ名前が 2 つあれば 2 つ数える
    print(f"         ROS 2 のノード: {names(sorted(set(all_nodes)))}")
    if not bridges:
        report("注意", "橋渡し", "名前に bridge を含むノードが見えません",
               ["ロボットの PC で橋渡しが動いているか、ROS 2 のドメインとネットワーク（link_check.md の 2〜4）"])
    elif len(bridges) > 1:
        report("注意", "橋渡し", f"{names(sorted(bridges))}（2 つ以上動いていないか. "
               "同じトピックを両方で橋渡しすると、二重やループになる）")
    else:
        report("OK", "橋渡し", bridges[0])

    # ---- 7. 速度 0 を流す（--zero） ----
    if args.zero:
        print("\n== 速度 0 を 3 秒流す ==")
        active = False
        if node.p_drive.last is not None:
            try:
                active = bool(json.loads(node.p_drive.last.data).get("active"))
            except ValueError:
                active = False
        if active:
            report("--", "速度 0", "VLM の走行中なので流しません")
        else:
            pub = node.create_publisher(Twist, node.cmd_in, 10)
            seen = False
            for _ in range(30):
                pub.publish(Twist())
                time.sleep(0.1)
                last = node.p_gate_debug.last
                if last is not None:
                    try:
                        seen = seen or bool(json.loads(last.data).get("pub"))
                    except ValueError:
                        pass
            if seen:
                report("OK", "速度 0", f"安全ゲートが {node.cmd_out} に 0 を出しました",
                       [f"ロボットの PC の rostopic echo {node.cmd_out} に 0 が出ていれば、速度の通り道はつながっています"])
            else:
                report("NG", "速度 0", "安全ゲートが出しませんでした（安全ゲートのログ: journalctl -u safety-gate -n 30）")

    ng = RESULTS.count("NG")
    warn = RESULTS.count("注意")
    print("\n" + ("すべて OK" if not ng and not warn else f"NG {ng} 件、注意 {warn} 件"))
    node.destroy_node()
    rclpy.try_shutdown()
    # 1 と 2 は Python の例外や引数の誤りで使われるので、NG は 3 にする
    return 3 if ng else 0


if __name__ == "__main__":
    sys.exit(main())
