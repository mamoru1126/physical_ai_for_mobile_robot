#!/usr/bin/env python3
"""ROS 1 の roscore とやりとりする小さな道具（Python の標準のものだけで動く）.

  ros1_master.py find auto          同じネットワークの roscore を探して、URI を出す
  ros1_master.py find http://...    その roscore が答えるか確かめて、URI を出す
  ros1_master.py check URI          roscore が答えれば 0 で終わる
  ros1_master.py set-topics URI ODOM CLOUD CMD_VEL [TF [名前=型 ...]]
                                    parameter_bridge が読む /topics を roscore に書く.
                                    TF が true なら /tf と /tf_static も橋渡しする.
                                    名前=型（例 /vlm_drive/command=std_msgs/msg/String）も足せる
  ros1_master.py fix-hosts URI      名前（ホスト名）が引けないノードを、roscore と同じ PC とみなして
                                    /etc/hosts に書き足す. 書き足した名前を出す
"""
import concurrent.futures
import ipaddress
import socket
import subprocess
import sys
import urllib.parse
import xmlrpc.client

PORT = 11311
CALLER = "/ros1_bridge_setup"
HOSTS = "/etc/hosts"
SKIP_IFACES = ("lo", "docker", "br-", "veth", "virbr", "tun", "wg")


def master(uri, timeout=3.0):
    socket.setdefaulttimeout(timeout)
    return xmlrpc.client.ServerProxy(uri)


def own_uri(uri):
    """roscore が答えれば、roscore が名乗る自分の URI を返す. 答えなければ None."""
    try:
        code, _, name = master(uri).getUri(CALLER)
        return name if code == 1 else None
    except Exception:
        return None


def answers(uri):
    return own_uri(uri) is not None


def port_open(ip, timeout=0.3):
    try:
        with socket.create_connection((str(ip), PORT), timeout=timeout):
            return True
    except OSError:
        return False


def candidate_ips():
    """この PC のネットワーク（Docker などは除く）にある IP. 広すぎるネットワークは自分のまわりの /24 だけ."""
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True).stdout
    except OSError:
        out = ""
    ips = [ipaddress.ip_address("127.0.0.1")]
    for line in out.splitlines():
        parts = line.split()
        iface, cidr = parts[1], parts[3]
        if iface.startswith(SKIP_IFACES):
            continue
        net = ipaddress.ip_interface(cidr).network
        if net.prefixlen < 24:
            net = ipaddress.ip_interface(f"{cidr.split('/')[0]}/24").network
        for ip in net.hosts():
            if ip not in ips:
                ips.append(ip)
    return ips


def find_auto():
    ips = candidate_ips()
    with concurrent.futures.ThreadPoolExecutor(max_workers=128) as pool:
        opened = [ip for ip, ok in zip(ips, pool.map(port_open, ips)) if ok]
    # 同じ roscore に別の IP で届くことがあるので、roscore が名乗る URI でまとめる
    found, seen = [], set()
    for ip in opened:
        uri = f"http://{ip}:{PORT}"
        name = own_uri(uri)
        if name and name not in seen:
            seen.add(name)
            found.append(uri)
    if len(found) > 1:
        print("注意: roscore が複数あります: " + " ".join(found) + "（最初のものを使います. "
              "違うなら ROS_MASTER_URI を書いてください）", file=sys.stderr)
    return found[0] if found else None


def extra_topics(specs):
    """「名前=型」の並びを /topics の項目にする. 名前か型が空のものは飛ばす."""
    out = []
    for spec in specs:
        name, _, kind = str(spec).partition("=")
        name, kind = name.strip(), kind.strip()
        if name and kind:
            out.append({"topic": name, "type": kind, "queue_size": 10})
    return out


def set_topics(uri, odom, cloud, cmd_vel, tf="true", *extra):
    topics = [
        {"topic": odom, "type": "nav_msgs/msg/Odometry", "queue_size": 10},
        {"topic": cloud, "type": "sensor_msgs/msg/PointCloud2", "queue_size": 2},
        {"topic": cmd_vel, "type": "geometry_msgs/msg/Twist", "queue_size": 1},
    ]
    topics += extra_topics(extra)
    if str(tf).lower() in ("1", "true", "yes", "on"):
        # /tf_static は parameter_bridge が自動で「全部とっておき、あとから来た相手にも渡す」
        # （keep all、transient local）にする
        topics += [
            {"topic": "/tf", "type": "tf2_msgs/msg/TFMessage", "queue_size": 100},
            {"topic": "/tf_static", "type": "tf2_msgs/msg/TFMessage", "queue_size": 100},
        ]
    code, msg, _ = master(uri).setParam(CALLER, "/topics", topics)
    if code != 1:
        raise SystemExit(f"NG: /topics を書けません: {msg}")


def resolvable(host):
    try:
        socket.getaddrinfo(host, None, socket.AF_INET)
        return True
    except OSError:
        return False


def fix_hosts(uri):
    m = master(uri)
    master_host = urllib.parse.urlparse(uri).hostname
    master_ip = socket.gethostbyname(master_host)
    _, _, state = m.getSystemState(CALLER)
    nodes = {n for group in state for _, members in group for n in members}
    with open(HOSTS) as f:
        known = {name for line in f if not line.lstrip().startswith("#") for name in line.split()[1:]}
    added = []
    for node in sorted(nodes):
        code, _, node_uri = m.lookupNode(CALLER, node)
        host = urllib.parse.urlparse(node_uri).hostname if code == 1 else None
        if not host or host in added or host in known or resolvable(host):
            continue
        with open(HOSTS, "a") as f:
            f.write(f"{master_ip} {host}\n")
        added.append(host)
    for host in added:
        print(host)


def main(argv):
    if len(argv) < 2:
        raise SystemExit(__doc__)
    cmd = argv[1]
    if cmd == "find":
        want = argv[2] if len(argv) > 2 else "auto"
        uri = find_auto() if want in ("", "auto") else (want if answers(want) else None)
        if not uri:
            return 1
        print(uri)
        return 0
    if cmd == "check":
        return 0 if answers(argv[2]) else 1
    if cmd == "set-topics":
        set_topics(*argv[2:])
        return 0
    if cmd == "fix-hosts":
        fix_hosts(argv[2])
        return 0
    raise SystemExit(__doc__)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
