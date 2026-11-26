#!/usr/bin/env python3
"""安全ゲートの WebUI. 安全ゲートが出す /safety_gate/debug（JSON）を、ブラウザに流す.

  GET /         画面（上から見た 2D 表示）
  GET /events   新しい情報を流し続ける（Server-Sent Events）
  GET /latest   いちばん新しい情報（JSON）. curl で確かめる用

ブラウザに送るのは、安全ゲートが作った JSON そのまま（数値は小数 2 桁、点は表示のマスの番号を
2 バイトの整数にして base64 にしたもの）. ここでは手を加えない.
"""
import argparse
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class Latest:
    """いちばん新しい情報と、その番号. 新しい情報が来たら、待っている送り手を起こす."""

    def __init__(self):
        self.cond = threading.Condition()
        self.data = None
        self.seq = 0

    def put(self, data):
        with self.cond:
            self.data = data
            self.seq += 1
            self.cond.notify_all()

    def wait_newer(self, seq, timeout):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != seq, timeout=timeout)
            return self.seq, self.data


def make_handler(latest, page):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # アクセスのたびに出さない
            pass

        def _send(self, code, body, ctype):
            data = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, page, "text/html; charset=utf-8")
            elif path == "/latest":
                self._send(200, latest.data or "{}", "application/json")
            elif path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        new_seq, data = latest.wait_newer(seq, timeout=10.0)
                        if new_seq == seq or data is None:
                            self.wfile.write(b": keepalive\n\n")  # 何も来なくても、つながりを保つ
                        else:
                            seq = new_seq
                            self.wfile.write(b"data: " + data.encode("utf-8") + b"\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self._send(404, "not found", "text/plain")

    return Handler


class WebNode(Node):
    def __init__(self, topic, latest):
        super().__init__("safety_gate_web")
        self.create_subscription(String, topic, lambda m: latest.put(m.data), 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/config/config.yaml")
    args, ros_args = parser.parse_known_args()
    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    web = config.get("web") or {}
    port = int(web.get("port", 9001))
    topic = (config.get("topics") or {}).get("debug", "/safety_gate/debug")

    share = get_package_share_directory("safety_gate")
    with open(os.path.join(share, "web", "index.html"), encoding="utf-8") as f:
        page = f.read()

    latest = Latest()
    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(latest, page))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    rclpy.init(args=ros_args)
    node = WebNode(topic, latest)
    node.get_logger().info(f"WebUI: http://<この PC の IP>:{port}/ （{topic} を表示）")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
