#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ロボットの PC（ROS 1）から、VLM の走行（Orin の実行系）に合図を出す・状態を見る.

  python3 vlm_drive.py ping           届くかを確かめる（何もしない）. 答えが返れば両方向がつながっている
  python3 vlm_drive.py start          画面で「承認して開始の合図を待つ」にした計画を走らせる
  python3 vlm_drive.py start 2 3      手順 2〜3 だけ走らせる
  python3 vlm_drive.py finish         ここまで（その場で止めて、成功として終える）
  python3 vlm_drive.py stop           停止（失敗として止める. 画面で解除するまで動かない）
  python3 vlm_drive.py state          状態を出し続ける（Ctrl+C で終わる）

ROS 1 の環境で動かす（source /opt/ros/noetic/setup.bash）. このファイルだけをロボットの PC に置けばよい.
rostopic pub -1 と違い、橋渡しが合図を受ける準備（購読）ができるまで待ってから出し、Orin からの答え
（/vlm_drive/state の last_command）を待って表示する. 橋渡しが dynamic_bridge のときは、出す側が
できてから橋渡しが始まるので、rostopic pub -1 だと最初の合図を落とすことがある.

環境変数でトピックを変えられる: VLM_COMMAND_TOPIC（既定 /vlm_drive/command）、VLM_STATE_TOPIC（既定 /vlm_drive/state）
"""
import json
import os
import sys
import threading
import time

import rospy
from std_msgs.msg import String

COMMAND_TOPIC = os.environ.get("VLM_COMMAND_TOPIC", "/vlm_drive/command")
STATE_TOPIC = os.environ.get("VLM_STATE_TOPIC", "/vlm_drive/state")
COMMANDS = ("ping", "start", "finish", "stop")


class State(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None
        self.count = 0

    def on_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.data = data
            self.count += 1

    def get(self):
        with self.lock:
            return self.data, self.count


def summary(s):
    if not s:
        return "（なし）"
    text = "走っている %s、状態 %s" % ("はい" if s.get("active") else "いいえ", s.get("task_state"))
    if s.get("armed"):
        text += "、承認（手順 %s〜%s）" % tuple(s["armed"])
    if s.get("active") and s.get("step"):
        text += "、手順 %s %s" % (s.get("step"), s.get("skill", ""))
    if s.get("traveled_m") is not None:
        text += "、%.1f / %s m" % (s["traveled_m"], s.get("limit_m"))
    if s.get("result"):
        text += "、前回の終わり方 %s" % s["result"]
    return text


def watch(state):
    last = None
    while not rospy.is_shutdown():
        s, _ = state.get()
        if s is not None:
            line = summary(s)
            if line != last:
                print(time.strftime("%H:%M:%S"), line)
                sys.stdout.flush()
                last = line
        time.sleep(0.2)


def send(state, payload, wait_s=15.0):
    pub = rospy.Publisher(COMMAND_TOPIC, String, queue_size=1)
    # 橋渡しが購読するまで待つ（dynamic_bridge は、こちらが出す側になってから橋渡しを始める）
    end = time.time() + wait_s
    while pub.get_num_connections() == 0 and time.time() < end and not rospy.is_shutdown():
        time.sleep(0.1)
    if pub.get_num_connections() == 0:
        print("NG: %s を受けるものがいません（橋渡しが動いていないか、このトピックを橋渡ししていない）" % COMMAND_TOPIC)
        return 3
    # ROS 2 の側で、橋渡しと実行系がつながるのを少し待つ. その間に、今の知らせ（前の答え）も受けておく
    time.sleep(2.0)
    end = time.time() + 5.0
    while state.get()[0] is None and time.time() < end and not rospy.is_shutdown():
        time.sleep(0.1)
    before = state.get()[0]
    if before is None:
        print("注意: Orin からの知らせ（%s）が届きません（ROS 2 → ROS 1 の橋渡し）. 答えは確かめられません" % STATE_TOPIC)
    before_cmd = (before or {}).get("last_command")
    pub.publish(String(data=payload))
    print("出しました: %s %s" % (COMMAND_TOPIC, payload))
    if before is None:
        time.sleep(1.0)
        return 3
    end = time.time() + 6.0
    while time.time() < end and not rospy.is_shutdown():
        s = state.get()[0]
        cmd = (s or {}).get("last_command")
        if cmd and cmd != before_cmd:
            ok = bool(cmd.get("ok"))
            print("%s: %s（%s）" % ("OK" if ok else "受け付けられませんでした", cmd.get("message"), cmd.get("command")))
            time.sleep(0.5)
            print("状態:", summary(state.get()[0]))
            return 0 if ok else 4
        time.sleep(0.1)
    print("注意: 合図は出しましたが、答えが返りません（ROS 1 → ROS 2 の橋渡しか、実行系が古い. 実行系の画面の「外からの合図」も見る）")
    return 3


def main():
    args = [a for a in sys.argv[1:] if a]
    if not args or args[0] in ("-h", "--help") or args[0] not in COMMANDS + ("state",):
        print(__doc__)
        return 0 if args and args[0] in ("-h", "--help") else 2
    name = args[0]
    rospy.init_node("vlm_drive_cmd", anonymous=True)
    state = State()
    rospy.Subscriber(STATE_TOPIC, String, state.on_msg, queue_size=1)
    if name == "state":
        print("状態を出します（%s. Ctrl+C で終わる）" % STATE_TOPIC)
        watch(state)
        return 0
    if name == "start" and len(args) >= 3:
        payload = json.dumps({"command": "start", "from": int(args[1]), "to": int(args[2])})
    elif name == "start" and len(args) == 2:
        payload = json.dumps({"command": "start", "from": int(args[1]), "to": int(args[1])})
    else:
        payload = name
    return send(state, payload)


if __name__ == "__main__":
    sys.exit(main())
