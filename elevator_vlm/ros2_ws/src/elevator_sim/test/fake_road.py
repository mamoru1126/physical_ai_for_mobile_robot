"""道のある擬似世界を、実行系の RobotIO として見せる（テスト用）. 時刻は擬似時刻.

VLM への問いは、道に沿って走るスキルでは別のスレッドから来る. RoadSimVlm は、どのスレッドから
呼ばれても、擬似時刻で latency_s だけたってから答えを返す（推論の遅れを擬似時刻で再現する）.
擬似時刻を進める制御のスレッドは、答えの出る時刻を、その答え（HTTP の往復）ができる前に追い越さない.
また擬似時刻は、実時間の speed 倍より速くは進めない（問いかけのスレッドが画像を選ぶ間に、
擬似時刻が何十秒も先へ進んでしまわないように）.
"""
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional

from elevator_task.core.geometry import CameraModel, Pose2D, PoseHistory
from elevator_task.core.robot_io import Frame, RobotIO
from elevator_task.core.vlm_client import VlmClient, VlmConfig, VlmReply
from fake_world import true_camera

from elevator_sim.road_world import RoadParams, RoadWorld
from elevator_sim.world_params import RobotParams


class FakeRoad(RoadWorld, RobotIO):
    DT = 0.01

    def __init__(self, road: Optional[RoadParams] = None, camera_rate: float = 10.0, bounded: bool = True,
                 speed: float = 20.0):
        super().__init__(road, bounded=bounded)
        self.speed = speed              # 擬似時刻の進む速さの上限（実時間の何倍か）
        self.rp = RobotParams()
        self.camera = true_camera(self.rp)
        self.camera_rate = camera_rate
        self.history = PoseHistory(horizon_s=60.0)
        self._frame = bytes(self.rp.image_width * self.rp.image_height * 3)
        self.history.add(Pose2D(0.0, 0.0, 0.0, 0.0))
        self.frames_on = True           # False にするとカメラが止まる（新しい画像が来ない）
        self._calls_lock = threading.Lock()
        self._calls: List["_Call"] = []

    # -- 別のスレッドからの VLM の問いと、擬似時刻の進め方 ------------------------
    def call_begin(self, deadline: float) -> "_Call":
        call = _Call(deadline)
        with self._calls_lock:
            self._calls.append(call)
        return call

    def call_end(self, call: "_Call") -> None:
        call.ended.set()
        with self._calls_lock:
            if call in self._calls:
                self._calls.remove(call)

    def step(self, dt: Optional[float] = None) -> None:
        dt = self.DT if dt is None else dt
        with self._calls_lock:
            due = [c for c in self._calls if self.t + dt >= c.deadline]
        for call in due:
            call.http_done.wait(5.0)      # 答えができる前に、答えの出る時刻を追い越さない
        super().step(dt)
        self.history.add(Pose2D(self.t, self.ox, self.oy, self.oyaw))
        for call in due:
            if call.ended.wait(5.0):      # 答えを受け取った側（別のスレッド）が使えるようにするまで待つ
                time.sleep(0.001)

    # -- RobotIO ------------------------------------------------------------
    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        began = time.monotonic()
        for _ in range(max(1, int(round(seconds / self.DT)))):
            self.step()
        if self.speed > 0.0:
            rest = seconds / self.speed - (time.monotonic() - began)
            if rest > 0.0:
                time.sleep(rest)

    def get_frame(self) -> Optional[Frame]:
        if not self.frames_on:
            return Frame(0.0, self.rp.image_width, self.rp.image_height, self._frame)
        stamp = math.floor(self.t * self.camera_rate) / self.camera_rate
        return Frame(stamp, self.rp.image_width, self.rp.image_height, self._frame)

    def get_pose(self) -> Optional[Pose2D]:
        return self.history.latest()

    def pose_at(self, stamp: float) -> Optional[Pose2D]:
        return self.history.at(stamp)

    def get_camera(self) -> Optional[CameraModel]:
        return self.camera

    def send_cmd(self, linear: float, angular: float) -> None:
        self.set_cmd(linear, angular)

    # -- モック VLM の答え ---------------------------------------------------
    def path_answer(self):
        """道の中心線の少し先の点（0〜1000 の相対座標）."""
        uv = self.path_pixel(self.camera)
        if uv is None:
            return None
        return uv[0] / self.camera.width * 1000.0, uv[1] / self.camera.height * 1000.0


class RoadSimVlm(VlmClient):
    """答えを、擬似時刻で latency_s だけ遅らせて返す（呼んだスレッドによらない）."""

    def __init__(self, cfg: VlmConfig, world: FakeRoad, latency_s: float):
        super().__init__(cfg)
        self.world = world
        self.latency_s = latency_s
        self._main = threading.current_thread()

    def chat(self, *args, **kwargs) -> VlmReply:
        if threading.current_thread() is self._main or self.latency_s <= 0.0:
            reply = super().chat(*args, **kwargs)
            if self.latency_s > 0.0:
                self.world.sleep(self.latency_s)
            reply.latency_s = self.latency_s
            return reply
        call = self.world.call_begin(self.world.t + self.latency_s)
        try:
            try:
                reply = super().chat(*args, **kwargs)
            finally:
                call.http_done.set()
            # 擬似時刻を進めるのは制御のスレッド. 答えの出る時刻になるのを待つ（止まっていたら 10 秒であきらめる）
            give_up = time.monotonic() + 10.0
            while self.world.t < call.deadline and time.monotonic() < give_up:
                time.sleep(0.0002)
            reply.latency_s = self.latency_s
            return reply
        finally:
            self.world.call_end(call)


class _Call:
    def __init__(self, deadline: float):
        self.deadline = deadline
        self.http_done = threading.Event()
        self.ended = threading.Event()
