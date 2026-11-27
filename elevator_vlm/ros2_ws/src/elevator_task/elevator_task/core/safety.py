"""安全ゲートの中身. スキルが出した速度指令は、必ずここを通してから台車に渡す.

  * 速度の上限をかける.
  * 速度指令、オドメトリ、カメラのどれかが途絶えたら 0 を出す.
  * 停止の要求を受けたら、解除されるまで 0 を出し続ける.

カメラとオドメトリだけでは、走行中に現れた人や物を検知して止まることはできない.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

STATE_OK = "ok"
STATE_IDLE = "idle"                # 速度指令が来ていない
STATE_STOPPED = "stopped"          # 停止の要求で止めている
STATE_ODOM_LOST = "odom_lost"
STATE_CAMERA_LOST = "camera_lost"


@dataclass
class SafetyConfig:
    max_linear: float = 0.5        # [m/s]
    max_angular: float = 1.2       # [rad/s]
    cmd_timeout_s: float = 0.5
    odom_timeout_s: float = 1.0
    camera_timeout_s: float = 2.0  # 0 でカメラの監視を無効にする
    # 速度指令が来ていない（実行系が動かしていない）間も 0 を出し続けるか.
    # false にすると、0 を idle_tail_s 秒だけ出してから黙る. ROS1システム（ROS 1 のナビゲーションなど）と
    # 同じ /cmd_vel を使い分けるとき（途中から・ここまで）は false にする
    publish_when_idle: bool = True
    idle_tail_s: float = 0.5


class SafetyGate:
    def __init__(self, cfg: SafetyConfig):
        self.cfg = cfg
        self._cmd: Tuple[float, float] = (0.0, 0.0)
        self._cmd_at: Optional[float] = None
        self._odom_at: Optional[float] = None
        self._camera_at: Optional[float] = None
        self._stopped = False
        self.stop_reason = ""

    def on_cmd(self, now: float, linear: float, angular: float) -> None:
        self._cmd = (linear, angular)
        self._cmd_at = now

    def on_odom(self, now: float) -> None:
        self._odom_at = now

    def on_camera(self, now: float) -> None:
        self._camera_at = now

    def stop(self, reason: str = "") -> None:
        self._stopped = True
        self.stop_reason = reason

    def reset(self) -> None:
        self._stopped = False
        self.stop_reason = ""
        self._cmd = (0.0, 0.0)
        self._cmd_at = None

    @staticmethod
    def _stale(last: Optional[float], now: float, timeout: float) -> bool:
        # 時刻が巻き戻った場合（シミュレーションのリセット）も、届いていない扱いにする
        return last is None or now - last > timeout or now < last

    def should_publish(self, now: float) -> bool:
        """今の出力を出すか. publish_when_idle が false なら、指令が途絶えて idle_tail_s 秒たったら黙る."""
        if self.cfg.publish_when_idle:
            return True
        if self._cmd_at is not None and not self._stale(self._cmd_at, now, self.cfg.cmd_timeout_s):
            return True                    # 実行系が動かしている間は、止める指令（0）も含めて出す
        last = self._cmd_at if self._cmd_at is not None and now >= self._cmd_at else None
        if last is None:
            return False                   # 一度も指令が来ていない. 何も出さない
        return now - last <= self.cfg.cmd_timeout_s + self.cfg.idle_tail_s

    def output(self, now: float) -> Tuple[float, float, str]:
        """今出してよい速度指令と状態を返す."""
        cfg = self.cfg
        if self._stopped:
            return 0.0, 0.0, STATE_STOPPED
        if self._stale(self._odom_at, now, cfg.odom_timeout_s):
            return 0.0, 0.0, STATE_ODOM_LOST
        if cfg.camera_timeout_s > 0.0 and self._stale(self._camera_at, now, cfg.camera_timeout_s):
            return 0.0, 0.0, STATE_CAMERA_LOST
        if self._stale(self._cmd_at, now, cfg.cmd_timeout_s):
            return 0.0, 0.0, STATE_IDLE
        linear = max(-cfg.max_linear, min(cfg.max_linear, self._cmd[0]))
        angular = max(-cfg.max_angular, min(cfg.max_angular, self._cmd[1]))
        return linear, angular, STATE_OK
