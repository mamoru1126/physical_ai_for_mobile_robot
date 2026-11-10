"""設定ファイル（YAML）の読み込み. 知らない項目名は誤記とみなしてエラーにする."""
from __future__ import annotations

import dataclasses
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .executor import ExecutorConfig
from .perception import PerceptionConfig
from .plan import PlanLimits
from .safety import SafetyConfig
from .skills import SkillConfig
from .vlm_client import VlmConfig

DEFAULT_INSTRUCTIONS = [
    "扉が開いたら中に入って",
    "中に入ったら180度旋回して",
    "再び扉が開いたら出て",
]


class ConfigError(ValueError):
    pass


@dataclass
class CameraMount:
    height_m: float = 0.50         # 床からの高さ
    pitch_deg: float = 15.0        # 俯角（下向きが正）
    x_offset_m: float = 0.20       # ロボット中心からの前後の距離
    y_offset_m: float = 0.0
    hfov_deg: float = 90.0         # カメラ情報が届かないときだけ使う水平画角


@dataclass
class WebConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    stream_hz: float = 5.0
    # 画面に追加するボタン. 押すと std_srvs/Trigger のサービスを呼ぶ.
    # 例: [{name: call, label: エレベーターを呼ぶ, service: /elevator/call}]
    buttons: List[Dict[str, str]] = field(default_factory=list)
    # 画面に追加で映すカメラ（外から様子を見る固定カメラなど）. 実行系は使わない.
    # 例: [{name: hall, label: 乗場, topic: /observer/floor_1/image_raw}]
    extra_cameras: List[Dict[str, str]] = field(default_factory=list)


@dataclass
class TaskConfig:
    vlm: VlmConfig = field(default_factory=VlmConfig)
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)
    skills: SkillConfig = field(default_factory=SkillConfig)
    executor: ExecutorConfig = field(default_factory=ExecutorConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    camera: CameraMount = field(default_factory=CameraMount)
    web: WebConfig = field(default_factory=WebConfig)
    limits: PlanLimits = field(default_factory=PlanLimits)
    instructions: List[str] = field(default_factory=lambda: list(DEFAULT_INSTRUCTIONS))
    planner_thinking: bool = False
    log_dir: str = "runs"


def _coerce(value: Any, current: Any, where: str) -> Any:
    if isinstance(current, bool):
        if isinstance(value, bool):
            return value
        raise ConfigError(f"{where}: true か false で指定してください（{value!r}）")
    if isinstance(current, int):
        if isinstance(value, int) and not isinstance(value, bool):
            return value
        raise ConfigError(f"{where}: 整数で指定してください（{value!r}）")
    if isinstance(current, float):
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return float(value)
        raise ConfigError(f"{where}: 数値で指定してください（{value!r}）")
    if isinstance(current, str):
        if isinstance(value, str):
            return value
        raise ConfigError(f"{where}: 文字列で指定してください（{value!r}）")
    if isinstance(current, list):
        if isinstance(value, list):
            return value
        raise ConfigError(f"{where}: 配列で指定してください（{value!r}）")
    if isinstance(current, dict):
        if isinstance(value, dict):
            return value
        raise ConfigError(f"{where}: 連想配列で指定してください（{value!r}）")
    return value


def _apply(target: Any, data: Dict[str, Any], where: str) -> Any:
    """dataclass の値を dict で上書きした新しい値を返す（frozen な dataclass にも使える）."""
    names = {f.name for f in dataclasses.fields(target)}
    changes: Dict[str, Any] = {}
    for key, value in data.items():
        path = f"{where}.{key}" if where else str(key)
        if key not in names:
            raise ConfigError(f"{path}: 知らない項目です（使える項目: {sorted(names)}）")
        current = getattr(target, key)
        if dataclasses.is_dataclass(current):
            if not isinstance(value, dict):
                raise ConfigError(f"{path}: 項目の並びで指定してください")
            changes[key] = _apply(current, value, path)
        else:
            changes[key] = _coerce(value, current, path)
    return dataclasses.replace(target, **changes)


def load_config(path: Optional[str] = None, overrides: Optional[Dict[str, Any]] = None,
                env: Optional[Dict[str, str]] = None) -> TaskConfig:
    """設定を読む. 優先順は、環境変数 > overrides > YAML ファイル > 既定値."""
    cfg = TaskConfig()
    if path:
        import yaml  # 遅延 import

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except OSError as e:
            raise ConfigError(f"設定ファイルを開けません: {e}") from e
        except yaml.YAMLError as e:
            raise ConfigError(f"設定ファイルが YAML として読めません: {e}") from e
        if not isinstance(data, dict):
            raise ConfigError("設定ファイルの最上位は項目の並びにしてください")
        cfg = _apply(cfg, data, "")
    if overrides:
        cfg = _apply(cfg, overrides, "")
    env = os.environ if env is None else env
    vlm_changes: Dict[str, Any] = {}
    if env.get("VLM_BASE_URL"):
        vlm_changes["base_url"] = env["VLM_BASE_URL"]
    if env.get("VLM_MODEL"):
        vlm_changes["model"] = env["VLM_MODEL"]
    if env.get("VLM_API_KEY"):
        vlm_changes["api_key"] = env["VLM_API_KEY"]
    if vlm_changes:
        cfg = dataclasses.replace(cfg, vlm=dataclasses.replace(cfg.vlm, **vlm_changes))
    return cfg
