"""実行の記録. 画面表示用に直近の分をメモリに持ち、全部を JSONL と画像でディスクに残す.

VLM に渡した画像、問い、答え、所要時間をすべて保存する（机上評価と、後の学習データに使う）.
"""
from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional


class EventLog:
    def __init__(self, run_dir: Optional[str] = None, keep: int = 400):
        self._lock = threading.Lock()
        self._events: Deque[Dict[str, Any]] = deque(maxlen=keep)
        self._next_id = 1
        self._image_count = 0
        self.run_dir = run_dir
        self._file = None
        # 判定と指差しの記録に添える値を返す関数（シミュレーションの真値など. 分析用）
        self.context_fn: Optional[Callable[[], Optional[Dict[str, Any]]]] = None
        if run_dir:
            try:
                os.makedirs(os.path.join(run_dir, "images"), exist_ok=True)
                self._file = open(os.path.join(run_dir, "events.jsonl"), "a", encoding="utf-8")
            except OSError:
                # 記録先が書けなくても実行は続ける
                self.run_dir = None
                self._file = None

    def context(self) -> Optional[Dict[str, Any]]:
        """今の時点の付加情報（無ければ None）. 画像を撮った直後に呼び、その画像の記録に添える."""
        if self.context_fn is None:
            return None
        try:
            return self.context_fn() or None
        except Exception:  # noqa: BLE001  記録の付け足しで実行を止めない
            return None

    def emit(self, kind: str, **data: Any) -> Dict[str, Any]:
        with self._lock:
            event = {"id": self._next_id, "wall": round(time.time(), 3), "kind": kind}
            event.update({k: v for k, v in data.items() if not (k == "ctx" and v is None)})
            self._next_id += 1
            self._events.append(event)
            if self._file:
                try:
                    self._file.write(json.dumps(event, ensure_ascii=False) + "\n")
                    self._file.flush()
                except (OSError, TypeError, ValueError):
                    pass
            return event

    def save_image(self, label: str, jpeg: bytes) -> str:
        """画像を保存し、run_dir からの相対パスを返す. 保存先が無ければ空文字."""
        if not self.run_dir:
            return ""
        with self._lock:
            self._image_count += 1
            name = f"{self._image_count:05d}_{label}.jpg"
        rel = os.path.join("images", name)
        try:
            with open(os.path.join(self.run_dir, rel), "wb") as f:
                f.write(jpeg)
        except OSError:
            return ""
        return rel

    def since(self, last_id: int = 0, limit: int = 200) -> List[Dict[str, Any]]:
        with self._lock:
            out = [e for e in self._events if e["id"] > last_id]
        return out[-limit:]

    def close(self) -> None:
        with self._lock:
            if self._file:
                try:
                    self._file.close()
                finally:
                    self._file = None
