"""OpenAI 互換 API の VLM サーバ（vLLM、llama.cpp など）を呼ぶ薄いクライアント.

標準ライブラリだけで動く. 返答の読み取り（yes / no、点、JSON）もここに置く.
"""
from __future__ import annotations

import base64
import json
import math
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


class VlmError(RuntimeError):
    """VLM サーバとの通信、または返答の読み取りに失敗した."""


@dataclass
class VlmReply:
    text: str
    latency_s: float
    logprobs: Optional[List[Dict[str, Any]]] = None
    usage: Dict[str, Any] = field(default_factory=dict)


@dataclass
class VlmConfig:
    base_url: str = "http://127.0.0.1:8000/v1"
    model: str = "vlm"
    api_key: str = "EMPTY"
    timeout_s: float = 20.0
    # サーバの構造化出力（response_format の json_schema）で返答の形を縛る
    structured_output: bool = True
    # Qwen3.5 などは既定で思考モードが有効. 判定と指差しでは切って応答を速くする
    disable_thinking: bool = True
    # 追加でリクエストに入れる項目（top_k など）
    extra_body: Dict[str, Any] = field(default_factory=dict)


def image_part(jpeg: bytes) -> Dict[str, Any]:
    b64 = base64.b64encode(jpeg).decode("ascii")
    return {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}


class VlmClient:
    def __init__(self, cfg: VlmConfig):
        self.cfg = cfg

    def chat(self, messages: List[Dict[str, Any]], max_tokens: int,
             schema_name: Optional[str] = None, schema: Optional[Dict[str, Any]] = None,
             temperature: float = 0.0, logprobs: bool = False,
             thinking: Optional[bool] = None) -> VlmReply:
        body: Dict[str, Any] = {
            "model": self.cfg.model,
            "messages": messages,
            "max_tokens": int(max_tokens),
            "temperature": float(temperature),
        }
        if schema is not None and self.cfg.structured_output:
            body["response_format"] = {
                "type": "json_schema",
                "json_schema": {"name": schema_name or "reply", "schema": schema},
            }
        enable_thinking = (not self.cfg.disable_thinking) if thinking is None else thinking
        body["chat_template_kwargs"] = {"enable_thinking": bool(enable_thinking)}
        if logprobs:
            body["logprobs"] = True
            body["top_logprobs"] = 5
        body.update(self.cfg.extra_body or {})

        url = self.cfg.base_url.rstrip("/") + "/chat/completions"
        req = urllib.request.Request(
            url, data=json.dumps(body).encode("utf-8"), method="POST",
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {self.cfg.api_key}"})
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(req, timeout=self.cfg.timeout_s) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", "replace")[:500]
            raise VlmError(f"VLM サーバが HTTP {e.code} を返しました: {detail}") from e
        except (urllib.error.URLError, TimeoutError, ConnectionError, OSError) as e:
            raise VlmError(f"VLM サーバ（{url}）に接続できません: {e}") from e
        except json.JSONDecodeError as e:
            raise VlmError(f"VLM サーバの返答が JSON ではありません: {e}") from e
        latency = time.monotonic() - t0

        try:
            choice = payload["choices"][0]
            message = choice["message"]
        except (KeyError, IndexError, TypeError) as e:
            raise VlmError(f"VLM サーバの返答に choices がありません: {str(payload)[:300]}") from e
        text = message.get("content") or ""
        if isinstance(text, list):  # 一部のサーバは content を部品の配列で返す
            text = "".join(p.get("text", "") for p in text if isinstance(p, dict))
        lp = None
        if isinstance(choice.get("logprobs"), dict):
            lp = choice["logprobs"].get("content")
        return VlmReply(text=str(text), latency_s=latency, logprobs=lp, usage=payload.get("usage") or {})

    def ping(self) -> Tuple[bool, str]:
        """サーバが応答するか確かめる（/models を読む）."""
        url = self.cfg.base_url.rstrip("/") + "/models"
        req = urllib.request.Request(url, headers={"Authorization": f"Bearer {self.cfg.api_key}"})
        try:
            with urllib.request.urlopen(req, timeout=min(self.cfg.timeout_s, 5.0)) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            names = [m.get("id", "?") for m in data.get("data", [])] if isinstance(data, dict) else []
            return True, ", ".join(names) or "ok"
        except Exception as e:  # noqa: BLE001  状態表示用なので何が起きても文字列で返す
            return False, str(e)


# ---------------------------------------------------------------------------
# 返答の読み取り
# ---------------------------------------------------------------------------
_THINK_RE = re.compile(r"<think>.*?</think>", re.S | re.I)
_FENCE_RE = re.compile(r"```(?:json|JSON)?\s*(.*?)```", re.S)


def strip_reasoning(text: str) -> str:
    """思考の出力（<think>...</think>）を取り除く."""
    text = _THINK_RE.sub("", text)
    if "</think>" in text:  # 開始タグが省かれる場合
        text = text.split("</think>", 1)[1]
    return text.strip()


def extract_json(text: str) -> Any:
    """返答から最初の JSON の値を取り出す. コードブロックや前後の文は無視する. 無ければ None."""
    text = strip_reasoning(text)
    candidates = [m.group(1) for m in _FENCE_RE.finditer(text)] + [text]
    decoder = json.JSONDecoder()
    for cand in candidates:
        cand = cand.strip()
        for i, ch in enumerate(cand):
            if ch in "{[":
                try:
                    value, _ = decoder.raw_decode(cand[i:])
                    return value
                except json.JSONDecodeError:
                    continue
    return None


def parse_yes_no(text: str) -> Optional[bool]:
    """yes / no を読む. {"answer": "yes"} でも、ただの "Yes." でもよい. 読めなければ None."""
    cleaned = strip_reasoning(text)
    value = extract_json(cleaned)
    if isinstance(value, dict):
        for key in ("answer", "result", "value"):
            if key in value:
                ans = value[key]
                if isinstance(ans, bool):
                    return ans
                cleaned = str(ans)
                break
    m = re.search(r"\b(yes|no|true|false)\b", cleaned.lower())
    if not m:
        return None
    return m.group(1) in ("yes", "true")


def yes_probability(logprobs: Optional[List[Dict[str, Any]]]) -> Optional[float]:
    """トークンの対数確率から yes の確率を求める. 取れなければ None.

    yes / no に当たる最初のトークンを探し、その位置の候補の中で yes と no の確率を比べる.
    """
    if not logprobs:
        return None

    def norm(tok: Any) -> str:
        return re.sub(r"[^a-z]", "", str(tok).lower())

    for entry in logprobs:
        if not isinstance(entry, dict) or norm(entry.get("token")) not in ("yes", "no"):
            continue
        p_yes = p_no = 0.0
        tops = entry.get("top_logprobs") or [entry]
        for cand in tops:
            try:
                p = math.exp(float(cand.get("logprob")))
            except (TypeError, ValueError):
                continue
            name = norm(cand.get("token"))
            if name == "yes":
                p_yes += p
            elif name == "no":
                p_no += p
        total = p_yes + p_no
        return p_yes / total if total > 0.0 else None
    return None


_NUM = r"(-?\d+(?:\.\d+)?)"
_POINT_PATTERNS = [
    re.compile(r"point_2d[\"']?\s*[:=]\s*[\[(]\s*" + _NUM + r"\s*,\s*" + _NUM, re.I),
    re.compile(r"<points?\s+x\d*\s*=\s*[\"']?" + _NUM + r"[\"']?\s+y\d*\s*=\s*[\"']?" + _NUM, re.I),
    re.compile(r"<point>\s*[\[(]?\s*" + _NUM + r"\s*,\s*" + _NUM, re.I),
    re.compile(r"[\[(]\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*[\])]"),
]


def parse_point(text: str) -> Optional[Tuple[float, float]]:
    """点の座標（モデルが返した数値のまま、先頭の1点）を読む. 読めなければ None.

    Qwen 系の {"point_2d": [x, y]}、その配列、XML 風の <points x= y=>、ただの [x, y] に対応する.
    """
    cleaned = strip_reasoning(text)
    value = extract_json(cleaned)
    if isinstance(value, list) and value and isinstance(value[0], dict):
        value = value[0]
    if isinstance(value, dict):
        for key in ("point_2d", "point", "points", "coordinate", "coordinates"):
            pt = value.get(key)
            if isinstance(pt, list) and pt and isinstance(pt[0], list):
                pt = pt[0]
            if isinstance(pt, list) and len(pt) >= 2:
                try:
                    return float(pt[0]), float(pt[1])
                except (TypeError, ValueError):
                    return None
        if "x" in value and "y" in value:
            try:
                return float(value["x"]), float(value["y"])
            except (TypeError, ValueError):
                return None
    elif isinstance(value, list) and len(value) >= 2 and all(isinstance(n, (int, float)) for n in value[:2]):
        return float(value[0]), float(value[1])
    for pat in _POINT_PATTERNS:
        m = pat.search(cleaned)
        if m:
            return float(m.group(1)), float(m.group(2))
    return None
