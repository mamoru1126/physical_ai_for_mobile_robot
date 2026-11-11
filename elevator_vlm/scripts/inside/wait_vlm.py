#!/usr/bin/env python3
"""VLM サーバが応答するまで待つ（初回はモデルのダウンロードがあるので長い）."""
import json
import os
import sys
import time
import urllib.request

DIRECT = urllib.request.build_opener(urllib.request.ProxyHandler({}))

base = os.environ.get("VLM_BASE_URL", "http://127.0.0.1:8000/v1").rstrip("/")
timeout_s = float(sys.argv[1]) if len(sys.argv) > 1 else 3600.0
t0 = time.monotonic()
print(f"VLM サーバ（{base}）の起動を待っています. 経過は別の端末で  docker compose logs -f vlm  で見られます")
while True:
    try:
        opener = DIRECT if ("127.0.0.1" in base or "localhost" in base) else urllib.request.build_opener()
        with opener.open(base + "/models", timeout=5) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        names = [m.get("id") for m in data.get("data", [])]
        print(f"\n起動しました（{time.monotonic() - t0:.0f} 秒）. モデル: {names}")
        sys.exit(0)
    except Exception as e:  # noqa: BLE001
        if time.monotonic() - t0 > timeout_s:
            print(f"\n{timeout_s:.0f} 秒待っても応答がありません: {e}")
            sys.exit(1)
        print(".", end="", flush=True)
        time.sleep(5.0)
