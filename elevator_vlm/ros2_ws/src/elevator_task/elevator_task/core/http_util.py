"""HTTP の呼び出し. 自分の計算機（localhost、127.0.0.1 など）宛ては、プロキシを通さない.

社内ネットワークなどで http_proxy が設定されていると、urllib は 127.0.0.1 宛ての要求まで
プロキシに送り、プロキシが 403 などで断る. no_proxy の設定に頼らず、ここで必ず直接つなぐ.
それ以外の宛先は、環境変数のプロキシ設定に従う.
"""
from __future__ import annotations

import ipaddress
import urllib.request
from typing import Union
from urllib.parse import urlparse

_DIRECT = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def is_local(url: str) -> bool:
    host = (urlparse(url).hostname or "").strip("[]").lower()
    if host in ("localhost", "") or host.endswith(".localhost"):
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def urlopen(req: Union[str, urllib.request.Request], timeout: float):
    url = req if isinstance(req, str) else req.full_url
    if is_local(url):
        return _DIRECT.open(req, timeout=timeout)
    return urllib.request.urlopen(req, timeout=timeout)
