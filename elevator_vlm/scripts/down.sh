#!/bin/bash
# すべてのコンテナ（VLM サーバを含む）を止める
set -e
cd "$(dirname "$0")/.."
docker compose down
