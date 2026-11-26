#!/bin/bash
# Orin の状態を確かめる（何も変えない）. JetPack 7.2 を入れたあとと、VLM を動かす前に使う.
#   bash jetson/check.sh
# 結果は OK（そのままでよい）、注意（動くが、直した方がよい）、NG（直さないと先に進めない）で出す.
# NG があれば終了コードが 1 になる.
set -uo pipefail
cd "$(dirname "$0")/.."

IMAGE="${VLLM_IMAGE:-vllm/vllm-openai:v0.22.0-ubuntu2404}"
PORT="${VLM_PORT:-8000}"
ng=0
warn=0
ok() { printf 'OK    %s\n' "$*"; }
note() { printf '注意  %s\n' "$*"; warn=$((warn + 1)); }
bad() { printf 'NG    %s\n' "$*"; ng=$((ng + 1)); }
info() { printf '      %s\n' "$*"; }

# ---- JetPack ------------------------------------------------------------------
if [ -f /etc/nv_tegra_release ]; then
  release="$(head -n 1 /etc/nv_tegra_release | cut -d, -f1-2 | sed 's/^# *//')"
  case "${release}" in
    "R39 (release)"*) ok "Jetson Linux: ${release}（JetPack 7.2 の系列）" ;;
    *) bad "JetPack 7.2 ではありません: ${release}（jetson/README.md の手順で入れ直す）" ;;
  esac
else
  bad "/etc/nv_tegra_release がありません（Jetson ではないようです）"
fi

jetpack="$(dpkg-query -W -f='${Version}' nvidia-jetpack 2> /dev/null || true)"
if [ -n "${jetpack}" ]; then
  ok "nvidia-jetpack ${jetpack}"
else
  bad "nvidia-jetpack が入っていません（sudo apt update のあと、sudo apt install nvidia-jetpack）"
fi
cuda="$(/usr/local/cuda/bin/nvcc --version 2> /dev/null | grep -o 'release [0-9.]*' | head -n 1 || true)"
[ -n "${cuda}" ] && info "CUDA ${cuda#release }"

# ---- メモリと空き容量 -------------------------------------------------------------
mem_kib="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo 2> /dev/null || echo 0)"
mem_gib="$(awk -v k="${mem_kib:-0}" 'BEGIN { printf "%.1f", k / 1048576 }')"
ok "メモリ ${mem_gib} GiB（AGX Orin の 64GB 版なら約 61、32GB 版なら約 30）"
free_gb="$(df -BG --output=avail "${HOME}" 2> /dev/null | tail -n 1 | tr -dc '0-9')"
if [ -z "${free_gb}" ]; then
  note "空き容量を確かめられません"
elif [ "${free_gb}" -ge 50 ]; then
  ok "空き容量 ${free_gb} GB（${HOME}）"
else
  note "空き容量が ${free_gb} GB です. vLLM のイメージとモデルの重みで数十 GB 使うので、50 GB 以上あると安心です"
fi

# ---- 電源モード -----------------------------------------------------------------
power="$(nvpmodel -q 2> /dev/null || sudo -n nvpmodel -q 2> /dev/null || true)"
power_name="$(printf '%s\n' "${power}" | grep -m 1 'Power Mode' | sed 's/.*Power Mode: *//')"
if [ -z "${power_name}" ]; then
  note "電源モードを確かめられません（sudo nvpmodel -q で確かめる）"
elif [[ "${power_name}" == MAXN* ]]; then
  ok "電源モード ${power_name}"
else
  note "電源モードが ${power_name} です. 速さを最大で測るなら MAXN にします（jetson/vlm.md の「2. 電源モード」）. 電源が足りずに落ちる環境では MAXN にしません（jetson/bench/README.md）"
fi

# ---- Docker -------------------------------------------------------------------
if command -v docker > /dev/null 2>&1; then
  ok "$(docker --version 2> /dev/null)"
  if docker info > /dev/null 2>&1; then
    ok "sudo なしで docker を使えます"
    DOCKER=(docker)
  else
    note "sudo なしでは docker を使えません（sudo usermod -aG docker \$USER のあと、ログインし直す）"
    DOCKER=(sudo -n docker)
  fi
  runtimes="$("${DOCKER[@]}" info --format '{{json .Runtimes}}' 2> /dev/null || true)"
  default_runtime="$("${DOCKER[@]}" info --format '{{.DefaultRuntime}}' 2> /dev/null || true)"
  if [ -z "${runtimes}" ]; then
    note "Docker のランタイムを確かめられません（sudo docker info で確かめる）"
  elif [[ "${runtimes}" == *nvidia* ]]; then
    ok "Docker に NVIDIA のランタイムがあります（既定: ${default_runtime:-不明}）"
  else
    bad "Docker に NVIDIA のランタイムがありません（jetson/README.md の「Docker」）"
  fi
  if "${DOCKER[@]}" image inspect "${IMAGE}" > /dev/null 2>&1; then
    ok "vLLM のイメージを取得済み: ${IMAGE}"
  else
    info "vLLM のイメージはまだありません（jetson/vlm_up.sh が最初に取ります）: ${IMAGE}"
  fi
else
  bad "Docker がありません（jetson/README.md の「Docker」）"
fi

# ---- ネット ---------------------------------------------------------------------
if curl -s -o /dev/null --max-time 8 https://huggingface.co; then
  ok "インターネットにつながっています（モデルの重みを落とせます）"
else
  note "huggingface.co につながりません（モデルの重みを初めて落とすときに要ります）"
fi

# ---- 測るための Python の部品 --------------------------------------------------------
if python3 -c 'import PIL, numpy, yaml' 2> /dev/null; then
  ok "Python の部品（PIL、numpy、yaml）"
else
  bad "測るための Python の部品が足りません（sudo apt install -y python3-pil python3-numpy python3-yaml）"
fi

# ---- VLM サーバ -----------------------------------------------------------------
models="$(curl -sf --noproxy '*' --max-time 3 "http://127.0.0.1:${PORT}/v1/models" 2> /dev/null || true)"
if [ -n "${models}" ]; then
  root="$(printf '%s' "${models}" | python3 -c 'import json, sys; d = json.load(sys.stdin)["data"][0]; print(d.get("root") or d["id"])' 2> /dev/null || true)"
  ok "VLM サーバが動いています（ポート ${PORT}）: ${root:-不明}"
else
  info "VLM サーバは動いていません（bash jetson/vlm_up.sh で起動）"
fi

echo
if [ "${ng}" -gt 0 ]; then
  echo "NG が ${ng} 件あります. 直してから、もう一度実行してください."
  exit 1
fi
if [ "${warn}" -gt 0 ]; then
  echo "NG はありません（注意 ${warn} 件）."
else
  echo "すべて OK です."
fi
