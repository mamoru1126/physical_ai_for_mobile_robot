#!/bin/bash
# Orin で VLM のサーバ（vLLM、OpenAI 互換の API）を起動する. 起動して答えられるようになるまで待つ.
#   bash jetson/vlm_up.sh                                         既定のモデル（Qwen3.5-9B の 4bit 版）
#   bash jetson/vlm_up.sh RedHatAI/Qwen3.5-4B-quantized.w4a16     モデルを指定する
#
# 設定は 3090 の PC（elevator_vlm/docker-compose.yml）とそろえてある. 実行系からは
# http://127.0.0.1:8000/v1 の「vlm」という名前のモデルに見える.
#
# 環境変数（指定したときだけ使う）:
#   VLLM_IMAGE                   vLLM のイメージ（既定は NVIDIA が AGX Orin の JetPack 7.2 で試した版）
#   VLM_GPU_MEMORY_GIB           VLM に使わせるメモリ [GiB]（既定 20）. メモリの全体に対する割合に直して渡す
#   VLM_GPU_MEMORY_UTILIZATION   割合を直接指定する（例 0.4）. 指定すると VLM_GPU_MEMORY_GIB は使わない
#   VLM_MAX_MODEL_LEN            1回の問いの長さの上限 [トークン]（既定 8192）
#   VLM_MAX_NUM_SEQS             同時に受け付ける問いの数（既定 8）
#   VLM_PORT                     ポート（既定 8000）
#   VLM_EXTRA_ARGS               vllm serve に足す引数（例 "--enable-prefix-caching"）
#   VLM_DOCKER_GPUS              docker run の --gpus（既定 all. 空にすると付けない）
#   VLM_WAIT_S                   起動を待つ時間の上限 [秒]（既定 2400）
#   HF_TOKEN                     Hugging Face のトークン（公開のモデルなら要らない）
#   VLM_OFFLINE=1                ネットにつながずに起動する（ダウンロード済みの重みとイメージだけを使う.
#                                ロボットに載せた後や、載せる前にネットなしで動くかを確かめるとき）
#   VLM_RESTART                  コンテナの再起動の決まり（例 unless-stopped: 電源を入れ直しても自動で起動する.
#                                実機の jetson/robot/robot_up.sh が付ける. 既定は付けない）
set -euo pipefail
cd "$(dirname "$0")/.."

MODEL="${1:-RedHatAI/Qwen3.5-9B-quantized.w4a16}"
IMAGE="${VLLM_IMAGE:-vllm/vllm-openai:v0.22.0-ubuntu2404}"
NAME="elevator-vlm"
PORT="${VLM_PORT:-8000}"
WAIT_S="${VLM_WAIT_S:-2400}"
GPUS="${VLM_DOCKER_GPUS-all}"

say() { printf '%s\n' "$*"; }
fail() { printf 'NG: %s\n' "$*" >&2; exit 1; }

[ -f /etc/nv_tegra_release ] || fail "Jetson ではないようです（/etc/nv_tegra_release がありません）"
case "$(head -n 1 /etc/nv_tegra_release)" in
  *"R39 (release)"*) ;;
  *) fail "JetPack 7.2 ではありません. JetPack 6 の Orin では、このイメージは動きません（jetson/README.md）" ;;
esac
command -v docker > /dev/null 2>&1 || fail "Docker がありません（jetson/README.md の「Docker」）"
if docker info > /dev/null 2>&1; then
  DOCKER=(docker)
else
  DOCKER=(sudo docker)
fi
"${DOCKER[@]}" info --format '{{json .Runtimes}}' 2> /dev/null | grep -q nvidia \
  || fail "Docker に NVIDIA のランタイムがありません（jetson/README.md の「Docker」）"

# 前に起動したものが残っていたら消す. 別のサーバがポートを使っていたら、始めない
if "${DOCKER[@]}" ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
  say "前に起動した VLM のサーバを止めます"
  "${DOCKER[@]}" rm -f "${NAME}" > /dev/null
fi
if (exec 3<> "/dev/tcp/127.0.0.1/${PORT}") 2> /dev/null; then
  fail "ポート ${PORT} を別のプログラムが使っています. 止めてから、もう一度実行してください"
fi

# 電源モード（速さが変わるので、知らせるだけ）
power="$(nvpmodel -q 2> /dev/null || sudo -n nvpmodel -q 2> /dev/null || true)"
power_name="$(printf '%s\n' "${power}" | grep -m 1 'Power Mode' | sed 's/.*Power Mode: *//')"
if [ -n "${power_name}" ] && [[ "${power_name}" != MAXN* ]]; then
  say "電源モード: ${power_name}（MAXN より遅くなります. 電源が足りずに落ちる環境では、このままにします）"
fi

# VLM に使わせるメモリ. Orin は CPU と GPU でメモリを分け合うので、全体に対する割合で渡す
if [ -n "${VLM_GPU_MEMORY_UTILIZATION:-}" ]; then
  util="${VLM_GPU_MEMORY_UTILIZATION}"
else
  mem_kib="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)"
  util="$(awk -v g="${VLM_GPU_MEMORY_GIB:-20}" -v k="${mem_kib}" \
    'BEGIN { u = g * 1048576 / k; if (u < 0.2) u = 0.2; if (u > 0.85) u = 0.85; printf "%.2f", u }')"
fi

if ! "${DOCKER[@]}" image inspect "${IMAGE}" > /dev/null 2>&1; then
  [ "${VLM_OFFLINE:-0}" = "1" ] && fail "vLLM のイメージ ${IMAGE} がありません. ネットにつながる場所で一度起動してください"
  say "vLLM のイメージを取ってきます（初回だけ. 大きいので時間がかかります）: ${IMAGE}"
  "${DOCKER[@]}" pull "${IMAGE}"
fi

mkdir -p "${HOME}/.cache/huggingface"
args=(run -d --name "${NAME}" --runtime=nvidia --network=host --ipc=host
  -v "${HOME}/.cache/huggingface:/root/.cache/huggingface")
[ -n "${VLM_RESTART:-}" ] && args+=(--restart "${VLM_RESTART}")
[ -n "${GPUS}" ] && args+=(--gpus "${GPUS}")
[ -n "${HF_TOKEN:-}" ] && args+=(-e "HF_TOKEN=${HF_TOKEN}")
if [ "${VLM_OFFLINE:-0}" = "1" ]; then
  # Hugging Face に問い合わせず、~/.cache/huggingface にある重みだけを使う
  args+=(-e HF_HUB_OFFLINE=1 -e TRANSFORMERS_OFFLINE=1)
  say "ネットにつながずに起動します（ダウンロード済みの重みだけを使う）"
fi
# プロキシの下では、重みを落とすためにコンテナにもプロキシの設定を渡す
for var in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do
  [ -n "${!var:-}" ] && args+=(-e "${var}=${!var}")
done
args+=(--entrypoint vllm "${IMAGE}" serve "${MODEL}"
  --served-model-name vlm
  --host 0.0.0.0 --port "${PORT}"
  --max-model-len "${VLM_MAX_MODEL_LEN:-8192}"
  --gpu-memory-utilization "${util}"
  --max-num-seqs "${VLM_MAX_NUM_SEQS:-8}"
  --reasoning-parser qwen3)
if [ -n "${VLM_EXTRA_ARGS:-}" ]; then
  read -r -a extra <<< "${VLM_EXTRA_ARGS}"
  args+=("${extra[@]}")
fi

say "VLM のサーバを起動します"
say "  モデル: ${MODEL}"
say "  イメージ: ${IMAGE}"
say "  メモリの割合: ${util}（全体 $(awk '/^MemTotal:/ { printf "%.1f", $2 / 1048576 }' /proc/meminfo) GiB のうち）"
"${DOCKER[@]}" "${args[@]}" > /dev/null

say "答えられるようになるまで待ちます. 初回はモデルの重みを落とすので、長くかかります（止めるときは Ctrl+C. サーバは動き続けます）"
start=${SECONDS}
last=0
until curl -sf --noproxy '*' --max-time 3 "http://127.0.0.1:${PORT}/v1/models" > /dev/null 2>&1; do
  if ! "${DOCKER[@]}" ps --format '{{.Names}}' | grep -qx "${NAME}"; then
    say ""
    say "NG: VLM のサーバが止まりました. ログの最後:"
    "${DOCKER[@]}" logs --tail 60 "${NAME}" 2>&1 | sed 's/^/  /'
    exit 1
  fi
  elapsed=$((SECONDS - start))
  if [ "${elapsed}" -ge "${WAIT_S}" ]; then
    fail "${WAIT_S} 秒たっても答えられるようになりません（ログ: ${DOCKER[*]} logs ${NAME}）"
  fi
  if [ $((elapsed - last)) -ge 30 ]; then
    last=${elapsed}
    line="$("${DOCKER[@]}" logs --tail 1 "${NAME}" 2>&1 | cut -c1-160)"
    say "  ${elapsed} 秒: ${line}"
  fi
  sleep 5
done
say ""
say "OK: VLM のサーバが答えられるようになりました（$((SECONDS - start)) 秒）"
say "  http://127.0.0.1:${PORT}/v1（モデル名 vlm）"
say "次は測ります: bash jetson/vlm_bench.sh"
say "止めるとき: bash jetson/vlm_down.sh"
