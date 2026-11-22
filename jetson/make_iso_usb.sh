#!/bin/bash
# JetPack 7.2.1 を入れ直すための USB メモリ（Jetson ISO）を作る.
# Orin の上でも、ふつうの Linux の PC でも動く（書き込みには sudo を使う）.
#
#   bash jetson/make_iso_usb.sh             ISO を落とし、つないである USB メモリを一覧にする（書き込まない）
#   bash jetson/make_iso_usb.sh /dev/sda    ISO を落とし、/dev/sda に書き込む（書き込む前に確認する）
#
# 環境変数（指定したときだけ使う）:
#   ISO_URL      落とす ISO（既定は JetPack 7.2.1 / Jetson Linux r39.2.1 の ISO）
#   ISO_PATH     ISO の置き場所（既定は ~/jetson-iso-r39.2.1.iso）
#   ISO_SHA256   ISO の SHA-256. 指定すると、書き込む前に照合する
#
# 守っていること:
#   - USB でつながったディスク全体にしか書き込まない（eMMC、NVMe、SD カード、パーティションは断る）
#   - 今動いているシステム（/）が載っているディスクには書き込まない
#   - 16 GB より小さいものには書き込まない
#   - 書き込む前に、ディスクの名前を打ち込んでもらって確かめる
set -euo pipefail

ISO_URL="${ISO_URL:-https://developer.nvidia.com/downloads/embedded/l4t/r39_release_v2.1/iso/jetsoninstaller-r39.2.1-2026-08-07-18-30-47-arm64.iso}"
ISO_PATH="${ISO_PATH:-${HOME}/jetson-iso-r39.2.1.iso}"
ISO_SHA256="${ISO_SHA256:-}"
MIN_ISO_BYTES=$((1024 * 1024 * 1024))         # これより小さければ ISO ではない（ログイン画面の HTML など）
MIN_USB_BYTES=$((15 * 1000 * 1000 * 1000))    # 16 GB の USB メモリ（表示は 16 GB でも、実際は少し小さい）

say() { printf '%s\n' "$*"; }
fail() { printf 'NG: %s\n' "$*" >&2; exit 1; }
gb() { awk -v b="$1" 'BEGIN { printf "%.1f GB", b / 1e9 }'; }

# lsblk の 1 行（KEY="値" の並び）から、指定した項目の値を取り出す
field() {
  local line="$1" key="$2"
  if [[ "${line}" =~ (^|[[:space:]])${key}=\"([^\"]*)\" ]]; then
    printf '%s' "${BASH_REMATCH[2]}"
  fi
}

# 今動いているシステム（/）が載っているディスクの名前（例: mmcblk0、nvme0n1）
root_disk() {
  local src
  src="$(findmnt -n -o SOURCE / 2> /dev/null || true)"
  [ -n "${src}" ] || return 0
  lsblk -n -o PKNAME "${src}" 2> /dev/null | head -n 1 | tr -d '[:space:]' || true
}

# ---- 1. 今の版 ---------------------------------------------------------------
if [ -f /etc/nv_tegra_release ]; then
  say "今の版: $(head -n 1 /etc/nv_tegra_release)"
fi

# ---- 2. ISO を落とす ----------------------------------------------------------
# 落とし終わるまでは .part という名前にしておく. 途中で切れても、もう一度実行すれば続きから落とす
if [ ! -f "${ISO_PATH}" ]; then
  part="${ISO_PATH}.part"
  if [ -f "${part}" ] && head -c 512 "${part}" | grep -qi '<html\|<!doctype'; then
    rm -f "${part}"                                   # 前回、HTML を落としていた
  fi
  say "ISO を落とします（数 GB あります）: ${ISO_URL}"
  wget -c -O "${part}" "${ISO_URL}"
  mv "${part}" "${ISO_PATH}"
fi

size="$(stat -c %s "${ISO_PATH}")"
if [ "${size}" -lt "${MIN_ISO_BYTES}" ]; then
  mv "${ISO_PATH}" "${ISO_PATH}.html"
  fail "落としたファイルが小さすぎます（$(gb "${size}")）. ISO ではなく、ログイン画面などの HTML のようです.
    ${ISO_PATH}.html に移しました. ブラウザで JetPack の配布ページから ISO を落とし、
    ${ISO_PATH} に置いてから、もう一度実行してください."
fi
# ISO 9660 の印（先頭から 32769 バイト目に CD001）があるか
signature="$({ dd if="${ISO_PATH}" bs=1 skip=32769 count=5 status=none 2> /dev/null || true; } | tr -d '\000')"
if [ "${signature}" != "CD001" ]; then
  fail "${ISO_PATH} は ISO の形式ではないようです. 落とし直すときは、このファイルを消してから実行してください."
fi
if [ -n "${ISO_SHA256}" ]; then
  say "SHA-256 を照合しています..."
  actual="$(sha256sum "${ISO_PATH}" | awk '{print $1}')"
  if [ "${actual}" != "${ISO_SHA256,,}" ]; then
    fail "SHA-256 が合いません（${actual}）. ファイルを消して、落とし直してください."
  fi
  say "SHA-256 は一致しました"
fi
say "ISO: ${ISO_PATH}（$(gb "${size}")）"

# ---- 3. 書き込み先を決める ------------------------------------------------------
ROOT_DISK="$(root_disk)"

if [ "$#" -eq 0 ]; then
  say ""
  say "つないである USB のディスク:"
  found=0
  while IFS= read -r line; do
    [ -n "${line}" ] || continue
    name="$(field "${line}" NAME)"
    tran="$(field "${line}" TRAN)"
    type="$(field "${line}" TYPE)"
    bytes="$(field "${line}" SIZE)"
    [ "${tran}" = "usb" ] && [ "${type}" = "disk" ] || continue
    found=1
    note=""
    if [ "${name}" = "${ROOT_DISK}" ]; then
      note="  ← 今動いているシステムが載っているので使えません"
    elif [ "${bytes:-0}" -lt "${MIN_USB_BYTES}" ]; then
      note="  ← 16 GB より小さいので使えません"
    fi
    say "  /dev/${name}  $(gb "${bytes:-0}")  $(lsblk -d -n -o MODEL "/dev/${name}" 2> /dev/null | xargs)${note}"
  done < <(lsblk -d -n -b -P -o NAME,SIZE,TRAN,TYPE)
  if [ "${found}" -eq 0 ]; then
    say "  （見つかりません. USB メモリを挿してから、もう一度実行してください）"
    exit 0
  fi
  say ""
  say "書き込むときは、名前を付けて実行します（中身はすべて消えます）. 例:"
  say "  bash $0 /dev/sda"
  exit 0
fi

if [ "$#" -ne 1 ]; then
  fail "書き込み先を1つだけ指定してください（例: /dev/sda）"
fi
dev="$1"
case "${dev}" in
  /dev/*) ;;
  *) dev="/dev/${dev}" ;;
esac
name="${dev#/dev/}"
case "${name}" in
  mmcblk* | nvme* | loop* | ram* | zram*)
    fail "${dev} は USB メモリではありません（eMMC、NVMe、SD カードなどには書き込みません）" ;;
esac

line="$(lsblk -d -n -b -P -o NAME,SIZE,TRAN,TYPE "${dev}" 2> /dev/null | head -n 1 || true)"
[ -n "${line}" ] || fail "${dev} が見つかりません. 何も指定せずに実行すると、USB のディスクの一覧が出ます"
tran="$(field "${line}" TRAN)"
type="$(field "${line}" TYPE)"
bytes="$(field "${line}" SIZE)"
[ "${type}" = "disk" ] || fail "${dev} はディスク全体ではありません（パーティションではなく、/dev/sda のように指定します）"
[ "${tran}" = "usb" ] || fail "${dev} は USB でつながっていません（つながり方: ${tran:-不明}）"
[ "${name}" != "${ROOT_DISK}" ] || fail "${dev} には、今動いているシステムが載っています"
[ "${bytes:-0}" -ge "${MIN_USB_BYTES}" ] || fail "${dev} は $(gb "${bytes:-0}") で、16 GB より小さいです"
[ "${bytes}" -gt "${size}" ] || fail "${dev}（$(gb "${bytes}")）は ISO（$(gb "${size}")）より小さいです"

# マウントされているパーティション. システムが使っている場所にマウントされていたら断る
mounted=()
while IFS= read -r part_line; do
  [ -n "${part_line}" ] || continue
  part_name="$(field "${part_line}" NAME)"
  mountpoint="$(field "${part_line}" MOUNTPOINT)"
  [ -n "${mountpoint}" ] || continue
  case "${mountpoint}" in
    / | /boot* | /home* | /usr* | /var* | /opt* | /etc* | \[SWAP\])
      fail "${part_name} が ${mountpoint} にマウントされています. システムが使っているディスクには書き込みません" ;;
  esac
  mounted+=("${part_name}")
done < <(lsblk -l -n -p -P -o NAME,MOUNTPOINT "${dev}")

model="$(lsblk -d -n -o MODEL "${dev}" 2> /dev/null | xargs || true)"
say ""
say "書き込み先: ${dev}  $(gb "${bytes}")  ${model}"
say "このディスクの中身は、すべて消えます."
printf '確かめのため、ディスクの名前（%s）を打ち込んで Enter を押してください: ' "${name}"
read -r answer || answer=""
if [ "${answer}" != "${name}" ]; then
  fail "名前が一致しないので、やめました（何も書き込んでいません）"
fi

# ---- 4. 書き込む ---------------------------------------------------------------
for part_name in "${mounted[@]+"${mounted[@]}"}"; do
  say "マウントを外します: ${part_name}"
  sudo umount "${part_name}"
done
say "書き込みます（数分かかります）..."
sudo dd if="${ISO_PATH}" of="${dev}" bs=4M status=progress conv=fsync
sync
say ""
say "OK: USB メモリができました（${dev}）."
say "次は、USB メモリを挿したまま Orin を再起動し、UEFI のブートメニューで USB を選びます."
say "続きは jetson/README.md の「2. 入れる」を見てください."
