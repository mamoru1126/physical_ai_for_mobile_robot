# Jetson AGX Orin を JetPack 7.2 にする

Orin で VLM を動かすソフトの新しい版は、JetPack 7.2 向けに出ています。たとえば、3090 の PC と同じ上流の vLLM（`vllm/vllm-openai`）や、TensorRT Edge-LLM の新しい版です。JetPack 7.2 から、Orin も JetPack 7 の系列に入りました。ここでは、JetPack 6.2 の Orin を JetPack 7.2.1（Jetson Linux r39.2.1）に入れ直す手順をまとめます。

| 項目 | 状態 |
| --- | --- |
| 手順 | NVIDIA の手引き（2026年11月時点）をもとにまとめた。この Orin ではまだ通していない |
| USB メモリを作るスクリプト | 偽物のディスクで動きを確かめた。本物の USB メモリではまだ |

入れ直すと、JetPack 6 向けに作った GPU を使うコンテナ（omnivla_real の推論サーバなど）は作り直しになります（「4. 作り直しが要るもの」）。

入れたあとは、[Orin で VLM を動かして、速さを測る](vlm.md) に進みます。ロボットに載せる前にエレベーターのタスクや「道に沿って走って」を通すときは [Orin のベンチ](bench/)、ロボットに載せた後は [実機で動かす](robot/)（手順書。ロボットの PC との疎通確認は [robot/link_check.md](robot/link_check.md)）を使います。USB カメラを使うときは [USB カメラの映像を ROS 2 のトピックに流す](camera/)、ROS 1 のシステムとつなぐときは [ROS 1 のシステムとの橋渡し](../ros1_bridge/)（ロボットの PC で動かす） も参照してください。

## 入れる前に

### 確かめること

| 項目 | 条件 | 確かめ方 |
| --- | --- | --- |
| 機種 | NVIDIA の Jetson AGX Orin 開発キット。ほかの会社のキャリアボードに載せた Orin なら、その会社の JetPack 7.2 対応版が要る | 本体の型番 |
| 今の版 | L4T r35.5 以上（r36.4.7 なら満たす） | `cat /etc/nv_tegra_release` |
| ネット | 入れている間、Orin がインターネットにつながっていること | |
| USB メモリ | 16 GB 以上。中身は消える | |
| 空き容量 | USB メモリを作る機械に 25 GB 以上 | `df -h ~` |
| 画面 | DisplayPort のモニタと USB キーボード、またはシリアル端末（「2. 入れる」） | |

### バックアップ

書き込み先（eMMC か NVMe）の中身は消える前提で、先に別の場所へ移します。

- この Orin で使っているリポジトリと、その中の重み、データ、記録、`.env`（omnivla_real の重み・topomap・`runs/`・`debug_log/`、nakanoshima_2026 など）
- ネットワークの設定（固定 IP、経路）、ssh の鍵（`~/.ssh`）、時計合わせの設定
- 残しておきたい Docker のイメージ（ただし、GPU を使うものは JetPack 7.2 ではそのまま動かない見込み）

## 1. USB メモリを作る（Orin の上で）

Orin がネットにつながっていれば、Orin の上で ISO を落として USB メモリに書けます。ふつうの Linux の PC でも同じ手順です。

このリポジトリを取ってきます（取得済みなら不要）。

```bash
git clone https://github.com/mamoru1126/physical_ai_for_mobile_robot.git
```

```bash
cd physical_ai_for_mobile_robot
```

USB メモリを挿してから、スクリプトを実行します。ISO（数 GB）を `~/jetson-iso-r39.2.1.iso` に落とし、つないである USB のディスクを一覧にします。この段階では何も書き込みません。

```bash
bash jetson/make_iso_usb.sh
```

一覧に出た名前を付けて、もう一度実行します。確かめのため、ディスクの名前を打ち込むよう求められます。マウントを外してから書き込みます。

```bash
bash jetson/make_iso_usb.sh /dev/sda
```

スクリプトは、次のものには書き込みません。

- USB でつながっていないディスク（eMMC、NVMe、SD カード）
- パーティション（`/dev/sda1` のような名前）
- 今動いているシステムが載っているディスク
- 16 GB より小さいもの

ISO が落とせないとき（ログイン画面の HTML が落ちてくるなど）は、ブラウザで [JetPack の配布ページ](https://developer.nvidia.com/embedded/jetpack/downloads) から ISO を落とし、`~/jetson-iso-r39.2.1.iso` に置いてから、もう一度実行します。

<details>
<summary>スクリプトを使わずに、コマンドで書く場合</summary>

ISO を落とします。

```bash
wget -O $HOME/jetson-iso-r39.2.1.iso https://developer.nvidia.com/downloads/embedded/l4t/r39_release_v2.1/iso/jetsoninstaller-r39.2.1-2026-08-07-18-30-47-arm64.iso
```

```bash
ls -lh $HOME/jetson-iso-r39.2.1.iso
```

数 GB あれば成功です。USB メモリを挿す前と後で一覧を見比べて、USB メモリの名前を確かめます。**ここを間違えると、Orin の中身が消えます。** `mmcblk0`（eMMC）と `nvme0n1`（NVMe）は選ばないでください。

```bash
lsblk
```

自動でマウントされていたら外します（`sda1` は例。パーティションの数だけ実行します）。

```bash
sudo umount /dev/sda1
```

書き込みます（`sda` は確かめた名前に置き換えます）。ファイルとしてコピーするだけでは起動しません。

```bash
sudo dd if=$HOME/jetson-iso-r39.2.1.iso of=/dev/sda bs=4M status=progress conv=fsync
```

```bash
sync
```

</details>

## 2. 入れる

### 画面の用意

モニタを使う場合は、DisplayPort のモニタと USB キーボードを直接つなぎます。KVM 切替器を通すと、画面が出ないことがあります。

モニタを使わない場合は、PC と、Orin の 40 ピンの横にある USB Type-C をつなぎ、115200 baud のシリアル端末で操作します。このとき DisplayPort には何もつなぎません。Linux の PC では、つないだ後に増えた名前を確かめて、`screen` で開きます。

```bash
sudo dmesg | grep --color tty
```

```bash
sudo screen /dev/ttyACM0 115200
```

`ttyACM0` は例です。Windows の PC では、Tera Term などで COM ポートを 115200 baud で開きます。

### 手順

1. バックアップが済んでいるかを、もう一度確かめます。
2. USB メモリを挿したまま再起動します。

   ```bash
   sudo reboot
   ```

3. USB メモリから自動で起動しなければ、起動中に UEFI のメニューを開き（画面の案内に従います。ふつうは ESC）、Boot Manager から USB メモリを選びます。
4. ファームウェア（QSPI）を更新するか聞かれたら、Y を押します。飛ばしてはいけません。更新は 2 回走ります。聞かれるのを見逃したら、入れ直しの手順を最初からやり直します。
5. メニューで「Install Jetson ISO r39.2.1」を選びます。
6. 入れ先を選びます。
   - Install on eMMC: 本体の eMMC に入れます。
   - Install on NVMe: NVMe の SSD に入れます。SSD があれば、こちらが勧められています。
7. 終わるまで待ちます。15 分ほどかかります。
8. 再起動の前に USB メモリを抜きます。抜かないと、また USB メモリから起動します。
9. 初期設定（oem-config）で、ユーザーを作ります。

## 3. 入れたあと

版を確かめます。`R39 (release), REVISION: 2.1` と出れば成功です。

```bash
cat /etc/nv_tegra_release
```

CUDA などの部品をまとめて入れます（ネットが要ります）。

```bash
sudo apt update
```

```bash
sudo apt install nvidia-jetpack
```

### Docker

ISO で入れた場合は、Docker と NVIDIA のランタイムが入っている見込みです。次の2つで確かめます。2つ目の出力の `Runtimes` に `nvidia` があれば、そのまま使えます。

```bash
docker --version
```

```bash
sudo docker info
```

入っていなければ、NVIDIA の手引きのとおりに入れます。

```bash
sudo apt install -y nvidia-container curl
```

```bash
curl https://get.docker.com | sh
```

```bash
sudo systemctl --now enable docker
```

```bash
sudo nvidia-ctk runtime configure --runtime=docker
```

```bash
sudo systemctl daemon-reload
```

```bash
sudo systemctl restart docker
```

`--runtime nvidia` を毎回付けなくて済むように、`/etc/docker/daemon.json` に `"default-runtime": "nvidia"` を足します。次のようになっていれば十分です。

```json
{
  "runtimes": {
    "nvidia": {
      "args": [],
      "path": "nvidia-container-runtime"
    }
  },
  "default-runtime": "nvidia"
}
```

書き換えたら、Docker を再起動します。

```bash
sudo systemctl restart docker
```

`sudo` なしで `docker` を使うには、自分を docker のグループに入れて、ログインし直します。

```bash
sudo usermod -aG docker $USER
```

### 元に戻す設定

固定 IP などのネットワークの設定、ssh の鍵、時計合わせを、バックアップから戻します。

ここまで済んだら、[Orin で VLM を動かして、速さを測る](vlm.md) に進みます。

## 4. 作り直しが要るもの

| もの | JetPack 7.2 で | 理由 |
| --- | --- | --- |
| GPU を使うコンテナ（omnivla_real の推論サーバなど） | 作り直し | JetPack 6 向け（L4T r36.4）のイメージをもとにしている。JetPack 7.2 は Jetson Linux r39.2、CUDA 13 |
| TensorRT のエンジン | 作り直し | JetPack 6 で作ったエンジンは、そのままでは使えない |
| ROS 2 Humble のコンテナ | そのまま動く見込み（未確認） | GPU を使わない |
| VLM のサーバ | 上流の vLLM（`vllm/vllm-openai:latest`）を使う | JetPack 7.2 の Orin では、上流の vLLM が使える |

## ほかの入れ方

| 方法 | 要るもの | 向いている場合 |
| --- | --- | --- |
| SDK Manager（Ubuntu の PC） | Ubuntu の x86 の PC、USB-C のケーブル、リカバリーモード | USB メモリから起動できないとき。いちばん確実 |
| SDK Manager（Windows の PC） | Windows 版の SDK Manager。WSL2 は SDK Manager が自動で用意する | Ubuntu の PC がないとき。JetPack 6.2.1 から公式に対応したが、書き込みの途中で USB のつながりが切れて失敗する報告が多い |

リカバリーモードは、電源を切った状態で Force Recovery ボタンを押したまま電源ボタンを押し、両方を離すと入ります。

## 参考

- [Jetson AGX Orin Developer Kit User Guide: Quick Start Guide](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/quick_start.html)（Jetson ISO での入れ方）
- [Jetson AGX Orin Developer Kit User Guide: BSP Installation](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/setup_bsp.html)（入れ方の選び方、r35.5 以上の条件）
- [Jetson AGX Orin Developer Kit User Guide: Docker Setup](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/setup_docker.html)
- [JetPack 7.2.1 の配布ページ](https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2.1)（ISO のリンク）
- [JetPack 7.2 のお知らせ（NVIDIA Developer Forums）](https://forums.developer.nvidia.com/t/jetpack-7-2-jetson-software-goes-agentic-with-jetson-linux-39-2/372056)（Orin への対応、`nvidia-jetpack` での部品の入れ方）
- [NVIDIA SDK Manager: Install Jetson Software with SDK Manager](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html)（Windows と WSL での書き込み）
- [NVIDIA skills: jetson-llm-serve](https://github.com/NVIDIA/skills)（JetPack 7.2 の Orin で上流の vLLM を使う）
- [Seeed Studio: JetPack 7.2 Resource Hub](https://wiki.seeedstudio.com/jetpack_7_2_resource_hub/)（JetPack 6 から移るときに作り直すもの）
