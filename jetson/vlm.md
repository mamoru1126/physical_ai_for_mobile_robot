# Orin で VLM を動かして、速さを測る

JetPack 7.2 を入れたあとの手順です。3090 の PC と同じ VLM を Orin で動かし、エレベーターのタスクで使う「扉の判定」と「指差し」の応答時間と正解率を測ります。結果を見て、使うモデルの大きさ（9B・4B・2B）を決めます。

| 項目 | 状態 |
| --- | --- |
| 手順とスクリプト | 用意した。Orin ではまだ通していない（スクリプトは、偽物の docker と模擬の VLM サーバで動きを確かめた） |
| Orin での応答時間と正解率 | まだ測っていない（この手順で測る） |

## 流れ

| 手順 | やること | 時間の目安 |
| --- | --- | --- |
| 1 | リポジトリを取り、Orin の状態を確かめる | 数分 |
| 2 | 電源モードを最大にする | 1 分 |
| 3 | VLM を動かす | 初回は数十分（イメージと重みを落とす） |
| 4 | 測る | 数分（全部の言い回しを比べるなら 10〜20 分） |
| 5 | ほかの大きさのモデルでも測る | 1 つにつき、3 と 4 の時間 |
| 6 | 結果を送る | |

## 1. 準備

JetPack を入れ直したので、前に取ったリポジトリは消えています。取り直します。

```bash
git clone https://github.com/mamoru1126/physical_ai_for_mobile_robot.git
```

```bash
cd physical_ai_for_mobile_robot
```

測るための Python の部品を入れます。

```bash
sudo apt update
```

```bash
sudo apt install -y python3-pil python3-numpy python3-yaml
```

Orin の状態を確かめます。何も変えません。結果は OK（そのままでよい）、注意（動くが、直した方がよい）、NG（直さないと先に進めない）で出ます。

```bash
bash jetson/check.sh
```

| 出たもの | 直し方 |
| --- | --- |
| NG: JetPack 7.2 ではない | [jetson/README.md](README.md) の手順で入れ直す |
| NG: nvidia-jetpack が入っていない | `sudo apt install nvidia-jetpack` |
| NG: Docker、または NVIDIA のランタイムがない | [jetson/README.md](README.md) の「Docker」 |
| NG: Python の部品が足りない | 上の `apt install` |
| 注意: sudo なしで docker を使えない | `sudo usermod -aG docker $USER` のあと、ログインし直す（そのままでも、スクリプトは sudo で動く） |
| 注意: 電源モードが MAXN ではない | 次の「2. 電源モード」 |

## 2. 電源モードを最大にする

Orin は、電源モードによって速さが大きく変わります。測る前に最大（MAXN）にします。まず、選べるモードの一覧と、今のモードを見ます。

```bash
sudo nvpmodel -q --verbose
```

AGX Orin 開発キットの JetPack 7.2.1 では、モード 0 が MAXN です（既定はモード 2 の MODE_30W）。32GB 版では、JetPack 7.2 から MAXN_SUPER も選べます。一覧に MAXN_SUPER があれば、その番号を使います。

```bash
sudo nvpmodel -m 0
```

クロックを最大に固定し、固定されたことを確かめます。

```bash
sudo jetson_clocks
```

```bash
sudo jetson_clocks --show
```

- 電源モードは、再起動しても残ります。`jetson_clocks` は再起動すると元に戻るので、測る前に毎回実行します。
- `jetson_clocks` のあとで電源モードを変えると、再起動が要ります。電源モードを先に決めます。
- 再起動を求められたら、従います。

## 3. VLM を動かす

```bash
bash jetson/vlm_up.sh
```

- モデルの既定は Qwen3.5-9B の 4bit 版（`RedHatAI/Qwen3.5-9B-quantized.w4a16`）です。3090 で使っているものと同じで、Jetson AI Lab が Orin 用として挙げている重みです。
- vLLM は上流のイメージ（`vllm/vllm-openai:v0.22.0-ubuntu2404`）を使います。NVIDIA が、AGX Orin の JetPack 7.2 で動かして見せた版です。JetPack 6 の Orin では動きません。
- 初回は、イメージと重み（約 11 GB）を落とすので時間がかかります。起動に 15 分ほどかかったという報告もあります。スクリプトは、答えられるようになるまで待ち、30 秒ごとにログの最後の行を出します。
- 設定は 3090 の PC（`elevator_vlm/docker-compose.yml`）とそろえてあります（モデル名 `vlm`、問いの長さの上限 8192、同時に受け付ける問い 8）。結果を 3090 と比べられます。
- Orin は CPU と GPU でメモリを分け合います。VLM には 20 GiB を使わせます（メモリ全体に対する割合に直して渡します）。

設定は環境変数で変えられます。例：`VLM_GPU_MEMORY_GIB=16 bash jetson/vlm_up.sh`

| 環境変数 | 既定 | 意味 |
| --- | --- | --- |
| `VLM_GPU_MEMORY_GIB` | 20 | VLM に使わせるメモリ [GiB] |
| `VLM_GPU_MEMORY_UTILIZATION` | なし | メモリ全体に対する割合を直接指定する（例 0.4） |
| `VLM_MAX_MODEL_LEN` | 8192 | 1 回の問いの長さの上限 [トークン] |
| `VLM_MAX_NUM_SEQS` | 8 | 同時に受け付ける問いの数 |
| `VLM_EXTRA_ARGS` | なし | `vllm serve` に足す引数（例 `--enable-prefix-caching`）。空白で区切る |
| `VLLM_IMAGE` | `vllm/vllm-openai:v0.22.0-ubuntu2404` | vLLM のイメージ |
| `VLM_PORT` | 8000 | ポート |
| `HF_TOKEN` | なし | Hugging Face のトークン。ここで使うモデルは公開なので要らない |

動いているかは、次で確かめられます。

```bash
curl -s --noproxy '*' http://127.0.0.1:8000/v1/models
```

止めるときは、次を実行します。

```bash
bash jetson/vlm_down.sh
```

## 4. 測る

```bash
bash jetson/vlm_bench.sh
```

次のことをして、結果を `jetson/log/<日時>_<モデル>/` に残します。問いかけは 1 回ずつ順に行うので、出てくる応答時間が、実機で 1 回待つ時間の目安になります。

- 設定の確認：合成した画像での yes / no の判定、指差しの座標の約束、3 つの指示からの計画づくり
- 机上評価：Gazebo で撮った正解つきの画像 39 枚（判定 35 枚、指差し 4 枚）を、今の既定の言い回しで問いかける
- 測っている間の GPU の使用率・温度・電力（`tegrastats`）

| ファイル | 中身 |
| --- | --- |
| `env.txt` | 測ったときの状態（JetPack、メモリ、電源モード、モデル、vLLM の版と起動の引数） |
| `check_vlm.txt` | 設定の確認と机上評価の結果 |
| `tegrastats.log` | GPU の使用率など（1 秒ごと） |
| `vlm.log` | VLM のサーバのログの最後 |

見るところは次の 4 つです。

1. 設定の確認（[1]〜[3]）が、すべて OK になっているか。
2. 「応答時間（1回あたり）」の、判定と指差しの値。
3. 判定の正解数。3090 では 35 枚中 31 枚でした。
4. まとめに出る GPU の使用率の最大。0% なら、GPU を使っていません。

結果は、次の表のように 3090 と比べます。判定が 1 回 0.3 秒以内なら、今の作り（1 秒に 2 回判定する）のまま使える見込みです。

| | 3090（参考） | Orin 9B | Orin 4B | Orin 2B |
| --- | --- | --- | --- | --- |
| 判定の応答（平均） | 0.20 秒 | | | |
| 指差しの応答（平均） | 0.34 秒 | | | |
| 判定の正解 | 31 / 35 | | | |

3090 の値は、`elevator_vlm/log/` の 1 回目と 2 回目の実験から取ったものです。

言い回しの候補をすべて比べるときは、`--full` を付けます。

```bash
bash jetson/vlm_bench.sh --full
```

## 5. ほかの大きさのモデルでも測る

いったん止めてから、モデルを指定して起動し直し、同じように測ります。

| モデル | 重み | 備考 |
| --- | --- | --- |
| Qwen3.5-9B | `RedHatAI/Qwen3.5-9B-quantized.w4a16` | 既定。Jetson AI Lab の Orin 用 |
| Qwen3.5-4B | `RedHatAI/Qwen3.5-4B-quantized.w4a16` | Jetson AI Lab の Orin 用 |
| Qwen3.5-2B | `Qwen/Qwen3.5-2B` | 元の重み（16bit）。Orin 用の 4bit 版は Jetson AI Lab に載っていない |

4B の場合：

```bash
bash jetson/vlm_down.sh
```

```bash
bash jetson/vlm_up.sh RedHatAI/Qwen3.5-4B-quantized.w4a16
```

```bash
bash jetson/vlm_bench.sh
```

2B の場合：

```bash
bash jetson/vlm_down.sh
```

```bash
bash jetson/vlm_up.sh Qwen/Qwen3.5-2B
```

```bash
bash jetson/vlm_bench.sh
```

## 6. 結果を送る

Orin から、そのままプッシュする場合：

```bash
git add jetson/log
```

```bash
git commit -m "add orin vlm log"
```

```bash
git push
```

Orin で GitHub にログインしていなければ、`git push` のときにユーザー名とトークンを聞かれます（パスワードは使えません）。

Orin からプッシュしない場合は、PC に `jetson/log/` をコピーして、PC からプッシュします。PC で実行します。

```bash
scp -r <ユーザー名>@<OrinのIP>:~/physical_ai_for_mobile_robot/jetson/log .
```

## 7. このあと（結果を見てから）

- **速さが足りないとき**：問い方を変えて速くします。答えを 1 語にする、画像を小さくする、判定と指差しを同時に頼む、`--enable-prefix-caching` を付ける、などです。どれだけ速くなるかを、同じ道具で測ります。
- **モデルが決まったら**：実機につなぎます。ロボットのカメラの圧縮画像（`/camera/compressed`）への対応、画像の時刻の扱い、カメラの取り付けの実測、指令を `/elevator/cmd_vel/raw` に出して ATMOBI 側で `/cmd_vel` に切り替える、の 4 つが要ります。
- **omnivla_real の推論サーバ**：JetPack 7.2 用に作り直しが要ります（[jetson/README.md](README.md) の「4. 作り直しが要るもの」）。

## 困ったとき

| 症状 | 対処 |
| --- | --- |
| 起動中に `Free memory on device ... is less than desired GPU memory utilization` と出て止まる | ほかのプログラムやファイルのキャッシュがメモリを使っています。`VLM_GPU_MEMORY_GIB=16` などで減らすか、下のコマンドでキャッシュを空けてから起動し直します |
| メモリが足りずに止まる（out of memory） | `VLM_GPU_MEMORY_GIB` を下げるか、`VLM_MAX_MODEL_LEN=4096` や `VLM_MAX_NUM_SEQS=4` で減らします |
| `--gpus` のエラーで起動しない | `VLM_DOCKER_GPUS=` を付けて起動します（`--gpus` を外します） |
| 起動に何十分もかかる | 初回はダウンロードがあります。2 回目からは速くなります。`docker logs -f elevator-vlm` で様子を見られます |
| 応答に数秒かかる | `tegrastats.log` の `GR3D_FREQ` が 0% のままなら、GPU で動いていません（コンテナが CPU で動いたという報告があります）。`vlm.log` を送ってください |
| プロキシの下で、イメージを取れない | Docker のデーモンにプロキシの設定が要ります（`/etc/systemd/system/docker.service.d/` に置く）。重みのダウンロードには、シェルのプロキシの設定をスクリプトが自動で渡します |

キャッシュを空けるコマンドです。

```bash
sync
```

```bash
sudo sysctl -w vm.drop_caches=3
```

## 参考

- [JetPack 7.2 / Jetson Linux r39.2 on Jetson AGX Orin Developer Kit — Getting Started and feedback thread（NVIDIA Developer Forums）](https://forums.developer.nvidia.com/t/jetpack-7-2-jetson-linux-r39-2-on-jetson-agx-orin-developer-kit-getting-started-and-feedback-thread/372156)（上流の vLLM のイメージを AGX Orin で動かす手順、起動の時間などの報告）
- [Jetson AI Lab: Qwen3.5 9B](https://www.jetson-ai-lab.com/models/qwen3-5-9b) と [Qwen3.5 4B](https://www.jetson-ai-lab.com/models/qwen3-5-4b/)（Orin 用の重み）
- [NVIDIA skills: jetson-llm-serve](https://github.com/NVIDIA/skills)（JetPack 7.2 の Orin では上流の vLLM を使う）
- [RidgeRun: NVIDIA Jetson Orin JetPack 7.2 Tuning Power](https://developer.ridgerun.com/wiki/index.php/NVIDIA_Jetson_Orin/JetPack_7.2/Performance_Tuning/Tuning_Power)（AGX Orin の電源モードの一覧、`nvpmodel` と `jetson_clocks` の注意）
