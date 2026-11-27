# 実機の手順（ロボットに載せた Orin で、VLM の指示で走る）

ロボットに載せた Orin で、「道に沿って走って」などの指示で走らせる手順です。コースの途中で VLM に走行を任せ、決めたところで終えて、元のシステム（ROS 1 のナビゲーション）に戻す使い方を前提にしています。仕組みは [道に沿って走る（follow_path）と、途中から・ここまで](../../elevator_vlm/docs/follow_path.md) を参照してください。

上から順に進めます。A と B はネットのあるところで、C 以降はロボットの上で行います。

| 段階 | 場所 | 内容 |
| --- | --- | --- |
| A | Orin（ネットあり） | 要るものを全部入れ、ネットなしで動くかを確かめる |
| B | 手元の x86 の PC → ロボットの PC | ROS 1 との橋渡しを作って入れる |
| C | ロボットの上 | 起動して、動かさずに確かめる |
| D | ロボットの上 | カメラの取り付けを合わせる |
| E | ロボットの上 | 短い距離で初めて動かす |
| F | ロボットの上 | 元のシステムからの合図で走らせる |
| G | コース | 区間を決めて使う |

## これまでに確かめたこと

| 項目 | 結果 |
| --- | --- |
| 実写（ATC のコース）での VLM の机上検証（Orin、MODE_30W、Qwen3.5-9B） | 道の指差し 33 枚中 32 枚が道の上（外れた 1 枚も奥の床を指していた）。止まる条件 50 問中 49 問正解。計画 3 つとも正しい。応答時間は指差し約 4.1 秒、判定約 2.3 秒（[記録](../log/path_eval_20261127_140726_Qwen3.5-9B-quantized.w4a16/report.md)） |
| 道のベンチ（Orin、本物の VLM） | 承認して開始の合図を待つ → ROS で `start` → 道に沿って走る → 止まる条件で止まる → 知らせのトピック、まで動いた。止まる条件「エレベーターの扉が目の前」は、扉の横の呼びボタンの壁の前（扉の 12 枚手前）で成り立った（早く止まる側） |
| 実機 | まだ |

MODE_30W では 1 回の答えに約 4 秒かかるので、止まって待ちながら進みます（平均 0.12〜0.16 m/s）。ロボットの上では MAXN にできるので、速くなる見込みです。

## A. Orin に要るものを入れる（ネットのあるところで）

リポジトリを最新にします。

```bash
cd ~/physical_ai_for_mobile_robot
```

```bash
git pull
```

### A-1. カメラのサービス

USB カメラをつないでから入れます（[jetson/camera/](../camera/)）。

```bash
bash jetson/camera/install.sh
```

### A-2. 点群を見る安全ゲートのサービス

```bash
bash safety_gate/install.sh
```

設定ファイル `/etc/safety-gate/config.yaml` を、実機に合わせて書き換えます。

```bash
sudo nano /etc/safety-gate/config.yaml
```

| 項目 | 書くもの |
| --- | --- |
| `footprint` | ロボットの足形（上から見た形の頂点。`base_link` の座標 [m]） |
| `topics` | `cmd_in: /vlm_cmd_vel`、`cloud: /cloud`、`cmd_out: /cmd_vel`（ふつうはこのまま） |
| `safety:` の下 | `publish_when_idle: false` と `idle_tail: 0.5` があること（ないと、VLM が走らせていない間も 0 を出し続け、元のシステムの指令とぶつかる） |

書き換えたら、起動し直します。

```bash
sudo systemctl restart safety-gate
```

### A-3. VLM の重みと、実行系のイメージ

一度起動すると、両方が入ります（初回は数十分）。

```bash
bash jetson/robot/robot_up.sh
```

カメラやロボットがつながっていないので「まだ準備ができていません」と出ますが、入れるだけならそれでかまいません。止めておきます。

```bash
bash jetson/robot/robot_down.sh --all
```

### A-4. ネットなしで起動できるかを確かめる

LAN ケーブルや Wi-Fi を外して、もう一度起動します。「ネットなしで起動します」と出て、VLM が「接続」になれば、要るものは全部入っています。

```bash
bash jetson/robot/robot_up.sh
```

```bash
bash jetson/robot/robot_down.sh
```

## B. ROS 1 との橋渡しを入れる（ロボットの PC）

`/vlm_drive/...`（開始と終了の合図、知らせ）を足したので、前に入れていても作り直します。手元の x86 の PC で作ります（ネットが要る。数十分）。

```bash
bash ros1_bridge/build_image.sh
```

できた `ros1_bridge/dist/ros1-bridge-bundle.tar` を USB メモリでロボットの PC に運び、置いたフォルダで入れます。

```bash
tar -xf ros1-bridge-bundle.tar
```

```bash
bash ros1-bridge-bundle/install.sh
```

前に入れていた場合、設定ファイルは上書きされません。`/etc/ros1-bridge/bridge.env` に次の 3 行があるかを確かめ、なければ足します。

```bash
sudo nano /etc/ros1-bridge/bridge.env
```

```text
VLM_COMMAND_TOPIC=/vlm_drive/command
VLM_STATE_TOPIC=/vlm_drive/state
VLM_ACTIVE_TOPIC=/vlm_drive/active
```

```bash
sudo systemctl restart ros1-bridge
```

詳しくは [ros1_bridge/](../../ros1_bridge/) を参照してください。

## C. 起動して、動かさずに確かめる（ロボットの上）

ロボットの電源を入れ、元のシステム（roscore）が動いている状態にします。ロボットは止めておきます。

### C-1. 電源モードを最大にする

ロボットの電源なら MAXN でも足ります。

```bash
sudo nvpmodel -m 0
```

### C-2. 起動する

```bash
cd ~/physical_ai_for_mobile_robot
```

```bash
bash jetson/robot/robot_up.sh
```

| WebUI | URL | 見るところ |
| --- | --- | --- |
| 実行系 | `http://<OrinのIP>:8080/` | 「入力」が「すべて受信中」、「VLM」が「接続」 |
| 安全ゲート | `http://<OrinのIP>:9001/` | 点群とロボットの足形が出て、判定が `IDLE`。理由に「速度指令は出していません」 |
| カメラ | `http://<OrinのIP>:9000/` | 映像が出る |

離れた PC から SSH でつないでいるときは、つなぐときに `-L 8080:localhost:8080 -L 9001:localhost:9001` を付けると、手元のブラウザの `http://localhost:8080/` などで見られます。

### C-3. トピックを確かめる

Orin で、実行系のコンテナの中の `ros2` コマンドを使います。オドメトリと点群が届いているか（数十 Hz と出る。Ctrl+C で止める）:

```bash
docker exec -it elevator-task /entrypoint.sh ros2 topic hz /odom
```

```bash
docker exec -it elevator-task /entrypoint.sh ros2 topic hz /cloud
```

VLM の側が `/cmd_vel` を出していないこと（元のシステムも止まっていれば、何も出なければよい）:

```bash
docker exec -it elevator-task /entrypoint.sh ros2 topic hz /cmd_vel
```

ロボットの PC（ROS 1）で、知らせが届いていること（`active: false` の JSON が出る）:

```bash
rostopic echo /vlm_drive/state
```

## D. カメラの取り付けを合わせる

VLM が指した画像の上の点を、床の上の点に直すのに使います。実機で測って、[task_robot.yaml](task_robot.yaml) の `camera:` を書き換えます。

```bash
nano jetson/robot/task_robot.yaml
```

| 項目 | 測るもの |
| --- | --- |
| `height_m` | 床からレンズの中心までの高さ [m] |
| `pitch_deg` | 下向きの角度 [度]。水平なら 0 |
| `x_offset_m` | ロボットの中心（車輪の軸の真ん中）からレンズまでの前後の距離 [m] |

書き換えたら、実行系だけ起動し直します。

```bash
bash jetson/robot/robot_up.sh
```

確かめ方: ロボットの前、ちょうど 2 m の床に目印（テープなど）を置きます。台車の非常停止を押してロボットが動かないようにしてから、実行系の画面の「計画の JSON を直接編集する／読み込む」に次の計画を貼って読み込み、「承認して実行」を押します。

```json
{"steps": [{"instruction": "床の目印の上", "when": {"type": "now"},
  "do": {"skill": "go_to_visual_target", "target": "the tape mark on the floor"}, "timeout_s": 30}]}
```

記録の欄の `point_ground` の `ahead_m` が 2.0 前後なら合っています（ロボットは動けないので、その後「発進できない」で止まります）。大きくずれていたら `pitch_deg` を直します（遠く出るなら角度を大きく）。

## E. 短い距離で初めて動かす

- 周りに人や物がない、広くまっすぐな通路で行います。
- 台車の非常停止をすぐ押せる人が付きます。画面の「停止」は VLM の側を止めるだけです。
- 元のシステムのナビゲーションは止めておきます（`/cmd_vel` を出さない状態）。

1. 実行系の画面で「計画の JSON を直接編集する／読み込む」を開き、「用意済み: path_10m.json」を押す
2. 計画の表に「道に沿って走る … 10 m 走ったら終わり … 答え 1 回で最大 1 m」と出ているのを確かめる
3. 「承認して実行」を押す
4. 左下の「道に沿って走る」で、走った距離、答えの数と応答時間を見る。途中でやめるときは「ここまで（終了）」

確かめること:

| 項目 | 期待 |
| --- | --- |
| 動き | 道の真ん中あたりを進む。答えを待つ間は止まり、また進む |
| 安全ゲートの画面 | 走っている間は `PASS`。前に人が立つと `SLOW` か `STOP` で止まる（人が試すときは、ゆっくり近づく） |
| 終わった後 | 安全ゲートが `IDLE` に戻り、「速度指令は出していません」 |
| 応答時間（画面の「直近 ○ 秒」） | MAXN で 2 秒以下なら、ほぼ止まらずに進む |

## F. 元のシステムからの合図で走らせる

1. 実行系の画面で計画を読み込み、「承認して開始の合図を待つ」を押す（「VLM の走行」が「開始の合図を待っています」になる）
2. ロボットの PC（ROS 1）から開始の合図を出す

```bash
rostopic pub -1 /vlm_drive/command std_msgs/String "data: 'start'"
```

3. 走っている間は `/vlm_drive/active` が true。終えるときは `finish` を出す（距離や止まる条件で先に終わることもある）

```bash
rostopic pub -1 /vlm_drive/command std_msgs/String "data: 'finish'"
```

```bash
rostopic echo /vlm_drive/active
```

4. 終わり方は `/vlm_drive/state` の `result`（`done` 完了、`finished` 合図で終了、`failed` 失敗、`stopped` 停止）

元のシステムに組み込むときの約束:

- VLM に任せる区間の始まりで、自分の `/cmd_vel` を止めてから `start` を出す。
- `/vlm_drive/active` が true の間は `/cmd_vel` を出さない。false に戻ったら、`/vlm_drive/state` の `result` を見て再開する。
- 承認は残るので、区間ごとに範囲を変えて何度でも走らせられる。範囲つきの合図は JSON で出す（例: 手順 2〜3）。

```bash
rostopic pub -1 /vlm_drive/command std_msgs/String "data: '{\"command\": \"start\", \"from\": 2, \"to\": 3}'"
```

## G. コースで使う

- 机上検証で道を指せた場面（屋外の歩道、屋内の通路、スロープなど）を、VLM に任せる区間の候補にします。
- 止まる条件の文は、なるべく具体的にします。ベンチでは「エレベーターの扉が目の前」が、扉の横の呼びボタンの壁の前で成り立ちました。止める場所をはっきりさせるなら、例えば「The closed metal doors of an elevator fill the center of the image, less than 2 m away.」のように書きます。新しい文は、実写で確かめてから使います（[jetson/bench/](../bench/) の 8.）。
- 止まる条件が成り立たなかったときのために、距離（`distance_m`）か `finish` を併せて使います。
- 区間ごとの計画を 1 つの計画にまとめて承認しておくと、元のシステムは `from`・`to` で区間を選べます。

## 止める

実行系だけを止めます。

```bash
bash jetson/robot/robot_down.sh
```

VLM のサーバも止めるときは `--all` を付けます。

```bash
bash jetson/robot/robot_down.sh --all
```

実行系のコンテナは、Orin を起動し直すと自動で起動します（止めた後は起動しない）。

## 記録

VLM に渡した画像、問いと答え、経過は `elevator_vlm/runs/<日時>/` に残ります。共有するときは、そのフォルダを `jetson/log/` にコピーしてコミットします。

## うまくいかないとき

| ようす | 見るところ |
| --- | --- |
| 「オドメトリが未受信」 | ロボットの PC の橋渡し（`journalctl -u ros1-bridge -f`）、両方の `ROS_DOMAIN_ID` が 42 か |
| 「カメラ画像が未受信」 | `systemctl status usb-camera`、ブラウザで `http://<OrinのIP>:9000/` |
| 起動のログに「時刻が … 秒ずれています」 | ロボットの PC と Orin の時計が合っていない。直して使うので動くが、時計を合わせるのが確実 |
| 安全ゲートが `STOP` のまま | 理由の欄（「TF がありません」「点群が途切れました」）。`journalctl -u safety-gate -f` |
| 走らせていない間、元のシステムの指令でロボットが動かない | 安全ゲートの設定に `publish_when_idle: false` があるか（A-2） |
| 「道の指差しが続けて使えません」で止まる | 記録の `path_point_rejected` の理由。「近すぎます」が多ければ D のカメラの取り付けを見直す |
| 進みが遅い | 電源モード（C-1）、画面の応答時間。遅いままなら `task_robot.yaml` の `skills.follow_path.step_m` を 1.5 にする |
| 開始の合図で走らない | 画面で「承認して開始の合図を待つ」を押したか、「停止」のままでないか。画面の「外からの合図」の欄に理由が出る |
