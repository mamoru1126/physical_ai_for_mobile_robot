# 実機で動かす（ロボットに載せた Orin）

ロボットに載せた Orin で、VLM の指示で走る実行系を動かします。コースの途中で VLM に走行を任せ（「道に沿って走って」など）、決めたところで終えて、元のシステム（ROS 1 のナビゲーション）に戻す使い方を前提にしています。仕組みは [道に沿って走る（follow_path）と、途中から・ここまで](../../elevator_vlm/docs/follow_path.md) を参照してください。

| 項目 | 状態 |
| --- | --- |
| スクリプトと設定 | 用意した。実機ではまだ動かしていない |
| 同じ部品の組み合わせ（実行系、安全ゲート 2 段、合図と知らせのトピック） | 道のベンチで、モックの VLM を使って GitHub Actions で確認 |

## つながり

```text
USB カメラ ─ /camera/compressed ─▶ 実行系（elevator-task）◀─ HTTP ─▶ VLM（elevator-vlm）
                                     │ cmd_vel_raw → 速度の上限 → /vlm_cmd_vel
                                     ▼
ロボットの PC（ROS 1）◀─ 橋渡し ─ /cmd_vel ◀─ 安全ゲート（点群を見る。safety-gate）
       │                        ▲
       └─ /odom、/cloud、/tf ────┘（橋渡し）
元のシステム ─ /vlm_drive/command（start、finish、stop）─▶ 実行系
元のシステム ◀─ /vlm_drive/state、/vlm_drive/active ─ 実行系
```

ROS 2 のドメインは 42 です（カメラ、安全ゲート、橋渡しと同じ）。

## 0. ネットのあるところで入れておくもの

ロボットに載せるとネットにつながらないので、先に入れておきます。リポジトリのフォルダで、1 行ずつ実行します。

```bash
cd ~/physical_ai_for_mobile_robot
```

```bash
git pull
```

カメラのサービス（[jetson/camera/](../camera/)）:

```bash
bash jetson/camera/install.sh
```

点群を見る安全ゲートのサービス（[safety_gate/](../../safety_gate/)）。足形とトピックは、実機に合わせて `/etc/safety-gate/config.yaml` を書き換えます:

```bash
bash safety_gate/install.sh
```

前に入れたことがあると、設定ファイルは上書きされません。`/etc/safety-gate/config.yaml` の `safety:` の下に、次の 2 行があるかを確かめます。ないと、VLM が走らせていない間も安全ゲートが 0 を出し続け、元のシステムの指令とぶつかります。

```yaml
  publish_when_idle: false
  idle_tail: 0.5
```

書き足したら、起動し直します。

```bash
sudo systemctl restart safety-gate
```

VLM の重みと、実行系のイメージ（このスクリプトを一度動かすと、両方が入る）:

```bash
bash jetson/robot/robot_up.sh
```

カメラやロボットがつながっていなければ「まだ準備ができていません」と出ますが、入れるだけならそれでかまいません。ベンチを使うときは、先に止めておきます（ポート 8080 を使うため）。

```bash
bash jetson/robot/robot_down.sh
```

ロボットの PC には、ROS 1 との橋渡し（[ros1_bridge/](../../ros1_bridge/)）を入れておきます。`/vlm_drive/...` のトピックも橋渡しします（`bridge.env` の `VLM_COMMAND_TOPIC` など）。

## 1. カメラの取り付けを設定に書く

道の上の点を床の上の点に直すのに使います。実機で測って、[task_robot.yaml](task_robot.yaml) の `camera:` を書き換えます。

| 項目 | 測るもの |
| --- | --- |
| `height_m` | 床からレンズの中心までの高さ [m] |
| `pitch_deg` | 下向きの角度 [度]。水平なら 0 |
| `x_offset_m` | ロボットの中心（車輪の軸の真ん中）からレンズまでの前後の距離 [m] |

ずれていると、点までの距離を短く（長く）見積もります。1 回の答えで進むのは最大 1.0 m なので、大きくは外れませんが、近すぎると判断して答えを捨てることが増えます。実行系の画面のカメラ映像で、緑の印（進んでよい点）が VLM の点（赤）の方向の床の上に出ていれば合っています。

## 2. 起動する（ロボットの上で）

```bash
bash jetson/robot/robot_up.sh
```

ネットがなければ、VLM を落としてある重みで起動します。電源モードは MAXN にしてよいです（ロボットの電源なら足りる）。

| WebUI | URL |
| --- | --- |
| 実行系 | `http://<OrinのIP>:8080/` |
| 安全ゲート | `http://<OrinのIP>:9001/` |

実行系の画面の「入力」が「すべて受信中」、「VLM」が「接続」になっていれば準備ができています。「オドメトリが未受信」なら、ロボットの PC の橋渡しを確かめます。

## 3. VLM に任せる区間を走らせる

1. 画面で計画を作る（例「道に沿って走って」）か、用意済みの計画を読み込む。計画の表で、距離（「10 m 走ったら終わり」など）と止まる条件を確かめる。
2. 「承認して開始の合図を待つ」を押す。すぐ走らせるなら「承認して実行」。
3. 元のシステムが、VLM に任せる区間の始まりで `start` を出す。

ROS 1 の側で手で試すときは、次のように出します。

```bash
rostopic pub -1 /vlm_drive/command std_msgs/String "data: 'start'"
```

区間の終わりで、元のシステムが `finish` を出す（止まる条件や距離で先に終わることもある）。

```bash
rostopic pub -1 /vlm_drive/command std_msgs/String "data: 'finish'"
```

VLM が走らせている間は、`/vlm_drive/active` が true です。元のシステムは、この間 `/cmd_vel` を出さないでください。

```bash
rostopic echo /vlm_drive/active
```

終わり方と結果は `/vlm_drive/state` に出ます（`result` が `done`、`finished`、`failed`、`stopped`）。

```bash
rostopic echo /vlm_drive/state
```

## 4. 止める

実行系だけを止めます。

```bash
bash jetson/robot/robot_down.sh
```

VLM のサーバも止めるときは `--all` を付けます。

```bash
bash jetson/robot/robot_down.sh --all
```

実行系のコンテナは、Orin を起動し直すと自動で起動します（`--restart unless-stopped`）。止めた後は起動しません。

## 安全について

- 障害物で止めるのは、点群を見る安全ゲートです。入っていないと、起動のスクリプトが注意を出します。
- 画面の「停止」は、VLM の側の指令を止めるだけです。ロボット全体の非常停止は、台車の非常停止ボタンで行います。
- 初めて走らせるときは、`path_10m.json` のような短い距離の計画で、人が近くで見ながら確かめてください。
- 止まる条件の文は、本番の前に実写で確かめます（[jetson/bench/](../bench/) の 8.）。止まらなかったときのために、距離の上限か `finish` を併せて使います。

## 記録

VLM に渡した画像、問いと答え、経過は `elevator_vlm/runs/<日時>/` に残ります。
