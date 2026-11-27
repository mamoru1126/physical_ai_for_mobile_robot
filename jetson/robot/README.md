# 実機の手順書（Orin を受け取った人向け）

上から順に、Orin の `~/physical_ai_for_mobile_robot` で実行します。

> [!WARNING]
> **最終の速度指令: `/cmd_vel`**（`/etc/safety-gate/config.yaml` の `topics.cmd_out`）
> ROS1システムの台車が受けるトピックと同じ名前にする（橋渡しは名前を変えられない）。変えたら `sudo systemctl restart safety-gate`。

> [!WARNING]
> **ロボットの足形: `/etc/safety-gate/config.yaml` の `footprint`**
> 今は仮の値（前後 0.70 m × 左右 0.60 m の四角）。実機を測って書き換えるまで走らせない。`base_link` から測る [m]（x が前、y が左）。変えたら `sudo systemctl restart safety-gate`。

## 受け渡しのときの状態

- 電源を入れると自動で起動: `usb-camera`（カメラ、WebUI 9000）、`safety-gate`（安全ゲート、WebUI 9001。走らせていない間は何も出さない）
- 入っているが止めてある: `elevator-vlm`（VLM）、`elevator-task`（実行系、WebUI 8080）→ 手順 6 で起動
- 設定は既定のまま（ROS 2 のドメイン 42、足形とカメラの取り付けは仮の値）。ネットなしで動く
- カメラとロボットはつないでいない

## 手順

### 1. 状態を見る

```bash
cd ~/physical_ai_for_mobile_robot
```

```bash
bash jetson/robot/orin_state.sh
```

→ `usb-camera` と `safety-gate` が `active`

### 2. カメラをつなぐ

USB カメラを挿してから:

```bash
bash jetson/robot/ros2.sh topic hz /camera/compressed
```

→ 30 前後（Ctrl+C で止める）

### 3. ROS 2 のドメインを、ROS1システムの橋渡しにそろえる

LAN でつないでから。橋渡しの番号の調べ方は [link_check.md](link_check.md)（番号が出なければ 0）。

```bash
bash jetson/robot/set_domain.sh 0
```

### 4. 安全ゲートを実機に合わせる

```bash
sudo nano /etc/safety-gate/config.yaml
```

⚠ `footprint`、⚠ `topics.cmd_out`、`topics.cloud`（点群のトピック）を書き換える。

```bash
sudo systemctl restart safety-gate
```

### 5. つながりを確かめる

```bash
bash jetson/robot/check_link.sh
```

→ `[NG]` がなくなるまで直す（直し方は `→` の行と、下の「設定の場所」）

### 6. 起動する

```bash
sudo nvpmodel -m 0
```

```bash
bash jetson/robot/robot_up.sh
```

→ 最後に「OK: 実行系を起動しました」

### 7. 速度の通り道を確かめる（ロボットは止めたまま）

ROS1システムで `rostopic echo /cmd_vel` を出しておき、Orin で:

```bash
bash jetson/robot/check_link.sh --zero
```

→ ROS1システムに 0 の指令が 3 秒ほど出る

### 8. カメラの取り付けを書く

```bash
sudo nano /etc/elevator-task/task.yaml
```

`camera:` の `height_m`（床からレンズ [m]）、`pitch_deg`（下向きの角度）、`x_offset_m`（`base_link` からレンズまで前後 [m]）、`hfov_deg`（水平画角）を実測値にする。

```bash
bash jetson/robot/robot_up.sh
```

### 9. VLM で走らせる（Orin から）

非常停止を押せる人を付け、ROS1システムのナビゲーションは止めておく（`/cmd_vel` を出さない）。

```bash
bash jetson/robot/drive.sh load path_10m.json
```

```bash
bash jetson/robot/drive.sh arm
```

```bash
bash jetson/robot/drive.sh start
```

途中で終えるとき（10 m 走れば自分で終わる）:

```bash
bash jetson/robot/drive.sh finish
```

様子を見るとき（Ctrl+C で終わる）:

```bash
bash jetson/robot/drive.sh state
```

VLM の側だけを止めるなら `bash jetson/robot/drive.sh stop`（解除は `bash jetson/robot/drive.sh reset`）。ロボット全体は台車の非常停止で止める。

### 10. ROS1システムから start / finish を出す（使うとき）

手順 9 の `arm` まで Orin で済ませてから、ROS1システムで:

```bash
rostopic pub -1 /vlm_drive/command std_msgs/String "data: 'start'"
```

```bash
rostopic pub -1 /vlm_drive/command std_msgs/String "data: 'finish'"
```

届いたかは Orin の `bash jetson/robot/drive.sh state` で見る。橋渡しが dynamic_bridge だと 1 回目を落とすことがあるので、走り出さなければもう一度出す。走っている間は `/vlm_drive/active` が true（その間、ROS1システムは `/cmd_vel` を出さない）。

## 止める

```bash
bash jetson/robot/robot_down.sh --all
```

`robot_up.sh` の後は、電源を入れ直しても VLM と実行系が自動で起動する（`robot_down.sh` で止めるまで）。

## 設定の場所

| もの | ファイル | 項目 | 既定 | 反映 |
| --- | --- | --- | --- | --- |
| ⚠ 最終の速度指令 | `/etc/safety-gate/config.yaml` | `topics.cmd_out` | `/cmd_vel` | `sudo systemctl restart safety-gate` |
| ⚠ 足形 | 同上 | `footprint` | 仮の四角 | 同上 |
| 点群 | 同上 | `topics.cloud` | `/cloud` | 同上 |
| 座標系 | 同上 | `frames.base`、`frames.cloud` | `base_link`、点群のヘッダ | 同上 |
| TF が来ないとき | 同上 | `tf.publish_static`、`xyz`、`rpy` | false | 同上 |
| 速さの上限 | 同上 | `limits.max_linear_x` | 0.5 m/s | 同上 |
| オドメトリ | `/etc/elevator-task/robot.env` | `ODOM_TOPIC` | `/odom` | `bash jetson/robot/robot_up.sh` |
| カメラの取り付け | `/etc/elevator-task/task.yaml` | `camera.*` | 仮の値 | 同上 |
| 走る速さ | 同上 | `skills.follow_path.speed` | 0.3 m/s | 同上 |
| 答え 1 回で進む距離 | 同上 | `skills.follow_path.step_m` | 1.0 m | 同上 |
| カメラの機器 | `/etc/usb-camera/camera.env` | `CAMERA_DEVICE` | auto | `sudo systemctl restart usb-camera` |
| ROS 2 のドメイン | 上の 3 つにある | — | 42 | `bash jetson/robot/set_domain.sh <番号>` |

## 困ったとき

| ようす | すること |
| --- | --- |
| 何かが届かない | `bash jetson/robot/check_link.sh` の `→` の行。ROS1システムの側は [link_check.md](link_check.md) |
| 安全ゲートが `STOP` のまま | `journalctl -u safety-gate -n 30`（「TF がありません」「点群が途切れました」） |
| VLM で走らせてもロボットが動かない | ⚠ `topics.cmd_out` が台車の名前か（手順 7） |
| 「道の指差しが続けて使えません」で止まる | カメラの取り付け（手順 8） |
| `robot_up.sh` が「イメージが…作ったものではありません」 | `ROBOT_REBUILD=1 bash jetson/robot/robot_up.sh`（下の「ソースを変えたとき」） |

## ソースを変えたとき

`elevator_vlm/` の中（実行系）を変えたら、イメージを作り直します。**ネットはなくてよい**です。

```bash
ROBOT_REBUILD=1 bash jetson/robot/robot_up.sh
```

- やり直しになるのは、ソースのコピーから後ろ（colcon build と単体テスト）だけです。apt で道具を入れる段は、前に作った分を使います。単体テストを全部通すので、Orin だと数分かかります。
- ネットが要るのは、前に作った分を消したとき（`docker system prune` など）と、`elevator_vlm/docker/Dockerfile.runtime` の apt の行を変えたときだけです。そのときは「イメージを作れませんでした」で止まるので、ネットのあるところで同じコマンドを実行します。
- 単体テストが落ちても「イメージを作れませんでした」で止まります（前のイメージはそのまま残る）。上のログで、どのテストが落ちたかを見ます。
- 安全ゲート（`safety_gate/`）とカメラ（`jetson/camera/`）は別のイメージです。変えたら、それぞれの `install.sh` で作り直します。

記録は `elevator_vlm/runs/<日時>/`。仕組みは [follow_path.md](../../elevator_vlm/docs/follow_path.md)。
