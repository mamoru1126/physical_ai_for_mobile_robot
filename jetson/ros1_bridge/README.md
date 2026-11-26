# ROS 1 のシステムとの橋渡し（自動で起動）

ROS 1 で動いているロボットのシステムと、Orin の ROS 2（Humble）をつなぎます。Docker のコンテナとして動き、電源を入れるたびに自動で起動します。ふつうは設定を書き換えずに使えます（roscore は同じネットワークから自動で探します）。

| 項目 | 状態 |
| --- | --- |
| スクリプト | 用意した。Orin ではまだ通していない。roscore を探す・つなぐ・止まったら起動し直す流れは、にせの roscore で確かめた。コンテナのイメージは、まだ作っていない |

## 橋渡しするトピック

| 向き | トピック | 型 |
| --- | --- | --- |
| ROS 1 → ROS 2（向こうのものを受ける） | `/odom` | `nav_msgs/Odometry` |
| ROS 1 → ROS 2（向こうのものを受ける） | `/cloud` | `sensor_msgs/PointCloud2` |
| ROS 2 → ROS 1（こちらから流す） | `/cmd_vel` | `geometry_msgs/Twist` |

## 中身

| もの | 中身 |
| --- | --- |
| 橋渡し | [ros1_bridge](https://github.com/ros2/ros1_bridge) の `parameter_bridge`（決めたトピックだけを橋渡しする） |
| ROS 2 | Humble（`ros:humble-ros-base-jammy`。カメラなど、ほかのコンテナと同じ） |
| ROS 1 | Ubuntu 22.04 が配っている ROS 1 のパッケージ（`ros-core-dev` など）。Noetic と同じ世代で、Noetic のロボットとそのままやりとりできる |
| 自動の起動 | systemd のサービス `ros1-bridge` が、Docker のコンテナ `ros1-bridge` を起動する。止まったら 5 秒後に起動し直す |

- Humble の Ubuntu 22.04 には、ROS 1 の公式のパッケージがありません。そこで、Ubuntu 自身が配っている ROS 1 を入れ、ros1_bridge をコンテナの中で作ります。やり方は ROS 2 の公式の手引き（[Using ros1_bridge with upstream ROS on Ubuntu 22.04](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)）と、[ros-humble-ros1-bridge-builder](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder) にならいました。
- イメージを作る最後に、`Odometry`、`PointCloud2`、`Twist` を橋渡しできるかを確かめます。できなければ、そこで止まります。
- ROS 2 のドメインは、omnivla_real と同じ 42 にしてあります。
- 橋渡しは、両側に送り手と受け手がそろったトピックだけ流れます。こちらで `/cmd_vel` を出すノードが動いていなければ、向こうには何も流れません。

## 設定を書かなくてよい理由と、書くとき

ROS 1 は、ROS 2 と違って、相手を自動で見つけません。すべてのノードが roscore の場所（`ROS_MASTER_URI`）を知っている必要があります。何も書かなければ「同じ PC の roscore」になるので、roscore と同じ PC で動かすときだけは、書かなくても動きます。

この橋渡しは、そこを自動でやります。

| 項目 | 自動でやること |
| --- | --- |
| `ROS_MASTER_URI` | 同じネットワーク（Orin のつながっている各ネットワークの /24）から、ポート 11311 で答える roscore を探す |
| `ROS1_IP` | roscore の PC へ届く、この Orin の IP を選ぶ（向こうのノードが、この IP でつないでくる） |
| `ROS1_HOSTS` | 向こうのノードが名前（ホスト名）で名乗っていて、Orin から名前が引けないときは、roscore と同じ PC とみなす |

書くのは、次のときだけです。設定は [bridge.env](bridge.env) にあります（入れたあとは `/etc/ros1-bridge/bridge.env`）。

| ようす | 書くもの |
| --- | --- |
| 同じネットワークに roscore が複数ある（ログに「roscore が複数あります」と出る） | `ROS_MASTER_URI=http://<ロボットのIP>:11311` |
| ロボットのノードが、roscore とは別の PC でも動いている | `ROS1_HOSTS=auto <その PC の名前>:<その PC の IP>` |
| トピックの名前が違う | `ODOM_TOPIC`、`CLOUD_TOPIC`、`CMD_VEL_TOPIC` |

## 入れる

このリポジトリのフォルダで実行します。初回は ros1_bridge を作るので、数十分かかります（ネットが要ります）。

```bash
bash jetson/ros1_bridge/install.sh
```

入れたあとに設定を変えるときは、`/etc/ros1-bridge/bridge.env` を書き換えて、サービスを起動し直します（`install.sh` は、すでにある設定ファイルを上書きしません）。

```bash
sudo nano /etc/ros1-bridge/bridge.env
```

```bash
sudo systemctl restart ros1-bridge
```

前の版（Foxy のもの）を入れていたときは、古い設定ファイルを消してから入れ直してください。

```bash
sudo rm /etc/ros1-bridge/bridge.env
```

```bash
bash jetson/ros1_bridge/install.sh
```

## 動いているか確かめる

ログを見ます。「roscore につながりました」と出れば、つながっています。

```bash
journalctl -u ros1-bridge -f
```

向こうのトピックが ROS 2 に届いているかは、橋渡しのコンテナの中で見ます（Orin 本体は Ubuntu 24.04 なので、Humble の `ros2` コマンドがありません）。

```bash
docker exec -it ros1-bridge bash
```

コンテナの中で、次を実行します。

```bash
source /opt/ros/humble/setup.bash
```

```bash
ros2 topic hz /odom
```

```bash
ros2 topic hz /cloud
```

こちらから速度の指令を流して、向こうに届くか見ます。**ロボットが動くので、周りに人や物がないこと、すぐ止められることを確かめてから**にしてください。止めるときは Ctrl+C です。

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

向こうの ROS 1 の PC で、届いているか見ます。

```bash
rostopic echo /cmd_vel
```

## 外す

```bash
bash jetson/ros1_bridge/uninstall.sh
```

## うまくいかないとき

| ようす | 見るところ |
| --- | --- |
| 「roscore を探しています」のまま | 向こうで roscore が動いているか。Orin から向こうに `ping` が通るか。別のネットワーク（/24 の外）にあるなら `ROS_MASTER_URI` を書く |
| つながったのに、トピックが届かない | 向こうのノードが、roscore とは別の PC で動いていないか（`ROS1_HOSTS` に名前と IP を書く）。向こうでノードを起動するときに `export ROS_IP=<向こうのIP>` をしておくと、名前の問題は起きない |
| `/cmd_vel` が向こうに届かない | 向こうの PC から Orin に届くか（`ping <OrinのIP>`）。ログの「この PC の IP」が、向こうから届く IP か |
| イメージを作るところで止まる | メモリが足りないと止まることがある。ほかのコンテナ（VLM など）を止めてから作り直す |
| ROS 2 の側で同じトピックが 2 つ出ている | 同じ名前のトピックを ROS 2 で直接出しているノードと、同時に動かさない。動かすなら、`ODOM_TOPIC` などで名前を変える（ROS 1 と ROS 2 で同じ名前になる） |

## 気をつけること

- 速度の指令をどう扱うか（指令が来なくなったら止まるか、など）は、向こうのロボットの仕組みしだいです。こちらのノードが止まって指令が来なくなったとき、ロボットが止まるかを、先に確かめてください。
- 点群（`/cloud`）は大きいので、ネットワークが細いと遅れます。遅れるときは、向こうで点群を間引くか、送る回数を減らしてください。
- roscore を探すときは、同じネットワークの各 IP のポート 11311 に、つなげるかを試します。人の管理するネットワークでこれが困るときは、`ROS_MASTER_URI` を書いてください（書けば探しません）。
