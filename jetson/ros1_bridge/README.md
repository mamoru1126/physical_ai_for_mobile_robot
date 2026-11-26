# ROS 1 のシステムとの橋渡し（自動で起動）

ROS 1 で動いているロボットのシステムと、Orin の ROS 2 をつなぎます。Docker のコンテナとして動き、電源を入れるたびに自動で起動します。

| 項目 | 状態 |
| --- | --- |
| スクリプト | 用意した。Orin ではまだ通していない（シェルの文法だけ確かめた） |

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
| コンテナ | 公式の `ros:foxy-ros1-bridge`（ROS 1 Noetic と ROS 2 Foxy、arm64 もある） |
| 自動の起動 | systemd のサービス `ros1-bridge` が、Docker のコンテナ `ros1-bridge` を起動する。止まったら 5 秒後に起動し直す |

- ROS 1 と ROS 2 を両方入れられるのは Ubuntu 20.04 だけなので、コンテナの中は Foxy です。Orin の Humble とは、ふつうのメッセージならそのままやりとりできます。
- ROS 2 のドメインは、omnivla_real と同じ 42 にしてあります。
- 向こうの roscore が動いていなければ、動くまで待ちます。roscore が止まったら（15 秒ほど見えなかったら）、起動し直して、また待ちます。
- 橋渡しは、両側に送り手と受け手がそろったトピックだけ流れます。こちらで `/cmd_vel` を出すノードが動いていなければ、向こうには何も流れません。
- 橋渡しのコンテナは UDP だけでやりとりします（Foxy と Humble の共有メモリの通信は相性が悪いことがあるため）。

## 入れる前に決めること

設定は [bridge.env](bridge.env) にあります。最低限、`ROS_MASTER_URI` を、向こうで roscore が動いている PC に書き換えます。

```bash
nano jetson/ros1_bridge/bridge.env
```

| 項目 | 既定 | 意味 |
| --- | --- | --- |
| `ROS_MASTER_URI` | http://192.168.0.100:11311 | 向こうの roscore。必ず書き換える |
| `ROS1_IP` | auto | この Orin の IP。auto なら、roscore の PC へ届く IP を自動で選ぶ |
| `ROS1_HOSTS` | なし | 向こうの PC が名前（ホスト名）で名乗るときに、名前と IP を書く。例: `atmobi-pc:192.168.0.100` |
| `ODOM_TOPIC` | /odom | 受けるオドメトリのトピック |
| `CLOUD_TOPIC` | /cloud | 受ける点群のトピック |
| `CMD_VEL_TOPIC` | /cmd_vel | 流す速度の指令のトピック |
| `ROS_DOMAIN_ID` | 42 | ROS 2 のドメイン |

## 入れる

このリポジトリのフォルダで実行します。初回はイメージを取ってくるので、数分かかります（ネットが要ります）。

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

## 動いているか確かめる

ログを見ます。「roscore につながりました」と出れば、つながっています。

```bash
journalctl -u ros1-bridge -f
```

Orin の ROS 2 から、向こうのトピックが届いているか見ます。

```bash
export ROS_DOMAIN_ID=42
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
| 「roscore を待っています」のまま | `ROS_MASTER_URI` の IP とポートが合っているか。Orin から `ping 192.168.0.100` が通るか |
| つながったのに、トピックが届かない | 向こうの PC が名前で名乗っていて、Orin から名前が引けないことが多い。向こうで `export ROS_IP=<向こうのIP>` をしてからノードを起動するか、`ROS1_HOSTS` に名前と IP を書く |
| `/cmd_vel` が向こうに届かない | 向こうの PC から Orin に届くか（`ping <OrinのIP>`）。`ROS1_IP` が、向こうから届く IP になっているか（ログの「この PC の IP」） |
| 型が対応していないと出る | `ros1_bridge` が知っている型か確かめる: `docker exec ros1-bridge bash -c "source /opt/ros/noetic/setup.bash; source /opt/ros/foxy/setup.bash; ros2 run ros1_bridge dynamic_bridge --print-pairs"` |
| ROS 2 の側で同じトピックが 2 つ出ている | `/odom` などを ROS 2 で直接出しているノード（ATMOBI の PC など）と、同時に動かさない。動かすなら、`ODOM_TOPIC` などで名前を変える（ROS 1 と ROS 2 で同じ名前になる） |

## 気をつけること

- 速度の指令をどう扱うか（止まったときに止まるか、など）は、向こうのロボットの仕組みしだいです。こちらのノードが止まって指令が来なくなったとき、ロボットが止まるかを、先に確かめてください。
- 点群（`/cloud`）は大きいので、ネットワークが細いと遅れます。遅れるときは、向こうで点群を間引くか、送る回数を減らしてください。
