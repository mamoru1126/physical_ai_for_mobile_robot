# ROS 1 のシステムとの橋渡し（ロボットの PC で動かす、自動で起動）

ROS 1 で動いているロボットのシステムと、Orin などの ROS 2（Humble）をつなぎます。橋渡しは**ロボットの PC（ROS 1 が動いている PC）**で動かします。Docker のコンテナとして動き、電源を入れるたびに自動で起動します。

ロボットの PC はネットにつながっていないので、イメージは手元の x86 の PC で作り、ファイルにしてロボットの PC に運びます。

> [!NOTE]
> **ロボットの PC に、すでに橋渡しが入っているとき**（Foxy 用の `dynamic_bridge` など）は、それを使えます。このリポジトリの橋渡しは入れません（2 つ同時に動かすと、指令が二重になったりループしたりする）。すでにある橋渡しを調べて、Orin とつながるかを確かめる手順は [jetson/robot/link_check.md](../jetson/robot/link_check.md) です。

| 項目 | 状態 |
| --- | --- |
| イメージ | GitHub Actions（x86）で作れることを確かめた（[.github/workflows/ros1-bridge.yml](../.github/workflows/ros1-bridge.yml)）。`Odometry`、`PointCloud2`、`Twist`、`TFMessage`、`String`、`Bool` を橋渡しできる。大きさは圧縮して約 380 MB |
| スクリプト | 実機ではまだ通していない。一式をまとめる・ネットなしで入れる流れは、にせの Docker で確かめた。roscore につなぐ・止まったら起動し直す流れは、にせの roscore で確かめた |

## つながり方

```text
ロボットの PC                                        Orin
┌───────────────────────────────────────┐          ┌──────────────────┐
│ ROS 1 のノード ── roscore（localhost） │          │                  │
│        │ ROS 1（同じ PC の中だけ）     │          │                  │
│ 橋渡しのコンテナ（ros1-bridge）        │── ROS 2 ──│ ROS 2 のノード   │
└───────────────────────────────────────┘ ドメイン42 └──────────────────┘
```

- ROS 1 のやりとりは、ロボットの PC の中だけで済みます。ロボットの `ROS_MASTER_URI` が localhost のままでも動きます。
- PC の間をまたぐのは ROS 2 だけです。ROS 2 は相手を自動で見つけるので、ドメイン（42）をそろえれば、IP の設定は要りません。

## 橋渡しするトピック

| 向き | トピック | 型 |
| --- | --- | --- |
| ROS 1 → ROS 2（ロボットのものを Orin で受ける） | `/odom` | `nav_msgs/Odometry` |
| ROS 1 → ROS 2（ロボットのものを Orin で受ける） | `/cloud` | `sensor_msgs/PointCloud2` |
| ROS 2 → ROS 1（Orin からロボットへ流す） | `/cmd_vel` | `geometry_msgs/Twist` |
| 両方向 | `/tf`、`/tf_static` | `tf2_msgs/TFMessage` |
| ROS 1 → ROS 2（元のシステムから VLM へ、開始と終了の合図） | `/vlm_drive/command` | `std_msgs/String`（`start`、`finish`、`stop`、`ping`） |
| ROS 2 → ROS 1（VLM が走らせているかの知らせ） | `/vlm_drive/state` | `std_msgs/String`（JSON） |
| ROS 2 → ROS 1（同上） | `/vlm_drive/active` | `std_msgs/Bool` |

- TF は、[安全ゲート](../safety_gate/) が点群を `cloud_link` から `base_link` に直すのに使います。要らなければ `BRIDGE_TF=false` にします。
- `/vlm_drive/...` は、コースの途中で VLM に走行を任せ、決めたところで終える（途中から・ここまで）ための合図と知らせです（[説明](../elevator_vlm/docs/follow_path.md)）。要らなければ `VLM_COMMAND_TOPIC=` のように空にします。
- 合図は、ロボットの PC で [tools/vlm_drive.py](tools/vlm_drive.py) を使って出せます（`python3 vlm_drive.py ping` で届くかを確かめる、`start`、`finish`）。橋渡しの準備ができるのを待ってから出し、Orin からの答えを表示します。
- `/tf_static` は、ros1_bridge が自動で「全部とっておき、あとから起動した相手にも渡す」設定（keep all、transient local）にします。ROS 2 の側で、安全ゲートをあとから起動しても受け取れます。

## 中身

| もの | 中身 |
| --- | --- |
| 橋渡し | [ros1_bridge](https://github.com/ros2/ros1_bridge) の `parameter_bridge`（決めたトピックだけを橋渡しする） |
| ROS 2 | Humble（`ros:humble-ros-base-jammy`。Orin のカメラなど、ほかのコンテナと同じ） |
| ROS 1 | Ubuntu 22.04 が配っている ROS 1 のパッケージ（`ros-core-dev` など）。Noetic と同じ世代で、Noetic のロボットとそのままやりとりできる |
| 自動の起動 | systemd のサービス `ros1-bridge` が、Docker のコンテナ `ros1-bridge` を起動する。止まったら 5 秒後に起動し直す |

- Humble の Ubuntu 22.04 には、ROS 1 の公式のパッケージがありません。そこで、Ubuntu 自身が配っている ROS 1 を入れ、ros1_bridge をコンテナの中で作ります。やり方は ROS 2 の公式の手引き（[Using ros1_bridge with upstream ROS on Ubuntu 22.04](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)）と、[ros-humble-ros1-bridge-builder](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder) にならいました。
- コンテナの中は Ubuntu 22.04 なので、ロボットの PC が Ubuntu 20.04 などでも動きます。
- イメージを作る最後に、`Odometry`、`PointCloud2`、`Twist`、`TFMessage`、`String`、`Bool` を橋渡しできるかを確かめます。できなければ、そこで止まります。
- roscore が動くまで待ちます。roscore が止まったら（15 秒ほど見えなかったら）、起動し直して、また待ちます。ロボットの PC の電源を入れた直後に、roscore があとから起動しても大丈夫です。
- 橋渡しは、両側に送り手と受け手がそろったトピックだけ流れます。Orin で `/cmd_vel` を出すノードが動いていなければ、ロボットには何も流れません。

## 1. 手元の PC でイメージを作る（ネットが要る）

x86 の PC で、Docker が使えるものにします。このリポジトリのフォルダで実行します。初回は ros1_bridge を作るので、数十分かかります。

```bash
bash ros1_bridge/build_image.sh
```

終わると `ros1_bridge/dist/ros1-bridge-bundle.tar` ができます。イメージ・入れるスクリプト・設定がまとまっています（大きさは 1 GB 前後の見込みです）。

## 2. ロボットの PC に運ぶ

USB メモリにコピーして運びます。手元の PC からロボットの PC に LAN で届くなら、`scp` でも送れます。

```bash
scp ros1_bridge/dist/ros1-bridge-bundle.tar <ユーザー名>@<ロボットのPCのIP>:~/
```

## 3. ロボットの PC で入れる（ネットは要らない）

運んだファイルを置いたフォルダで実行します。

```bash
tar -xf ros1-bridge-bundle.tar
```

```bash
bash ros1-bridge-bundle/install.sh
```

ファイルが運ぶ途中で壊れていないかを確かめてから、イメージを入れます。入れたあとは、電源を入れるたびに自動で起動します。

作り直したイメージに入れ替えるときも、1〜3 を同じようにやります。設定ファイル（`/etc/ros1-bridge/bridge.env`）は上書きしません。

## 設定

ふつうは書き換えなくて動きます。書き換えるときは、ロボットの PC で `/etc/ros1-bridge/bridge.env` を書き換えて、サービスを起動し直します。

```bash
sudo nano /etc/ros1-bridge/bridge.env
```

```bash
sudo systemctl restart ros1-bridge
```

| 項目 | 既定 | 意味 |
| --- | --- | --- |
| `ROS_MASTER_URI` | http://localhost:11311 | ROS 1 の roscore。同じ PC の roscore（ROS 1 の既定と同じ） |
| `ROS1_IP` | auto | ROS 1 のノードどうしがつなぐときの、この PC の IP。localhost なら 127.0.0.1 になる |
| `ROS1_HOSTS` | auto | ROS 1 のノードが名前（ホスト名）で名乗っていて、名前が引けないときは、roscore と同じ PC とみなす |
| `ODOM_TOPIC` / `CLOUD_TOPIC` / `CMD_VEL_TOPIC` | /odom / /cloud / /cmd_vel | 橋渡しするトピックの名前 |
| `BRIDGE_TF` | true | `/tf` と `/tf_static` も橋渡しするか |
| `VLM_COMMAND_TOPIC` | /vlm_drive/command | 元のシステムから VLM への開始と終了の合図（ROS 1 → ROS 2）。空なら橋渡ししない |
| `VLM_STATE_TOPIC` / `VLM_ACTIVE_TOPIC` | /vlm_drive/state / /vlm_drive/active | VLM が走らせているかの知らせ（ROS 2 → ROS 1）。空なら橋渡ししない |
| `ROS_DOMAIN_ID` | 42 | ROS 2 のドメイン。Orin（omnivla_real）とそろえる |
| `ROS2_PEERS` | なし | ROS 2 の相手が自動で見つからないときだけ、Orin の IP を書く。例: `ROS2_PEERS=192.168.0.200` |

## 動いているか確かめる

ロボットの PC でログを見ます。「roscore につながりました」と出れば、ROS 1 の側はつながっています。

```bash
journalctl -u ros1-bridge -f
```

Orin では、カメラのコンテナ（[jetson/camera](../jetson/camera/)）の中の `ros2` コマンドで見ます（Orin 本体は Ubuntu 24.04 なので、Humble の `ros2` コマンドがありません）。

```bash
docker exec -it usb-camera bash
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

Orin から速度の指令を流して、ロボットに届くか見ます。**ロボットが動くので、周りに人や物がないこと、すぐ止められることを確かめてから**にしてください。止めるときは Ctrl+C です。

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

ロボットの PC で、届いているか見ます。

```bash
rostopic echo /cmd_vel
```

## 外す

ロボットの PC で実行します。

```bash
bash ros1-bridge-bundle/uninstall.sh
```

## Docker がないとき

ロボットの PC に Docker がなければ、手元の PC で Docker のパッケージ（.deb）を落としてきて、運んで入れます。Docker の公式のページ（[Install from a package](https://docs.docker.com/engine/install/ubuntu/#install-from-a-package)）から、ロボットの PC の Ubuntu の版（`lsb_release -cs` で出る名前）と `amd64` を選び、`containerd.io`、`docker-ce`、`docker-ce-cli` の 3 つを落とします。ロボットの PC では、3 つを置いたフォルダで次を実行します。

```bash
sudo dpkg -i ./containerd.io_*.deb ./docker-ce-cli_*.deb ./docker-ce_*.deb
```

足りないものがあると言われたら、その名前のパッケージも同じように落としてきて入れます。

## うまくいかないとき

| ようす | 見るところ |
| --- | --- |
| 「roscore を待っています」のまま | ロボットの roscore が動いているか（ロボットの PC で `rostopic list`）。roscore が localhost 以外で動いているなら、`ROS_MASTER_URI` をロボットと同じにする |
| ロボットの PC のログは「つながりました」なのに、Orin に届かない | 両方の `ROS_DOMAIN_ID` が 42 か。ロボットの PC の防火壁（`sudo ufw status`）が ROS 2 の通信（UDP）を止めていないか。無線などで自動で見つからないなら、`ROS2_PEERS` に Orin の IP を書く |
| ROS 1 のノードにつながらない | ロボットのノードが、roscore とは別の PC で動いていないか（`ROS1_HOSTS` に名前と IP を足す。例: `ROS1_HOSTS=auto lidar-pc:192.168.0.101`） |
| `docker load` でエラーになる | 手元の PC とロボットの PC で、Docker の版が離れすぎていないか。ロボットの PC の Docker を新しくする |
| イメージを作るところで止まる | 手元の PC のメモリが足りないと止まることがある。ほかの重いものを止めてから作り直す |
| Orin の側で同じトピックが 2 つ出ている | 同じ名前のトピックを ROS 2 で直接出しているノードと、同時に動かさない。動かすなら、`ODOM_TOPIC` などで名前を変える（ROS 1 と ROS 2 で同じ名前になる） |

## 気をつけること

- 速度の指令をどう扱うか（指令が来なくなったら止まるか、など）は、ロボットの仕組みしだいです。Orin のノードが止まって指令が来なくなったとき、ロボットが止まるかを、先に確かめてください。
- 点群（`/cloud`）は大きいので、ROS 2 で PC の間を流すと、無線では遅れたり落ちたりしやすいです。ロボットの PC と Orin は、できれば有線でつないでください。遅れるときは、ロボットで点群を間引くか、送る回数を減らしてください。
