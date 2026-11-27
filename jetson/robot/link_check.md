# 疎通確認（Orin ↔ ロボットの PC）

ロボットの PC（ROS 1）と Orin（ROS 2 Humble）を、ROS 1 と ROS 2 の橋渡し（ros1_bridge）でつなぎ、要るトピックが通るかを確かめる手順です。

ロボットの PC に、すでに橋渡しが入っていれば（Foxy 用など）、それを使えます。このリポジトリの橋渡し（[ros1_bridge/](../../ros1_bridge/)）を新しく入れる必要はありません。

> [!CAUTION]
> **橋渡しは 1 つだけ動かします。** すでにある橋渡しを使うなら、このリポジトリの橋渡し（サービス `ros1-bridge`）は入れません（入れてあれば `sudo systemctl disable --now ros1-bridge`）。同じトピックを 2 つの橋渡しが通すと、指令が二重になったり、ぐるぐる回り続けたり（ループ）します。

| 順 | 場所 | すること |
| --- | --- | --- |
| 1 | — | 通すトピックを知る |
| 2 | ロボットの PC | すでにある橋渡しを調べる（種類、ドメイン、型） |
| 3 | Orin | ROS 2 のドメインを、橋渡しにそろえる |
| 4 | 両方 | ネットワークを確かめる |
| 5 | Orin | `check_link.sh` で確かめる |
| 6 | ロボットの PC | 合図・知らせ・速度の指令が届くかを確かめる |
| 7 | Orin | 結果を残す |

## 1. 通すトピック

橋渡しは、トピックの**名前を変えられません**。ROS 1 と ROS 2 で同じ名前になります。ロボットの名前が違うときは、Orin の側の設定を、ロボットの名前に合わせます（右の列）。

| トピック | 向き | 型 | 出すもの → 受けるもの | 要るか | 名前を変えるところ（Orin） |
| --- | --- | --- | --- | --- | --- |
| ⚠ `/cmd_vel` | Orin → ROS 1 | `geometry_msgs/Twist` | 安全ゲート → 台車のドライバ | 要る | `/etc/safety-gate/config.yaml` の `topics.cmd_out` |
| `/odom` | ROS 1 → Orin | `nav_msgs/Odometry` | ロボット → 実行系 | 要る | `/etc/elevator-task/robot.env` の `ODOM_TOPIC` |
| `/cloud` | ROS 1 → Orin | `sensor_msgs/PointCloud2` | LiDAR → 安全ゲート | 要る | `/etc/safety-gate/config.yaml` の `topics.cloud` |
| `/tf`、`/tf_static` | ROS 1 → Orin | `tf2_msgs/TFMessage` | ロボット → 安全ゲート | `base_link` ← 点群の座標系が取れればよい（来なければ安全ゲートの設定に書ける） | `/etc/safety-gate/config.yaml` の `frames`、`tf` |
| `/vlm_drive/command` | ROS 1 → Orin | `std_msgs/String` | ROS1システム → 実行系 | 合図で走らせるとき | `/etc/elevator-task/task.yaml` の `drive.command_topic` |
| `/vlm_drive/state` | Orin → ROS 1 | `std_msgs/String`（JSON） | 実行系 → ROS1システム | 同上 | `drive.state_topic` |
| `/vlm_drive/active` | Orin → ROS 1 | `std_msgs/Bool` | 実行系 → ROS1システム | 同上 | `drive.active_topic` |

カメラの画像は Orin の中で完結するので、橋渡しは通しません。

## 2. ロボットの PC: すでにある橋渡しを調べる

### 2-1. 動いているか、どの種類か

```bash
pgrep -af "dynamic_bridge|parameter_bridge"
```

| 出たもの | 種類 | 意味 |
| --- | --- | --- |
| `... dynamic_bridge` | dynamic_bridge | 片側に出すもの、反対側に受けるものがそろったトピックを、自動で橋渡しする |
| `... dynamic_bridge --bridge-all-topics` など | dynamic_bridge（全部） | 受けるものがいなくても、全部のトピックを橋渡しする |
| `... parameter_bridge` | parameter_bridge | 決めたトピックだけを橋渡しする（一覧は ROS 1 のパラメータ `topics`） |
| 何も出ない | — | 動いていない。起動のしかたをロボットの担当の人に聞くか、このリポジトリの [ros1_bridge/](../../ros1_bridge/) を入れる |

行の先頭の数（PID）を、次で使います。

parameter_bridge なら、1 のトピックが一覧に入っているかを見ます（入っていなければ、足して橋渡しを起動し直す）。

```bash
rosparam get /topics
```

### 2-2. ドメインなど（橋渡しの環境）

`<PID>` を 2-1 の数に置き換えます。

```bash
sudo cat /proc/<PID>/environ | tr '\0' '\n' | grep -E "ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|ROS_DISTRO|ROS_MASTER_URI"
```

| 項目 | 意味 | よい値 |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | ROS 2 のドメイン。出なければ 0 | Orin と同じ番号（3 でそろえる） |
| `ROS_LOCALHOST_ONLY` | 1 なら、同じ PC の中としか話さない | 出ないか 0。1 なら Orin と話せないので、橋渡しの起動のしかたを変える |
| `RMW_IMPLEMENTATION` | ROS 2 の通信のしくみ。出なければ `rmw_fastrtps_cpp` | 出ないか `rmw_fastrtps_cpp`（Orin と同じ）。`rmw_cyclonedds_cpp` でも、たいていは通じる |
| `ROS_DISTRO` | 橋渡しの ROS 2 の版 | `foxy` でもよい（下の注） |
| `ROS_MASTER_URI` | ROS 1 の roscore | ロボットの roscore |

> [!NOTE]
> Foxy の橋渡しと Orin（Humble）は ROS 2 の版が違います。版の違う ROS 2 どうしの通信は公式には保証されていませんが、ここで使う標準のメッセージ（1 の表の型）なら通じることが多いです。通るかどうかは 5 と 6 で実際に確かめます。

### 2-3. 型に対応しているか

橋渡しは、作ったときに入っていた型しか通せません。橋渡しを起動しているのと同じ環境（ROS 1 と ROS 2 と、橋渡しの setup.bash を読み込んだ端末）で:

```bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -E "Twist|Odometry|PointCloud2|TFMessage|std_msgs/(msg/)?(String|Bool)"
```

`geometry_msgs/msg/Twist`、`nav_msgs/msg/Odometry`、`sensor_msgs/msg/PointCloud2`、`tf2_msgs/msg/TFMessage`、`std_msgs/msg/String`、`std_msgs/msg/Bool` の 6 つが出れば足ります（標準の型なので、ふつうは全部あります）。

### 2-4. dynamic_bridge の注意

dynamic_bridge（`--bridge-all-topics` などなし）は、**片側に出すもの、反対側に受けるものがそろったときだけ**、そのトピックを橋渡しします。

- Orin の実行系を起動する前は、`/odom` は橋渡しされません（Orin に受けるものがいない）。`check_link.sh` は自分で受けるので、起動の前でも確かめられます。
- ロボットの PC で `rostopic echo /vlm_drive/state` を始めると、そのときから `/vlm_drive/state` の橋渡しが始まります（知らせは 1 秒に 2 回出しているので、すぐ届きます）。
- 合図（`/vlm_drive/command`）は、ROS 1 の側に出すものができてから橋渡しが始まるので、`rostopic pub -1` の 1 回目を落とすことがあります。合図は [ros1_bridge/tools/vlm_drive.py](../../ros1_bridge/tools/vlm_drive.py) で出します（橋渡しの準備ができるのを待ってから出し、答えを表示する）。ROS1システムのノードから出すときは、出す側（Publisher）を起動のときに作っておけば、この心配はありません。
- `/tf_static` は、橋渡しが「とっておいて、後から来た相手にも渡す」設定にします（ros1_bridge 0.9.4 から）。

## 3. Orin: ROS 2 のドメインをそろえる

Orin の 3 か所（カメラ、安全ゲート、実行系）を、2-2 で見た橋渡しの番号にまとめて変えます（例: 番号が出なかった＝ 0 のとき）。

```bash
bash jetson/robot/set_domain.sh 0
```

今の番号を見るだけなら、番号を付けずに実行します。

```bash
bash jetson/robot/set_domain.sh
```

> [!NOTE]
> 0 は、何も決めていない ROS 2 がみな使う番号です。同じネットワークに別の ROS 2 の機械があると、互いのトピックが混ざります。ロボットの中だけのネットワークなら気にしなくてよいです。橋渡しの方を 42 に変えてもよいですが、そのときは橋渡しの起動のしかた（サービスや起動スクリプトの `ROS_DOMAIN_ID`）を、ロボットの担当の人と相談して変えます。

## 4. ネットワークを確かめる

Orin とロボットの PC を、同じネットワークにつなぎます（有線が確実です）。Orin から:

```bash
ping -c 3 <ロボットの PC の IP>
```

ROS 2 は、相手をマルチキャストで自動で見つけます。マルチキャストが通るかを確かめます。ロボットの PC で（ROS 2 の setup.bash を読み込んだ端末で）:

```bash
ros2 multicast receive
```

Orin で:

```bash
bash jetson/robot/ros2.sh multicast send
```

ロボットの PC に `Received from ...` と出れば通ります。

| 通らないとき | 見るところ |
| --- | --- |
| ping も通らない | ケーブル、IP アドレス（同じネットワークか） |
| ping は通るが、マルチキャストが通らない | ロボットの PC の防火壁（`sudo ufw status`。動いていれば `sudo ufw allow from <OrinのIP>`）。無線やスイッチがマルチキャストを止めていないか（有線にする） |

## 5. Orin: check_link.sh で確かめる

ロボットの PC で、roscore、ロボット（オドメトリ、LiDAR、TF）、橋渡しを動かしておきます。ロボットは止めておきます。

```bash
cd ~/physical_ai_for_mobile_robot
```

```bash
bash jetson/robot/check_link.sh
```

15 秒ほど受けて、項目ごとに `[OK]`、`[NG]`、`[注意]` を出します。`[NG]` の下の `→` が直し方です。

| 項目 | OK のとき | NG のときの主な原因 |
| --- | --- | --- |
| ROS 2 のドメイン | 3 か所が同じ | 3 の `set_domain.sh` |
| カメラ画像 | 30 Hz 前後 | カメラ（README の手順 2） |
| オドメトリ | 数十 Hz、出しているもの `ros_bridge` など | 名前（`ODOM_TOPIC`）、ドメイン、ネットワーク、橋渡し |
| 点群 | 数 Hz 以上、点の数 | 名前（`topics.cloud`）、橋渡し |
| TF base_link ← 点群の座標系 | 位置と向きが出る（取り付けの値と合っているか見る） | ロボットの PC に TF があるか（`rosrun tf tf_echo base_link <点群の座標系>`）。どこにもなければ `/etc/safety-gate/config.yaml` の `tf:` に書く |
| 安全ゲートから見た点群 | 障害物のますの数、足形の大きさ | 点群か TF |
| ⚠ 最終の速度指令 | 受ける: `ros_bridge` など | 台車がその名前を受けていない、橋渡し |
| 知らせ・合図 | 実行系を起動した後に出る（起動の前は `[--]`） | 実行系のログ |
| 橋渡し | `ros_bridge` など 1 つ | 見えない: ドメイン・ネットワーク。2 つ以上: 橋渡しが 2 つ動いている |

`[NG]` がなくなるまで直します。実行系（`bash jetson/robot/robot_up.sh`）を起動した後に、もう一度実行して、知らせと合図も `[OK]` になるのを確かめます。

## 6. ロボットの PC: 届くかを確かめる

Orin で実行系を起動してから行います。

### 6-1. 知らせ（Orin → ROS 1）

JSON が 1 つ出れば届いています。

```bash
rostopic echo -n 1 /vlm_drive/state
```

### 6-2. 合図（ROS 1 → Orin）

[ros1_bridge/tools/vlm_drive.py](../../ros1_bridge/tools/vlm_drive.py) を USB メモリなどでロボットの PC に置き、置いたフォルダで実行します（ROS 1 の setup.bash を読み込んだ端末で）。`ping` は何もしない合図です。

```bash
python3 vlm_drive.py ping
```

`OK: 届きました（何もしません）` と出れば、両方向がつながっています。実行系の画面（`http://<OrinのIP>:8080/`）の「外からの合図」にも出ます。

### 6-3. 速度の指令（Orin → ROS 1 → 台車）

ロボットは止めておきます（非常停止を押しておくと確実です）。ROS1システムのナビゲーションも止めておきます（`/cmd_vel` を出さない状態）。ロボットの PC で:

```bash
rostopic echo /cmd_vel
```

Orin で、安全ゲートに速度 0 の指令を 3 秒流します。

```bash
bash jetson/robot/check_link.sh --zero
```

ロボットの PC に、`linear` も `angular` も 0 の指令が 3 秒ほど出れば、速度の通り道（実行系の出口 → 安全ゲート → 橋渡し → ロボットの PC）はつながっています。最後に、台車のドライバが受けているかを見ます。

```bash
rostopic info /cmd_vel
```

`Publishers` に橋渡し（`/ros_bridge` など）、`Subscribers` に台車のドライバが出れば OK です。台車のドライバが別の名前を受けているときは、その名前を `/etc/safety-gate/config.yaml` の `topics.cmd_out` に書きます（README の ⚠）。

## 7. 結果を残す

```bash
bash jetson/robot/check_link.sh --save
```

`jetson/log/link_check_<日時>.txt` に残ります。コミットして共有します。

## うまくいかないとき

| ようす | 見るところ |
| --- | --- |
| Orin に何も届かない（オドメトリも点群も NG、橋渡しも見えない） | ドメイン（2-2、3）、`ROS_LOCALHOST_ONLY`（2-2）、ネットワーク（4） |
| ROS 1 では出ているのに、点群だけ届かない | 名前（`topics.cloud`）。点群が大きく、無線で遅れていないか（有線にする） |
| TF が取れない | ロボットの PC に TF があるか。`/tf_static` だけが届かないなら、安全ゲートの `tf.publish_static: true` と取り付けの値 |
| ⚠ 最終の速度指令を受けるものがいない | 台車のドライバが受ける名前。dynamic_bridge は、ROS 1 の側に受けるものがいるときだけ橋渡しする。parameter_bridge なら `topics` の一覧に入っているか |
| 指令が二重に見える、止まらない | 橋渡しが 2 つ動いていないか（2-1。Orin の `check_link.sh` の「橋渡し」） |
| `vlm_drive.py ping` が「受け付けられませんでした」「知らない合図です」 | Orin の実行系が古い。ネットのあるところで `ROBOT_REBUILD=1 bash jetson/robot/robot_up.sh` |
| `vlm_drive.py ping` が「受けるものがいません」 | 橋渡しが `/vlm_drive/command` を通していない（parameter_bridge の一覧、dynamic_bridge なら Orin の実行系が動いているか） |
| 「時計がずれています」 | 実行系は直して使うので動きます。両方の時計を合わせるのが確実（chrony など） |
