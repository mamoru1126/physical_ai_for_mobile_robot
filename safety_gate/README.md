# 安全ゲート（Docker のコンテナ、自動で起動、WebUI つき）

VLM が出す速度の指令を、点群を見て安全なものにしてから、ロボットに流します。進む先のすぐ近くに障害物があれば止め、少し先にあれば速度を落とし、なければそのまま通します。デバッグ用に、ロボットのまわり 10 m 四方を上から見た画面を、ブラウザで見られます。

[samples/safety_gate](../samples/safety_gate/) をもとに作りました。もとのものから直したところは「[もとのサンプルから直したところ](#もとのサンプルから直したところ)」にあります。

| 項目 | 状態 |
| --- | --- |
| ロジック | 単体テスト 23 項目を通した（[document/単体テスト仕様書.md](document/単体テスト仕様書.md)） |
| コンテナ | GitHub Actions でイメージを作り、にせの点群と指令で通しの確認をした（[.github/workflows/safety-gate.yml](../.github/workflows/safety-gate.yml)） |
| 実機 | まだ通していない |

## つながり

| 向き | トピック（既定） | 型 | 中身 |
| --- | --- | --- | --- |
| 受ける | `/vlm_cmd_vel` | `geometry_msgs/Twist` | VLM が出す速度の指令 |
| 受ける | `/cloud` | `sensor_msgs/PointCloud2` | 点群（`cloud_link` 座標系）。TF で `base_link` に直して使う |
| 出す | `/cmd_vel` | `geometry_msgs/Twist` | 安全を考えた速度の指令（20 Hz） |
| 出す | `/safety_gate/state` | `std_msgs/String` | 判定の結果（`PASS`、`SLOW`、`STOP`、`IDLE` と理由） |
| 出す | `/safety_gate/debug` | `std_msgs/String` | WebUI 用の情報 |
| WebUI | ポート 9001 | HTTP | 上から見た 2D の画面 |

トピックの名前、ロボットの足形、余裕などは、すべて設定ファイル（YAML）で、コンテナの外から変えられます。

### TF（`cloud_link` → `base_link`）

ロボット（ROS 1）が `/tf` か `/tf_static` で `base_link` と `cloud_link` の関係を流していれば、[ROS 1 との橋渡し](../ros1_bridge/) が ROS 2 に流すので、そのまま使えます（`BRIDGE_TF=true`、既定）。確かめること:

- ロボットの TF の座標系の名前が、設定の `frames.base`（既定 `base_link`）と、点群のヘッダの座標系に合っているか。違えば `frames.base`、`frames.cloud` を書き換える。ロボットの PC で `rosrun tf tf_echo base_link cloud_link` を実行すると、関係が出るかでわかる。
- `base_link` と `cloud_link` が、TF の木でつながっているか（間に別の座標系があってもよい）。
- このときは `tf.publish_static` を false のままにする（同じ関係を 2 か所から出さない）。

ロボットが TF を流していないときだけ、`tf.publish_static: true` にして、`xyz`・`rpy` に取り付けの位置と向きを書きます。

## 判定のしかた

1. 点群を TF で `base_link` に直します（`cloud_link` から）。
2. 床と天井を除き（高さ `height_range`）、5 cm のボクセルにまとめます。1 つ 2 つの点はノイズとみなします（`obstacle_voxel_threshold` 個以上で障害物）。
3. 指令のまま 2 秒先（`time_horizon`）まで動いたときの、足形の動きを予測します。まっすぐ、横、回りながら（円弧）のどれでも予測します。
4. 足形のまわりの余裕（`margin`）の中に、**近づいて入ってくる**障害物を探します。近づく障害物がある場合は、ぶつかるまでの時間で決めます。

| ぶつかるまでの時間 | 出す指令 |
| --- | --- |
| 0.5 秒（`stop_time`）以内 | 止める（`STOP`） |
| 0.5〜2 秒 | 時間に合わせて速度を落とす（`SLOW`。2 秒なら指令のまま、0.5 秒なら 0） |
| 2 秒より先、またはぶつからない | そのまま通す（`PASS`） |

- **離れる動きは止めません**。前に障害物があっても、下がるのは通します。その場で回るときは、足形の角が障害物に近づいていく向きだけを止めます。
- **すでに余裕より近い壁と、平行に進むのは止めません**（エレベーターの入口など）。その壁にさらに近づくのは止めます。
- 足形の中の点は、ロボット自身が映ったものとみなして使いません。
- 次のときは止めます（0 を出します）。
  - 指令が 0.5 秒（`cmd_timeout`）来ない（`IDLE`）
  - 点群が 0.5 秒（`cloud_timeout`）来ない（`STOP`）
  - TF がない（`STOP`）
- 指令は上限（`limits`）で抑えます。

## 入れる

### ネットにつながる PC（Orin など）

このリポジトリのフォルダで実行します。初回はイメージを作るので、数分〜十数分かかります。

```bash
bash safety_gate/install.sh
```

### ネットにつながらない PC（ロボットの PC など）

[ROS 1 との橋渡し](../ros1_bridge/) と同じように、手元の x86 の PC でイメージを作って運びます。

```bash
bash safety_gate/build_image.sh
```

できた `safety_gate/dist/safety-gate-bundle.tar` を運び、ロボットの PC で実行します。

```bash
tar -xf safety-gate-bundle.tar
```

```bash
bash safety-gate-bundle/install.sh
```

### どちらで動かすか

| 置き場所 | よいところ | 気をつけること |
| --- | --- | --- |
| ロボットの PC | 点群が PC の間を流れない。Orin やネットワークが止まっても、指令が途切れれば止まる | イメージを運ぶ手間がある |
| Orin | 入れるのが楽 | 点群が PC の間を流れる（無線だと遅れやすい） |

安全のためには、ロボットの PC をすすめます。どちらで動かしても、出した `/cmd_vel` は [ROS 1 との橋渡し](../ros1_bridge/) がロボットに流します。同じ `/cmd_vel` を出すほかのノードとは、同時に動かさないでください。

## 設定を変える

設定は `/etc/safety-gate/config.yaml` にあります（入れるときに [config.yaml](ros2_ws/src/safety_gate/config/config.yaml) をコピーしたもの）。コンテナからは `/config/config.yaml` に見えます。書き換えたら、サービスを起動し直します。

```bash
sudo nano /etc/safety-gate/config.yaml
```

```bash
sudo systemctl restart safety-gate
```

| 項目 | 既定 | 意味 |
| --- | --- | --- |
| `topics.cmd_in` / `cloud` / `cmd_out` | /vlm_cmd_vel / /cloud / /cmd_vel | 受ける指令、点群、出す指令のトピック |
| `frames.base` | base_link | 判定に使う座標系 |
| `frames.cloud` | 空 | 空なら点群のヘッダの座標系（`cloud_link`）。先頭の `/` は外す |
| `tf.publish_static` | false | TF がどこからも来ないときだけ true にして、`xyz`・`rpy` に `base_link` から見た `cloud_link` の位置と向きを書く |
| `footprint` | 0.7 m × 0.6 m の長方形 | ロボットの足形（上から見た形）。`base_link` の座標で頂点を順に書く |
| `height_range` | 0.05〜1.5 m | 障害物として見る高さ（床と天井を除く） |
| `safety.margin` | 0.10 m | 足形のまわりの余裕 |
| `safety.time_horizon` / `stop_time` | 2.0 s / 0.5 s | 何秒先まで見るか / これより早くぶつかるなら止める |
| `safety.obstacle_voxel_threshold` | 3 | 障害物とみなすボクセルの数 |
| `safety.cmd_timeout` / `cloud_timeout` | 0.5 s / 0.5 s | 指令・点群がこれより長く来なければ止める |
| `limits` | 0.5 m/s、0.5 m/s、1.0 rad/s | 速度の上限 |
| `web.port` | 9001 | WebUI のポート |
| `web.rate` | 5 Hz | ブラウザに送る回数 |
| `ros_domain_id` | 42 | ROS 2 のドメイン |

## WebUI

同じネットワークの PC のブラウザで開きます。

```text
http://<安全ゲートが動いている PC の IP>:9001/
```

- ロボットを真ん中に、上を前にして、まわり 10 m 四方を上から見た画面です（1 マス 1 m）。
- 障害物（灰色）、近づく先の障害物（赤）、足形（白）、余裕（黄色の帯）、予測した足形（水色）を描きます。
- VLM の指令（青の矢印）と、出した指令（緑の矢印）も描きます。矢印は 1 秒で進む分、弧は 1 秒で回る分です。
- 判定の結果（`PASS`、`SLOW`、`STOP`、`IDLE`）と理由、ぶつかるまでの時間、入力の古さを出します。

### 軽くするための工夫

- **点を送らない**：点群は、表示のマス（5 cm）にまとめ、重なりを除いてから送ります。
- **マスの番号を 2 バイトで送る**：10 m 四方は 200 × 200 マスなので、マスの番号は 2 バイトの整数に収まります。座標の小数は送りません。番号は小さい順に並べて base64 にします。
- **数値は小数 2 桁まで**：速度、足形、予測した姿勢などは小数 2 桁に丸め、末尾の 0 も落とします（0.50 → 0.5）。
- **回数を絞る**：判定は 20 Hz ですが、ブラウザには 5 Hz（`web.rate`）だけ送ります。送り方は Server-Sent Events で、新しい情報があるときだけ流します。
- **ブラウザでも軽く**：画面の右に、1 回に受け取った量（KB）を出します。ふつうの部屋なら数 KB の見込みです。

## 動いているか確かめる

ログを見ます。

```bash
journalctl -u safety-gate -f
```

判定の結果を見ます（コンテナの中で）。

```bash
docker exec -it safety-gate bash
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
ros2 topic echo /safety_gate/state
```

## もとのサンプルから直したところ

| もとのサンプル | 困ること | 直したこと |
| --- | --- | --- |
| 見る範囲（ROI）が、設定ファイルでは x = -1.0〜-0.4（後ろだけ）。コードの既定は -1.0〜0.4 で、食い違っている | 前に進むときに、前の障害物を見ない | 範囲ではなく、ロボットの足形と余裕で見る。前も後ろも横も見る |
| 範囲に障害物があれば、どんな指令でも止める | 障害物から離れる（下がる、回る）こともできず、動けなくなる | 指令のまま動いたときに近づく障害物だけで決める。離れる動きは通す |
| 点群を LiDAR の座標系のまま使う | LiDAR の付け方で、範囲の意味が変わる | TF で `base_link` に直してから使う。ROS 1 から来る `/cloud_link` の先頭の `/` も外す |
| 最後に受けた指令と点群を、いつまでも使う | VLM や LiDAR が止まっても、最後の指令で動き続ける | 指令・点群が 0.5 秒来なければ止める |
| 止めるか通すかの 2 つだけ | 障害物の手前で急に止まる | 少し先の障害物では、速度を落とす |
| 50 Hz ごとに、同じ点群を毎回変換する | 計算の無駄 | 点群が来たときに 1 回だけ変換して、ボクセルにしておく |
| 速度の指令の受け手が reliable | 送り手が best effort だと受けられない | best effort で受ける（どちらの送り手からも受けられる） |
| 速度の上限がない | VLM が大きな値を出すと、そのまま流れる | 上限で抑える。数値でない指令は 0 にする |
| PCL を使う | イメージが大きく、作るのに時間がかかる | ボクセルにまとめるのも自前で書いた（PCL は要らない） |

- `test/PathTrackingController_unit_test.cpp` は、ほかのもの（経路追従）の試験で、安全ゲートとは関係がないので持ってきていません。
- 設計書と単体テスト仕様書は、直したロジックに合わせて書き直しました（[document/](document/)）。

## 外す

```bash
bash safety_gate/uninstall.sh
```
