# USB カメラの映像を ROS 2 のトピックに流す（自動で起動）

Orin（または Linux の PC）につないだ USB カメラの映像を、ROS 2 のトピックに流します。同じネットワークの PC から、ブラウザでも見られます（ポート 9000）。Docker のコンテナとして動き、電源を入れるたびに自動で起動します。

| 項目 | 状態 |
| --- | --- |
| スクリプト | 用意した。Orin ではまだ通していない（サービスの設定ファイルが正しく作られることは確かめた） |

## 中身

| もの | 中身 |
| --- | --- |
| カメラのノード | [usb_cam](https://index.ros.org/p/usb_cam)（ROS 2 Humble） |
| ブラウザ向けの配信 | [web_video_server](https://index.ros.org/p/web_video_server/)（ポート 9000） |
| トピック | `/usb_cam/image_raw`、`/usb_cam/image_raw/compressed`、`/usb_cam/camera_info` |
| 自動の起動 | systemd のサービス `usb-camera` が、Docker のコンテナ `usb-camera` を起動する。止まったら 5 秒後に起動し直す |

- カメラがつながっていなければ、つながるまで待ちます。抜けたら、挿し直したときに起動し直します。
- トピックの名前空間は `usb_cam` にしてあります。ロボットのカメラ（`/camera/...`）とぶつかりません。
- GPU は使いません。JetPack 6 でも 7.2 でも、x86 の PC でも動く見込みです。

## 入れる

このリポジトリのフォルダで実行します。初回はイメージを作るので、数分かかります（ネットが要ります）。

```bash
bash jetson/camera/install.sh
```

終わると、ブラウザで開く URL が出ます。同じネットワークの PC で開くと、トピックの一覧が出ます。`/usb_cam/image_raw` を押すと映像が出ます。

```text
http://<OrinのIP>:9000/
```

映像だけを直接開くこともできます。

```text
http://<OrinのIP>:9000/stream?topic=/usb_cam/image_raw
```

## 設定を変える

設定は `/etc/usb-camera/camera.env` にあります（入れるときに `jetson/camera/camera.env` をコピーしたもの）。書き換えたら、サービスを起動し直します。

```bash
sudo nano /etc/usb-camera/camera.env
```

```bash
sudo systemctl restart usb-camera
```

| 項目 | 既定 | 意味 |
| --- | --- | --- |
| `CAMERA_DEVICE` | auto | カメラのデバイス。auto なら USB のカメラを探す。`/dev/video0` のようにも指定できる |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | 640 / 480 | 画像の大きさ |
| `CAMERA_FPS` | 30.0 | 1 秒あたりの枚数 |
| `CAMERA_PIXEL_FORMAT` | mjpeg2rgb | カメラから受け取る形式。映らなければ yuyv2rgb を試す |
| `CAMERA_NAMESPACE` | usb_cam | トピックの名前空間 |
| `CAMERA_INFO_URL` | 空 | カメラの校正ファイル（例 `file:///etc/usb-camera/camera_info.yaml`） |
| `CAMERA_WEB_PORT` | 9000 | ブラウザで見るためのポート |
| `ROS_DOMAIN_ID` | 0 | ROS 2 のドメイン。ほかの PC やロボットとトピックをやりとりするなら、相手とそろえる |

画像の大きさと枚数は、カメラが対応している組み合わせにします。対応している組み合わせは、ログの最初に出ます。

## 様子を見る

サービスの状態とログです。

```bash
systemctl status usb-camera
```

```bash
journalctl -u usb-camera -f
```

ROS 2 の入った PC（同じ `ROS_DOMAIN_ID`）から、トピックが届いているかを見ます。

```bash
ros2 topic hz /usb_cam/image_raw
```

## 外す

自動で起動しないようにして、サービスを止めます。設定ファイルとイメージは残ります。

```bash
bash jetson/camera/uninstall.sh
```

## 困ったとき

| 症状 | 対処 |
| --- | --- |
| ログに「カメラがつながるのを待っています」が出続ける | `ls /dev/video*` でカメラが見えているか確かめる。見えていれば `CAMERA_DEVICE` にその名前を書く |
| 映像が出ない、形式のエラーが出る | `CAMERA_PIXEL_FORMAT=yuyv2rgb` にする。大きさと枚数を、ログの一覧にある組み合わせにする |
| ブラウザで開けない | Orin の IP が合っているか、ポート 9000 をほかのプログラムが使っていないか、ファイアウォールで止めていないかを確かめる |
| ほかの PC でトピックが見えない | `ROS_DOMAIN_ID` を相手とそろえる。ネットワークを限っている場合は、DDS の設定もそろえる |
