# physical_ai_for_mobile_robot

移動ロボット向けフィジカル AI（画像と言語の指示によるタスク実行）の検討資料を置くリポジトリ。

## 検討資料

| 日付 | 資料 | ファイル |
| --- | --- | --- |
| 2026-11-10 | 車輪型移動ロボットで VLM を使った研究例 | [docs/vlm-wheeled-survey/index.html](docs/vlm-wheeled-survey/index.html) |
| 2026-11-10 | 画像と言語の指示で、移動ロボットをエレベーターに乗り降りさせる（ソフトウェア構成の検討） | [docs/elevator-task/index.html](docs/elevator-task/index.html) |

資料は静的な HTML で、ビルドは要らない。

## GitHub Pages での公開

公開元のフォルダは `/ (root)` と `/docs` のどちらでも表示できる。

| 公開元 | 一覧のページ | エレベーターの資料 |
| --- | --- | --- |
| `/ (root)` | `https://mamoru1126.github.io/physical_ai_for_mobile_robot/` | `https://mamoru1126.github.io/physical_ai_for_mobile_robot/docs/elevator-task/` |
| `/docs` | `https://mamoru1126.github.io/physical_ai_for_mobile_robot/` | `https://mamoru1126.github.io/physical_ai_for_mobile_robot/elevator-task/` |

## 構成

```text
index.html                    docs/ へ転送するだけのページ
docs/index.html               検討資料の一覧
docs/elevator-task/index.html エレベーター乗り降りの検討資料
docs/vlm-wheeled-survey/index.html 車輪型移動ロボットの VLM 研究例
```
