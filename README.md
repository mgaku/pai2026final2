# AI信号認識によるロボットの停止・再発進・ゴール到達

Google Colab内でROS 2 HumbleとPyBulletを連携させるPhysical AI課題作品。

**検証状態：受講者のGoogle Colab実行結果JSONで、ROS 2 Humble使用、画像・制御・速度指令の各600件処理、49.9秒でのゴール到達を確認しました。受講者による動画の動作確認も報告されています。**

## 実行結果（1回のシミュレーション）

| 指標 | 記録値 |
|---|---|
| ROS2 | Humble |
| シミュレーション時間 | 60秒（動画は等速） |
| 画像処理 / 制御処理 / 速度指令受信 | 各600件 |
| ゴール到達 | true |
| 到着時刻 | 49.9秒 |
| 最終x座標 | 4.0081437901m（目標4m） |
| x方向最終位置誤差 | 約8.14mm |
| 赤信号区間22〜28秒のx方向変位 | 約0.220mm |

根拠は同梱の `results.json`。動画確認は受講者報告に基づきます。本結果は1回分であり、成功率や実機の位置決め精度を示すものではありません。今回の実行コード・環境との完全一致は、実行済みColabノートブックと環境記録を併せて保存して確認してください。

## 実行方法（WindowsにDockerを入れる必要はありません）

1. Google Colab https://colab.research.google.com/ を開く。
2. 「ファイル → ノートブックをアップロード」で `PAI_Colab_ROS2_Humble.ipynb` を選ぶ。
3. CPUのLinuxランタイムで上から実行。
4. 環境導入後、通信テストで `ROS2_HUMBLE_OK` を確認。失敗したら走行へ進まない。
5. 走行セルを実行し、動画と `results.json` を確認。
6. 最後のセルから `PAI_ros2_colab_results.zip` をダウンロード。

HumbleはRoboStack配布の専用conda互換環境に導入します。Colab標準Pythonへ直接apt版Humbleを入れる方式ではありません。導入には複数GBのディスク領域とネットワーク接続が必要です。ランタイム削除後は再構築してください。外部配布パッケージの更新・依存関係により導入できない可能性があります。

## ROS2の構成

3つのrclpyノードを**同一プロセスの単一Executor**で動かします。別プロセス間通信やGazeboとのブリッジを実証する構成ではありません。

| ノード | 処理 |
|---|---|
| pybullet_simulator | 画像・位置の配信、速度指令の受信、車輪駆動、動画生成 |
| ai_signal_recognizer | RGB画像から信号領域を抽出し、学習済みsoftmax回帰で赤・緑を分類 |
| signal_controller | AI判定と位置を受信し、停止・前進・ゴール停止を判断 |

| トピック | 型 |
|---|---|
| /camera/image | sensor_msgs/Image |
| /odom | nav_msgs/Odometry |
| /ai/observation | std_msgs/String（JSON） |
| /cmd_vel | geometry_msgs/Twist |
| /task/status | std_msgs/String（JSON） |

シミュレータからAI関数へ直接判断を依頼せず、画像配信→判定配信→速度指令受信をrclpyのpub/subで行います。各フレームの指令受信後に物理時間を進める同期シミュレーションです。メッセージヘッダのstamp.secはフレーム番号として使用しており、実時間やROS /clockを表すものではありません。

## タスクとAIの役割

- 赤信号で待機、緑で前進、走行途中の赤で停止、緑で再発進。
- 信号と別にx=4m地点へ白いゴール枠・GOAL表示を設置。
- 60秒・等速動画。速度指令0.105m/sとし、今回の実行ログでは49.9秒で到着。
- 信号は環境シナリオで0〜8秒と22〜28秒に赤となる。制御器は切替時刻の正解を受け取らず、カメラ画像の分類を使う。
- AIは合成RGB 1,600件で学習するsoftmax分類器。彩度による領域抽出、4回の連続確認、行動の選択はルール処理。
- 到着判定は/odomのx座標。シミュレータの真値位置をオドメトリの代用として配信している。AIによるゴールの文字認識ではない。

## ファイル

| ファイル | 内容 |
|---|---|
| PAI_Colab_ROS2_Humble.ipynb | 環境構築・テスト・実行・結果保存の全手順 |
| bootstrap_ros2.sh | Humbleと依存ライブラリを導入 |
| ros2_smoke.py | 2ノードの文字列送受信テスト |
| ros2_main.py | 学習、3ノード、物理シミュレーション、動画・ログ出力 |
| VALIDATION.md | 現時点の検証範囲と実行後の確認項目 |

ノートブックにはPython・シェルコードを埋め込んでいます。コードを修正した場合は、対応するノートブックのwritefileセルも更新してください。

## 出力と評価

実行に成功すると `pai_ros2_colab_output` に動画、学習重み、trajectory.json、results.json、レポート原稿を出力します。

1. `ros_distro` が `humble`、`ros2` が `true` であること。
2. `image_messages_processed`、`controller_messages_processed`、`cmd_vel_received` が各600件であること。
3. 動画で赤待機・緑前進・途中停止・再発進・ゴール停止を確認すること。
4. `events` の到着時刻と `goal_position_error_m` を記録すること。
5. `red_interval_displacement_m` は22秒から28秒までの変位。速度指令がゼロでも慣性や接地で微小変位があり得る。

今回確認したROS2版の `results.json` を同梱しています。最新実行の動画・trajectory.json・学習重み・環境記録は本一式には含めていません。必要なものは同じ実行で生成したファイルを追加してください。旧PyBullet版の結果と混在させないでください。

## 限界

多色の背景や複数信号、遮蔽・実カメラ画像での汎化は未検証。荷扱い・障害物回避・左折右折は対象外。モデルの付属URDFに慣性未定義のリンクがあり、PyBulletの既定値に依存するため実機性能を再現するものではありません。通信待ち5秒超過ではゼロ指令を設定して中止しますが、実機の安全機能ではありません。

## 参考と独自実装

- N622氏「gazeboとROS2を連携してロボットを操作する」 https://qiita.com/N622/items/9d89f77d85d9da0af29e — 当初の差動二輪・ROS2速度指令構成の参考。本版はPyBulletを用い、記事のサンプルコードやCADをコピーしていません。
- https://robostack.github.io/micromamba.html — Humble専用環境の導入方法。
- https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html — 環境管理ツールの導入。
- https://github.com/bulletphysics/bullet3 — 物理シミュレータと付属Huskyモデル。依存物のライセンスは配布元参照。

独自実装は信号画像の学習分類、連続確認、3ノードの連携、信号とゴールの分離、停止・再発進シナリオ、ログと動画の同時出力。生成AIによるコード作成支援を使用しており、提出者本人が理解・実行確認・独自の工夫と考察を加える前提です。


