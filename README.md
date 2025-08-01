# AI強化自律配送ロボット - Physical AI Final Project 2025

## 🤖 プロジェクト概要

このプロジェクトは、AI技術を活用した自律配送ロボットのシミュレーションです。TurtleBot3を使用し、Gazebo環境で複数の荷物を効率的に配送するタスクを自律的に実行します。

**完成する成果物**: Gazebo 3Dシミュレーター上で、移動ロボット（TurtleBot3）が「荷物の集荷場所への移動 → 停止 → 配送先への移動 → 停止」という一連の運搬タスクを自律的に実行するプログラム。

## 🔧 開発環境

- **OS**: Ubuntu 22.04.5 LTS (Docker Container)
- **Framework**: ROS2 Humble
- **Simulator**: Gazebo
- **Robot**: TurtleBot3 Waffle Pi
- **Programming**: Python 3.10
- **AI Library**: NumPy (推論処理)
- **Sensors**: LiDAR, Odometry
- **Development Tools**: 
  - Visual Studio Code (Dev Containers)
  - Docker Desktop
  - colcon (ROS2 build system)

## 🧠 AI機能

### 1. リアルタイム障害物検知・回避
- **LiDARセンサー活用**: 360度の環境認識
- **AI推論による判断**: 前方、左右の障害物を検知し、最適な回避経路を計算
- **緊急回避システム**: 近距離障害物に対する即座の反応
- **ゾーン別検知**: 前方60度、左右120度の範囲で段階的な障害物検知

### 2. インテリジェント経路最適化
- **動的目標選択**: センサー情報に基づく次の配送先のAI判断
- **適応速度制御**: 環境に応じた速度調整（障害物距離に基づく動的制御）
- **位置追跡**: オドメトリによる正確な位置把握と移動距離計算

### 3. 自律配送タスク管理
- **複数荷物対応**: Package A, B, C の順次配送
- **状態管理**: AI強化ステートマシンによる効率的タスク実行
- **完了確認**: 各配送の成功確認とログ出力

## プロジェクト構造

```
src/final_project_pkg/
├── package.xml                    # ROS2パッケージ設定
├── setup.py                      # Python setup設定
├── setup.cfg                     # Python setup設定
├── README.md                     # プロジェクト詳細説明
├── resource/
│   └── final_project_pkg         # リソースファイル
├── launch/
│   └── final_project_autonomous.launch.py  # メインランチファイル
├── final_project_pkg/
│   ├── __init__.py              # パッケージ初期化
│   └── task_executor.py         # AI自律制御メインプログラム
└── test/                        # テストファイル群
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

## 🚀 セットアップと実行方法

### 前提条件
- Docker Desktop がインストールされていること
- Visual Studio Code with Dev Containers 拡張機能

### 1. 開発環境の構築

#### Docker コンテナの起動
```bash
# Macのターミナルで実行
docker run -d -p 6080:80 \
  -v "ローカルのプロジェクトパス":/root/ros2_ws \
  --name pai_final_task_container \
  airobotbook/ros2-desktop-ai-robot-book-humble:latest
```

#### VSCode での開発環境接続
1. VSCode を起動
2. 左下の緑色「><」アイコンをクリック
3. "Dev Containers: Attach to Running Container..." を選択
4. `pai_final_task_container` を選択
5. フォルダーを開く: `/root/ros2_ws`

### 2. プロジェクトのビルド

```bash
# VSCode内ターミナルまたはDockerコンテナ内で実行
cd /root/ros2_ws
colcon build --packages-select final_project_pkg
source install/setup.bash
```

### 3. 実行方法

#### 方法1: 簡単実行（推奨）
```bash
# デモスクリプトを使用
./run_demo.sh
```

#### 方法2: 手動実行
```bash
# 環境設定
export TURTLEBOT3_MODEL=waffle_pi
source install/setup.bash

# シミュレーション実行
ros2 launch final_project_pkg final_project_autonomous.launch.py
```

### 4. 動作確認
ブラウザで `http://localhost:6080/` を開き、以下を確認。
1. Gazebo シミュレーターの起動
2. TurtleBot3 の出現
3. AI による自律配送タスクの実行

## 🔄 動作フロー

ロボットは以下の段階的なタスクを自律実行します。

### Phase 1: 初期化 (0-3秒)
- AI システムの初期化
- センサーデータの取得開始
- 配送タスクの準備

### Phase 2: 環境分析 (3-5秒)
- LiDAR による周囲環境のスキャン
- AI による配送順序の最適化
- 最初の配送目標の選択

### Phase 3: 配送サイクル (各配送につき約30-40秒)
1. **移動フェーズ**
   - 障害物検知・回避を行いながら目標地点へ移動
   - オドメトリによる距離追跡
   - 適応的速度制御

2. **ピックアップ・フェーズ**
   - 目標地点到達の確認
   - パッケージスキャン（AIシミュレーション）
   - ピックアップ動作の実行

3. **配送フェーズ**
   - 配送先への経路計算
   - 安全な配送実行
   - 配送完了の確認

### Phase 4: ミッション完了
- 全配送タスクの完了確認
- ベース位置への帰還
- システム終了

## 🛠️ 主要コマンドリファレンス

### Docker 関連
```bash
# コンテナ操作
docker start pai_final_task_container    # コンテナ起動
docker stop pai_final_task_container     # コンテナ停止
docker ps                                # 実行中コンテナ一覧
```

### ROS2 関連
```bash
# ビルドと環境設定
colcon build --packages-select final_project_pkg
source install/setup.bash

# 実行
ros2 launch final_project_pkg final_project_autonomous.launch.py
ros2 run final_project_pkg task_executor

# デバッグ
ros2 topic list                          # トピック一覧
ros2 topic echo /cmd_vel                 # 速度指令の確認
ros2 topic echo /scan                    # LiDARデータの確認
```

## 🔧 技術仕様詳細

### AI アルゴリズム
- **障害物検知**: NumPy による高速ベクトル演算
- **回避判断**: 3段階の距離閾値による段階的制御
  - 緊急回避: 0.5m 以下
  - 減速回避: 0.8m 以下
  - 通常走行: 0.8m 以上
- **経路選択**: 左右の安全度比較による最適経路選択

### センサー処理
- **LiDAR**: 360度スキャン、0.25度分解能
- **オドメトリ**: 位置・姿勢・速度の6DOF追跡
- **データ更新頻度**: 10Hz (0.1秒間隔)

### 制御パラメータ
```python
# 主要パラメータ
max_linear_speed = 0.3      # 最大直進速度 (m/s)
max_angular_speed = 1.0     # 最大角速度 (rad/s)
min_obstacle_distance = 0.5 # 緊急停止距離 (m)
safe_distance = 0.8        # 安全距離 (m)
```

## 🐛 トラブルシューティング

### ビルドエラーが発生する場合
```bash
# キャッシュクリア後の再ビルド
rm -rf build/ install/ log/
colcon build --packages-select final_project_pkg
```

### Gazebo が起動しない場合
```bash
# TurtleBot3 パッケージの再インストール
sudo apt update
sudo apt install ros-humble-turtlebot3*
```

### 権限エラーが発生する場合
```bash
# 権限の修正（Docker環境内で実行）
sudo chown -R ubuntu:ubuntu /root/ros2_ws
sudo chmod 755 /root
```

### パフォーマンスが低い場合
```bash
# Gazebo の設定調整
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

## 📚 関連ドキュメント

- [技術マニュアル_ROS2_Docker.md](../技術マニュアル_ROS2_Docker.md) - 開発環境の詳細設定
- [開発マニュアル_PhysicalAI2025最終課題.md](../開発マニュアル_PhysicalAI2025最終課題.md) - プロジェクト開発ガイド
- [ROS2 公式ドキュメント](https://docs.ros.org/en/humble/)

---

**Physical AI Course 2025 - Final Project**  
Created by: yf591  
Last Updated: August 1, 2025
