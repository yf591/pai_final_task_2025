#!/bin/bash
# AI配送ロボット デモ実行スクリプト

echo "🤖 AI強化自律配送ロボット - デモ開始"
echo "============================================"

# ROS2環境のセットアップ
echo "📋 ROS2環境をセットアップ中..."
cd /root/ros2_ws
source install/setup.bash

# TurtleBot3モデルの設定
export TURTLEBOT3_MODEL=waffle_pi

echo "🚀 Gazeboシミュレーションを開始します..."
echo "   - TurtleBot3 Waffle Pi モデルを使用"
echo "   - AI強化配送タスクを実行"
echo ""
echo "📦 配送タスク内容:"
echo "   1. Package A の配送"
echo "   2. Package B の配送" 
echo "   3. Package C の配送"
echo ""
echo "🧠 AI機能:"
echo "   - リアルタイム障害物検知・回避"
echo "   - インテリジェント経路最適化"
echo "   - 自律配送タスク管理"
echo ""
echo "ターミナルでCtrl+Cを押すとシミュレーションを停止できます"
echo "============================================"

# ローンチファイル実行
ros2 launch final_project_pkg final_project_autonomous.launch.py
