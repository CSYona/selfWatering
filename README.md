<<<<<<< HEAD
自花水 自律走行自動給水システム
================

* * *

目次
--

* [プロジェクト概要](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E3%83%97%E3%83%AD%E3%82%B8%E3%82%A7%E3%82%AF%E3%83%88%E6%A6%82%E8%A6%81)
* [主要機能](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E4%B8%BB%E8%A6%81%E6%A9%9F%E8%83%BD)
* [ハードウェア構成](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E3%83%8F%E3%83%BC%E3%83%89%E3%82%A6%E3%82%A7%E3%82%A2%E6%A7%8B%E6%88%90)
* [ソフトウェア構成](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E3%82%BD%E3%83%95%E3%83%88%E3%82%A6%E3%82%A7%E3%82%A2%E6%A7%8B%E6%88%90)
* [インストール方法](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB%E6%96%B9%E6%B3%95)
* [使用方法](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E4%BD%BF%E7%94%A8%E6%96%B9%E6%B3%95)
* [ファイル構造](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E6%A7%8B%E9%80%A0)
* [トラブルシューティング](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E3%83%88%E3%83%A9%E3%83%96%E3%83%AB%E3%82%B7%E3%83%A5%E3%83%BC%E3%83%86%E3%82%A3%E3%83%B3%E3%82%B0)
* [今後の改善事項](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E4%BB%8A%E5%BE%8C%E3%81%AE%E6%94%B9%E5%96%84%E4%BA%8B%E9%A0%85)
* [ライセンス](https://claude.ai/chat/dc9dc423-4edb-4c85-9f71-8bd14d703343#%E3%83%A9%E3%82%A4%E3%82%BB%E3%83%B3%E3%82%B9)

* * *

プロジェクト概要
--------

本プロジェクトは、**STELLA N2 自律走行ロボット**をベースに、農業環境で自動的に花壇を巡回しながら土壌水分データを収集するシステムです。

### プロジェクト目標

* 事前定義された経路（Waypoint）に沿った自律走行
* 花壇位置での土壌センサーデータ収集
* Webインターフェースを通じたリアルタイムモニタリング
* 障害物回避と安全な走行

### 開発期間

2024年3月 ～ 2024年10月

 
=======
自花水 자율주행 자동 수분 공급 시스템
===========================
>>>>>>> eac0c38cc5e3038d4ad15b03cd8b3b6d78641b7a



* * *

主要機能
----

**自律走行**: Waypoint基盤の順次走行

**障害物回避**: Nav2基盤の動的経路再計画

**SLAM マッピング**: Cartographerを利用したリアルタイム地図生成

**RViz 視覚化**: ロボット位置および経路のリアルタイムモニタリング

**統合実行システム**: 一度のコマンドで全システムを起動

### コア技術スタック

* **ROS2 Foxy**: ロボット制御および通信
* **Nav2**: 自律走行および経路計画
* **Cartographer**: SLAM基盤マップ生成
* **Python 3.8**: 走行ロジック実装

* * *

ハードウェア構成
--------

### STELLA N2 ロボット (NTREX 提供)

* **モータードライバー**: Differential Drive
* **LiDAR センサー**: YDLiDAR (360度回転スキャン対応)
* **IMU センサー**: AHRS
* **オンボードコンピュータ**: Raspberry Pi 4
* **通信**: WiFi / Ethernet

### 追加センサー

<<<<<<< HEAD
* Crowtail-Moisture Sensor 2.0 (土壌水分センサー)
* データロガー (SDカード)
=======
### 개발 기간

2024년 3월 ~ 2024년 10월
>>>>>>> eac0c38cc5e3038d4ad15b03cd8b3b6d78641b7a

* * *

ソフトウェア構成
--------

### ROS2 パッケージ構造

    STELLA_REMOTE_PC_ROS2_N2/
    ├── stella_bringup/              # ロボットハードウェア起動
    ├── stella_cartographer/         # SLAM マッピング
    ├── stella_description/          # ロボットURDFモデル
    ├── stella_navigation2/          # 自律走行（直接修正・作成）
    │   ├── launch/
    │   │   ├── autonav.launch.py       # 全システム自動実行
    │   │   └── navigation2.launch.py   # Nav2 実行
    │   ├── scripts/
    │   │   └── waypoint_nav.py         # Waypoint 走行ノード
    │   ├── waypoints/
    │   │   └── waypoints.yaml          # 花壇座標データ
    │   ├── map/
    │   ├── param/
    │   └── rviz/
    ├── stella_md/                   # モータードライバー
    ├── stella_ahrs/                 # IMU センサー
    └── ydlidar/                     # LiDAR ドライバー

* * *

インストール方法
--------

### 1. 事前要件

    # リモートPCを起動し、Ctrl+Alt+Tでターミナルを実行
    
    wget https://raw.githubusercontent.com/ntrexlab/ROS_INSTALL_SCRIPT/main/install_ros2_foxy.sh && chmod 755 ./install_ros2_foxy.sh && bash ./install_ros2_foxy.sh
    
    # 以下のコマンドでパッケージをインストール
    
    sudo apt-get install ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup
    
    # パッケージインストール完了後、以下のコマンドを順次実行してライブラリをインストール
    
    cd ~/colcon_ws/src/
    git clone https://github.com/ntrexlab/STELLA_REMOTE_PC_ROS2_N2.git
    cd ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_teleop/stella_teleop/script
    chmod +x teleop_keyboard.py
    cd ~/colcon_ws
    colcon build --symlink-install

[出典] [STELLA N2 CAM] リモートPCとRaspberry Piの基本設定方法|作成者 idea_robot

### 2. 追加ファイル設定

    cd ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_navigation2
    
    # scriptsフォルダ作成およびファイル追加
    mkdir -p scripts
    # waypoint_nav.pyファイルをscripts/フォルダにコピー
    chmod +x scripts/waypoint_nav.py
    
    # waypointsフォルダ作成および座標ファイル追加
    mkdir -p waypoints
    # waypoints.yamlファイルをwaypoints/フォルダにコピー
    
    # launchファイル追加
    # autonav.launch.pyをlaunch/フォルダにコピー

### 3. CMakeLists.txt 修正

    # 既存のinstallセクションに追加
    install(
      DIRECTORY launch map param rviz scripts waypoints
      DESTINATION share/${PROJECT_NAME}
    )
    
    install(PROGRAMS
      scripts/waypoint_nav.py
      DESTINATION lib/${PROJECT_NAME}
    )

### 4. ビルド

    cd ~/colcon_ws
    colcon build --packages-select stella_navigation2
    source install/setup.bash

* * *

使用方法
----

### ステップ1: マップ生成（初回のみ）

    # ロボットハードウェア起動
    ros2 launch stella_bringup robot.launch.py
    
    # 新しいターミナルで手動操縦パッケージファイル実行
    ros2 run stella_teleop teleop_keyboard
    
    # 新しいターミナルでCartographer実行
    ros2 launch stella_cartographer cartographer.launch.py
    
    # RVizでロボットを手動操縦しながらマップ生成
    # マップ生成完了後に保存
    ros2 run nav2_map_server map_saver_cli -f ~/map/stella_world

### ステップ2: Waypoint座標修正

    # waypoints.yamlファイル編集
    cd ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_navigation2/waypoints
    nano waypoints.yaml
    
    # 花壇位置に合わせて座標を修正
    # RVizでPublish Pointから座標確認可能

### ステップ3: 自律走行実行

    # 全システムを一度に実行！
    ros2 launch stella_navigation2 autonav.launch.py

### 走行プロセス

1. ロボットハードウェア初期化（5秒所要）
2. Nav2 経路計画システム開始
3. Waypoint ノード実行
4. RViz モニタリングウィンドウ自動実行
5. 最初のwaypointへ自動走行開始
6. 各waypoint到着時に1秒待機後、次の地点へ移動
7. 全waypoint完了時に自動終了

### 走行中の制御

    # 緊急停止
    Ctrl + C
    
    # 特定waypointへ移動
    ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
    
    # 走行状態確認
    ros2 topic echo /navigation_status

* * *

ファイル構造
------

### 追加されたコアファイル

    stella_navigation2/
    ├── launch/
    │   └── autonav.launch.py           # 統合実行ファイル（新規）
    ├── scripts/
    │   └── waypoint_nav.py             # Waypoint 走行ノード（新規）
    ├── waypoints/
    │   └── waypoints.yaml              # 花壇座標データ（新規）
    ├── CMakeLists.txt                  # 修正済み
    └── package.xml                     # 修正済み

### 主要ファイル説明

#### `autonav.launch.py`

全システムを統合実行するlaunchファイル

* ロボットハードウェア起動
* Navigation2 開始
* Waypoint 走行ノード実行
* RViz 視覚化

#### `waypoint_nav.py`

Waypointを順次走行するROS2ノード

* YAMLファイルから座標をロード
* Nav2 Action Clientを通じた目標送信
* 走行完了確認および次の目標へ移動

#### `waypoints.yaml`

花壇位置座標定義
    waypoints:
      - name: "開始点"
        x: 0.0
        y: 0.0
        yaw: 0.0
      - name: "花壇1"
        x: 2.5
        y: 1.5
        yaw: 1.57
      ...

* * *

トラブルシューティング
-----------

### 問題1: "navigate_to_pose Action Serverが見つかりません"

**原因**: Nav2が完全に起動していない

**解決方法**:
    # Nav2状態確認
    ros2 node list | grep nav2
    # 5-10秒待機後、再試行
    # autonav.launch.pyで自動的に5秒待機設定済み

### 問題2: ロボットがwaypointへ移動しない

**原因**: マップ座標系エラーまたは初期位置未設定

**解決方法**:
    # RVizで「2D Pose Estimate」をクリック
    # ロボットの実際の位置と方向を指定
    # マップが正しくロードされているか確認
    ros2 topic echo /map --once

### 問題3: Costmap inflation radius 調整

**問題**: ロボットが狭い通路を通過できない

**解決方法**:
    # stella.yamlファイル修正
    nano ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_navigation2/param/stella.yaml
    # local_costmapとglobal_costmapのinflation_radius値を調整
    # デフォルト値: 1.0 → 減少: 0.5（狭い通路）
    # デフォルト値: 1.0 → 増加: 1.5（安全マージン増加）

* * *

参考資料
----

### STELLA ロボット関連

* [STELLA N2 GitHub](https://github.com/ntrexlab/STELLA_REMOTE_PC_ROS2_N2)
* [NTREX 公式ホームページ](http://www.ntrexgo.com/)

### 参考チュートリアル

* [Nav2 Waypoint Follower](https://navigation.ros.org/tutorials/docs/navigation2_with_waypoint_follower.html)
* [ROS2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

* * *

ライセンス
-----

本プロジェクトはApache 2.0ライセンスに従います。

* STELLA ロボットパッケージ: Apache 2.0 (NTREX Co., Ltd.)
