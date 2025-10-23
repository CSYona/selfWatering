自花水 자율주행 자동 수분 공급 시스템
===========================



* * *

목차
-----

* [프로젝트 개요](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8-%EA%B0%9C%EC%9A%94)
* [시스템 아키텍처](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%EC%8B%9C%EC%8A%A4%ED%85%9C-%EC%95%84%ED%82%A4%ED%85%8D%EC%B2%98)
* [주요 기능](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%EC%A3%BC%EC%9A%94-%EA%B8%B0%EB%8A%A5)
* [하드웨어 구성](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%ED%95%98%EB%93%9C%EC%9B%A8%EC%96%B4-%EA%B5%AC%EC%84%B1)
* [소프트웨어 구성](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%EC%86%8C%ED%94%84%ED%8A%B8%EC%9B%A8%EC%96%B4-%EA%B5%AC%EC%84%B1)
* [설치 방법](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%EC%84%A4%EC%B9%98-%EB%B0%A9%EB%B2%95)
* [사용 방법](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%EC%82%AC%EC%9A%A9-%EB%B0%A9%EB%B2%95)
* [파일 구조](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%ED%8C%8C%EC%9D%BC-%EA%B5%AC%EC%A1%B0)
* [문제 해결](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%EB%AC%B8%EC%A0%9C-%ED%95%B4%EA%B2%B0)
* [향후 개선 사항](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%ED%96%A5%ED%9B%84-%EA%B0%9C%EC%84%A0-%EC%82%AC%ED%95%AD)
* [라이선스](https://claude.ai/chat/529da2e9-3846-4d9e-b0c6-5b869d54c61b#-%EB%9D%BC%EC%9D%B4%EC%84%A0%EC%8A%A4)

* * *

프로젝트 개요
----------

본 프로젝트는 **STELLA N2 자율주행 로봇**을 기반으로 농업 환경에서 자동으로 화단을 순회하며 토양 수분 데이터를 수집하는 시스템입니다.

### 프로젝트 목표

* 사전 정의된 경로(Waypoint)를 따라 자율 주행
* 화단 위치에서 토양 센서 데이터 수집
* 웹 인터페이스를 통한 실시간 모니터링
* 장애물 회피 및 안전한 주행

### 개발 기간

2024년 3월 ~ 2024년 10월

* * *

시스템 아키텍처
------------



![fa632aba-d9c3-4120-95cc-e83dbcb4c909](file:///C:/Users/yonac/Pictures/Typedown/fa632aba-d9c3-4120-95cc-e83dbcb4c909.png)



* * *

주요 기능
-------

**자율 주행**: Waypoint 기반 순차 주행

**장애물 회피**: Nav2 기반 동적 경로 재계획

**SLAM 맵핑**: Cartographer를 이용한 실시간 지도 생성

**RViz 시각화**: 로봇 위치 및 경로 실시간 모니터링

**통합 실행 시스템**: 한 번의 명령으로 전체 시스템 구동

### 핵심 기술 스택

* **ROS2 foxy**: 로봇 제어 및 통신
* **Nav2**: 자율 주행 및 경로 계획
* **Cartographer**: SLAM 기반 맵 생성
* **Python 3.8**: 주행 로직 구현

* * *

하드웨어 구성
----------

### STELLA N2 로봇 (NTREX 제공)

* **모터 드라이버**: Differential Drive
* **LiDAR 센서**: YDLiDAR (360도 회전 스캔 지원)
* **IMU 센서**: AHRS
* **온보드 컴퓨터**: Raspberry Pi 4
* **통신**: WiFi / Ethernet

### 추가 센서

* Crowtail-Moisture Sensor 2.0 ( 토양 수분 센서)
* 데이터 로거 (SD카드)

* * *

소프트웨어 구성
-----------

### ROS2 패키지 구조

    STELLA_REMOTE_PC_ROS2_N2/
    ├── stella_bringup/              # 로봇 하드웨어 구동
    ├── stella_cartographer/         # SLAM 맵핑
    ├── stella_description/          # 로봇 URDF 모델
    ├── stella_navigation2/          # 자율주행 (직접 수정 및 작성)
    │   ├── launch/
    │   │   ├── autonav.launch.py       # 전체 시스템 자동실행
    │   │   └── navigation2.launch.py   # Nav2 실행
    │   ├── scripts/
    │   │   └── waypoint_nav.py         # Waypoint 주행 노드
    │   ├── waypoints/
    │   │   └── waypoints.yaml          # 화단 좌표 데이터
    │   ├── map/
    │   ├── param/
    │   └── rviz/
    ├── stella_md/                   # 모터 드라이버
    ├── stella_ahrs/                 # IMU 센서
    └── ydlidar/                     # LiDAR 드라이버

* * *

설치 방법
--------

### 1. 사전 요구사항

    # 원격 PC를 키고 ctrl+alt+t를 눌러 터미널을 실행
    
    ﻿wget https://raw.githubusercontent.com/ntrexlab/ROS_INSTALL_SCRIPT/main/install_ros2_foxy.sh && chmod 755 ./install_ros2_foxy.sh && bash ./install_ros2_foxy.sh﻿
    
    # 아래 명령어를 사용하여 패키지를 설치
    
    sudo apt-get install ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup
    
    
    # 패키지 설치가 완료되면 아래 명령어를 차례대로 실행하여 라이브러리를 설치
    
    cd ~/colcon_ws/src/
    git clone https://github.com/ntrexlab/STELLA_REMOTE_PC_ROS2_N2.git
    cd ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_teleop/stella_teleop/script
    chmod +x teleop_keyboard.py
    cd ~/colcon_ws
    colcon build --symlink-install
    
    ​
    [출처] [STELLA N2 CAM] 원격 PC와 라즈베리파이 기본 설정 방법|작성자 idea_robot

### 2. 추가한 파일 설정

    cd ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_navigation2
    
    # scripts 폴더 생성 및 파일 추가
    mkdir -p scripts
    # waypoint_nav.py 파일을 scripts/ 폴더에 복사
    chmod +x scripts/waypoint_nav.py
    
    # waypoints 폴더 생성 및 좌표 파일 추가
    mkdir -p waypoints
    # waypoints.yaml 파일을 waypoints/ 폴더에 복사
    
    # launch 파일 추가
    # autonav.launch.py를 launch/ 폴더에 복사

### 3. CMakeLists.txt 수정

    # 기존 install 섹션에 추가
    install(
      DIRECTORY launch map param rviz scripts waypoints
      DESTINATION share/${PROJECT_NAME}
    )
    
    install(PROGRAMS
      scripts/waypoint_nav.py
      DESTINATION lib/${PROJECT_NAME}
    )

### 4. 빌드

    cd ~/colcon_ws
    colcon build --packages-select stella_navigation2
    source install/setup.bash

* * *

사용 방법
--------

### 1단계: 맵 생성 (최초 1회)

    # 로봇 하드웨어 구동
    ros2 launch stella_bringup robot.launch.py
    # 새 터미널에서 수동 조종 패키지 파일 실행
    ﻿ros2 run stella_teleop teleop_keyboard
    # 새 터미널에서 Cartographer 실행
    ros2 launch stella_cartographer cartographer.launch.py
    
    # RViz에서 로봇을 수동 조종하며 맵 생성
    # 맵 생성 완료 후 저장
    ros2 run nav2_map_server map_saver_cli -f ~/map/stella_world

### 2단계: Waypoint 좌표 수정

    # waypoints.yaml 파일 편집
    cd ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_navigation2/waypoints
    nano waypoints.yaml
    
    # 화단 위치에 맞게 좌표 수정
    # RViz에서 Publish Point로 좌표 확인 가능

### 3단계: 자율 주행 실행

    # 전체 시스템 한 번에 실행!
    ros2 launch stella_navigation2 autonav.launch.py

### 주행 과정

1. 로봇 하드웨어 초기화 (5초 소요)
2. Nav2 경로 계획 시스템 시작
3. Waypoint 노드 실행
4. RViz 모니터링 창 자동 실행
5. 첫 번째 waypoint로 자동 주행 시작
6. 각 waypoint 도착 시 1초 대기 후 다음 지점으로 이동
7. 모든 waypoint 완료 시 자동 종료

### 주행 중 제어

    # 긴급 정지
    Ctrl + C
    
    # 특정 waypoint로 이동
    ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
    
    # 주행 상태 확인
    ros2 topic echo /navigation_status

* * *

파일 구조
--------

### 추가된 핵심 파일

    stella_navigation2/
    ├── launch/
    │   └── autonav.launch.py           # 통합 실행 파일 (신규)
    ├── scripts/
    │   └── waypoint_nav.py             # Waypoint 주행 노드 (신규)
    ├── waypoints/
    │   └── waypoints.yaml              # 화단 좌표 데이터 (신규)
    ├── CMakeLists.txt                  # 수정됨
    └── package.xml                     # 수정됨

### 주요 파일 설명

#### `autonav.launch.py`

전체 시스템을 통합 실행하는 launch 파일

* 로봇 하드웨어 구동
* Navigation2 시작
* Waypoint 주행 노드 실행
* RViz 시각화

#### `waypoint_nav.py`

Waypoint를 순차적으로 주행하는 ROS2 노드

* YAML 파일에서 좌표 로드
* Nav2 Action Client를 통한 목표 전송
* 주행 완료 확인 및 다음 목표 이동

#### `waypoints.yaml`

화단 위치 좌표 정의
    waypoints:
      - name: "시작점"
        x: 0.0
        y: 0.0
        yaw: 0.0
      - name: "화단1"
        x: 2.5
        y: 1.5
        yaw: 1.57
      ...

* * *

문제 해결
--------

### 문제 1: "navigate_to_pose Action Server를 찾을 수 없습니다"

**원인**: Nav2가 완전히 시작되지 않음

**해결**:
    # Nav2 상태 확인
    ros2 node list | grep nav2
    # 5-10초 대기 후 다시 시도
    # autonav.launch.py에서 자동으로 5초 대기 설정됨



### 문제 2: 로봇이 waypoint로 이동하지 않음

**원인**: 맵 좌표계 오류 또는 초기 위치 미설정

**해결**:
    # RViz에서 "2D Pose Estimate" 클릭
    # 로봇의 실제 위치와 방향을 지정
    # 맵이 올바르게 로드되었는지 확인
    ros2 topic echo /map --once

### 문제 3: Costmap inflation radius 조정

**문제**: 로봇이 좁은 통로를 통과하지 못함

**해결**:
    # stella.yaml 파일 수정
    nano ~/colcon_ws/src/STELLA_REMOTE_PC_ROS2_N2/stella_navigation2/param/stella.yaml
    # local_costmap과 global_costmap의 inflation_radius 값 조정
    # 기본값: 1.0 → 줄이기: 0.5 (좁은 통로)
    # 기본값: 1.0 → 늘리기: 1.5 (안전 마진 증가)



* * *

참고 자료
--------

### STELLA 로봇 관련

* [STELLA N2 GitHub](https://github.com/ntrexlab/STELLA_REMOTE_PC_ROS2_N2)
* [NTREX 공식 홈페이지](http://www.ntrexgo.com/)

### 참고 튜토리얼

* [Nav2 Waypoint Follower](https://navigation.ros.org/tutorials/docs/navigation2_with_waypoint_follower.html)
* [ROS2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

* * *

라이선스
-------

본 프로젝트는 Apache 2.0 라이선스를 따릅니다.

* STELLA 로봇 패키지: Apache 2.0 (NTREX Co., Ltd.) 
