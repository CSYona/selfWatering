import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument, 
    LogInfo,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # 패키지 경로
    stella_bringup_dir = get_package_share_directory('stella_bringup')
    stella_navigation2_dir = get_package_share_directory('stella_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    
    # 설정 파일 경로
    map_file = os.path.join(
        stella_navigation2_dir,
        'map',
        'stella_world.yaml'
    )
    
    param_file = os.path.join(
        stella_navigation2_dir,
        'param',
        'stella.yaml'
    )
    
    waypoints_file = os.path.join(
        stella_navigation2_dir,
        'waypoints',
        'waypoints.yaml'
    )
    
    rviz_config = os.path.join(
        stella_navigation2_dir,
        'rviz',
        'stella_navigation2.rviz'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        
        # ========================================
        # Launch Arguments
        # ========================================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        
        # ========================================
        # 시작 메시지
        # ========================================
        LogInfo(msg=''),
        LogInfo(msg='========================================'),
        LogInfo(msg=' 자율 수분 공급 시스템 시작'),
        LogInfo(msg='   Autonomous Watering System'),
        LogInfo(msg='========================================'),
        LogInfo(msg=''),
        
        # ========================================
        # 1단계: 로봇 하드웨어 구동
        # ========================================
        LogInfo(msg='[1/4] 로봇 하드웨어 초기화 중...'),
        LogInfo(msg='      - Motor Driver (MD)'),
        LogInfo(msg='      - LiDAR Sensor'),
        LogInfo(msg='      - AHRS (IMU)'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(stella_bringup_dir, 'launch', 'robot.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
        # ========================================
        # 2단계: Navigation2 실행
        # ========================================
        LogInfo(msg=''),
        LogInfo(msg='[2/4]  Navigation2 시작 중...'),
        LogInfo(msg='      - Map Server'),
        LogInfo(msg='      - AMCL Localization'),
        LogInfo(msg='      - Path Planner'),
        LogInfo(msg='      - Controller'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file
            }.items(),
        ),
        
        # ========================================
        # 3단계: Waypoint Navigation (5초 후 시작)
        # ========================================
        LogInfo(msg=''),
        LogInfo(msg='[3/4] Waypoint 자동 주행 준비 중...'),
        
        TimerAction(
            period=5.0,  # Nav2가 준비될 시간 확보
            actions=[
                LogInfo(msg=' Waypoint Navigator 시작!'),
                Node(
                    package='stella_navigation2',
                    executable='waypoint_nav.py',
                    name='waypoint_navigator',
                    output='screen',
                    parameters=[{
                        'waypoints_file': waypoints_file,
                        'use_sim_time': use_sim_time
                    }]
                ),
            ]
        ),
        
        # ========================================
        # 4단계: RViz 시각화
        # ========================================
        LogInfo(msg=''),
        LogInfo(msg='[4/4] RViz 모니터링 시작 중...'),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # ========================================
        # 완료 메시지
        # ========================================
        LogInfo(msg=''),
        LogInfo(msg='========================================'),
        LogInfo(msg=' 모든 시스템 준비 완료!'),
        LogInfo(msg=' Waypoint 자동 주행을 시작합니다...'),
        LogInfo(msg='========================================'),
        LogInfo(msg=''),
    ])