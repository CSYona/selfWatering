from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rotate_robot',
            executable='main',
            name='odom_subscriber',
            output='screen',
        ),
    ])
