#!/usr/bin/env python3
import rclpy
import transforms3d.euler
from rclpy.node import Node
from nav_msgs.msg import Odometry

def euler_from_quaternion(orientation_list):
    return transforms3d.euler.quat2euler(orientation_list)

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.get_rotation,
            10)
        self.subscription  # prevent unused variable warning
        self.roll = self.pitch = self.yaw = 0.0

    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.get_logger().info(f'Yaw: {self.yaw}')

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    # 추가된 로그 메시지
    odom_subscriber.get_logger().info('rotate_robot 노드가 시작되었습니다.')

    try:
        rclpy.spin(odom_subscriber)
    except KeyboardInterrupt:
        odom_subscriber.get_logger().info('rotate_robot 노드가 종료되었습니다.')
        
    odom_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
