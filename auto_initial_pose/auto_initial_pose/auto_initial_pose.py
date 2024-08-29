import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.initial_pose_published = False

    def odom_callback(self, msg):
        if not self.initial_pose_published:
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = "map"
            initial_pose.pose.pose = msg.pose.pose
            initial_pose.pose.covariance = [0.1] * 36  # 불확실성 감소를 위한 작은 값
            self.publisher.publish(initial_pose)
            self.get_logger().info('Initial pose published based on odometry.')
            self.initial_pose_published = True

def main(args=None):
    rclpy.init(args=args)
    node = AutoInitialPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

