import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml
import os
from math import sin, cos

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # 파라미터 선언
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('use_sim_time', False)
        
        waypoints_file = self.get_parameter('waypoints_file').value
        
        # Nav2 Action Client 생성
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 상태 발행 토픽 (모니터링용)
        self.status_pub = self.create_publisher(String, 'waypoint_status', 10)
        
        # Waypoint 로드
        self.waypoints = self.load_waypoints(waypoints_file)
        self.current_idx = 0
        self.total_waypoints = len(self.waypoints)
        
        self.get_logger().info('========================================')
        self.get_logger().info(' Waypoint Navigator 초기화 완료')
        self.get_logger().info(f' 총 {self.total_waypoints}개 목표 지점 로드됨')
        self.get_logger().info('========================================')
        
        # 잠시 대기 후 시작 (Nav2가 준비될 때까지)
        self.timer = self.create_timer(3.0, self.start_navigation)
    
    def load_waypoints(self, filepath):
        """YAML 파일에서 waypoint 좌표 로드"""
        try:
            if not filepath or not os.path.exists(filepath):
                self.get_logger().warn(f' Waypoint 파일을 찾을 수 없습니다: {filepath}')
                self.get_logger().warn(' 기본 waypoint를 사용합니다.')
                return self.get_default_waypoints()
            
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                waypoints = data.get('waypoints', [])
                
                if not waypoints:
                    self.get_logger().warn(' Waypoint가 비어있습니다. 기본값 사용')
                    return self.get_default_waypoints()
                
                self.get_logger().info(f' Waypoint 파일 로드 성공: {filepath}')
                return waypoints
                
        except Exception as e:
            self.get_logger().error(f' Waypoint 로드 실패: {e}')
            self.get_logger().warn('  기본 waypoint를 사용합니다.')
            return self.get_default_waypoints()
    
    def get_default_waypoints(self):
        """기본 waypoint (테스트용)"""
        return [
            {'name': '시작점', 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'description': '대기 위치'},
            {'name': '화단1', 'x': 2.0, 'y': 1.0, 'yaw': 0.0, 'description': '첫 번째 화단'},
            {'name': '화단2', 'x': 3.0, 'y': 2.0, 'yaw': 1.57, 'description': '두 번째 화단'},
            {'name': '복귀', 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'description': '복귀 지점'},
        ]
    
    def start_navigation(self):
        """Navigation 시작 (타이머로 한 번만 호출)"""
        self.timer.cancel()  # 타이머 취소
        self.get_logger().info('')
        self.get_logger().info(' 자율 주행을 시작합니다!')
        self.get_logger().info('')
        self.send_next_goal()
    
    def send_next_goal(self):
        """다음 waypoint로 이동 명령 전송"""
        if self.current_idx >= self.total_waypoints:
            self.get_logger().info('')
            self.get_logger().info('========================================')
            self.get_logger().info(' 모든 Waypoint 주행 완료!')
            self.get_logger().info(f'   총 {self.total_waypoints}개 지점 방문 완료')
            self.get_logger().info('========================================')
            self.get_logger().info('')
            
            # 상태 발행
            status_msg = String()
            status_msg.data = 'MISSION_COMPLETED'
            self.status_pub.publish(status_msg)
            return
        
        waypoint = self.waypoints[self.current_idx]
        
        self.get_logger().info('----------------------------------------')
        self.get_logger().info(f' 목표 지점 {self.current_idx + 1}/{self.total_waypoints}')
        self.get_logger().info(f'   이름: {waypoint["name"]}')
        self.get_logger().info(f'   좌표: ({waypoint["x"]:.2f}, {waypoint["y"]:.2f})')
        
        if 'description' in waypoint:
            self.get_logger().info(f'   설명: {waypoint["description"]}')
        
        self.get_logger().info('----------------------------------------')
        
        # Goal 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 위치 설정
        goal_msg.pose.pose.position.x = float(waypoint['x'])
        goal_msg.pose.pose.position.y = float(waypoint['y'])
        goal_msg.pose.pose.position.z = 0.0
        
        # 방향 설정 (quaternion)
        yaw = float(waypoint.get('yaw', 0.0))
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2.0)
        
        # Nav2 Action Server 대기
        self.get_logger().info('⏳ Nav2 Action Server 연결 대기 중...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(' Nav2 Action Server에 연결할 수 없습니다!')
            self.get_logger().error('   Navigation2가 실행 중인지 확인하세요.')
            return
        
        # Goal 전송
        self.get_logger().info('  목표 지점 전송 완료! 주행을 시작합니다...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f'NAVIGATING_TO_{waypoint["name"]}'
        self.status_pub.publish(status_msg)
    
    def goal_response_callback(self, future):
        """Goal이 Nav2에 수락되었는지 확인"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(' Goal이 Nav2에 의해 거부되었습니다!')
            self.get_logger().error('   경로를 계획할 수 없는 위치일 수 있습니다.')
            
            # 다음 waypoint로 넘어가기
            self.current_idx += 1
            self.create_timer(2.0, self.send_next_goal)
            return
        
        self.get_logger().info(' Goal 수락됨! 주행 중...')
        
        # Result 대기
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Goal 완료 시 호출 - 다음 waypoint로 이동"""
        result = future.result()
        
        waypoint = self.waypoints[self.current_idx]
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('')
            self.get_logger().info('========================================')
            self.get_logger().info(f' {waypoint["name"]} 도착 완료!')
            self.get_logger().info('========================================')
            self.get_logger().info('')
        else:
            self.get_logger().warn(f'  {waypoint["name"]} 도착 실패 (Status: {result.status})')
        
        # 상태 발행
        status_msg = String()
        status_msg.data = f'ARRIVED_AT_{waypoint["name"]}'
        self.status_pub.publish(status_msg)
        
        # 다음 waypoint로
        self.current_idx += 1
        
        # 2초 대기 후 다음 목표로 (센서 데이터 수집 시간 확보)
        self.create_timer(2.0, self.send_next_goal)
    
    def feedback_callback(self, feedback_msg):
        """주행 중 피드백 (선택적으로 거리 정보 출력)"""
        feedback = feedback_msg.feedback
        
        # 10초마다 한 번씩 진행 상황 출력
        if hasattr(self, '_last_feedback_time'):
            elapsed = (self.get_clock().now() - self._last_feedback_time).nanoseconds / 1e9
            if elapsed < 10.0:
                return
        
        self._last_feedback_time = self.get_clock().now()
        
        # 남은 거리 출력 (있다면)
        if hasattr(feedback, 'distance_remaining'):
            self.get_logger().info(f' 주행 중... 남은 거리: {feedback.distance_remaining:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('')
        navigator.get_logger().info(' 사용자가 프로그램을 중지했습니다.')
        navigator.get_logger().info('')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()