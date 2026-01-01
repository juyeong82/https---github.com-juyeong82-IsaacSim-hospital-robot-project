# visual_servoing_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from moma_interfaces.msg import MarkerArray
from std_msgs.msg import Bool, String
import numpy as np
from scipy.spatial.transform import Rotation

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')
        
        # ========================================
        # 파라미터 설정
        # ========================================
        
        # 목표 마커 ID
        self.target_marker_id = 0  # 기본값, 파라미터로 변경 가능
        
        # 제어 게인
        self.kp_linear = 0.25     # 전진 속도 게인 (낮춰서 안정성 향상)
        self.kp_angular = 0.6     # 회전 속도 게인
        
        # 목표 거리 (마커 중심으로부터 몇 m 앞에 정지할지)
        self.target_distance = 0.5  # 50cm 앞
        
        # 허용 오차
        self.distance_threshold = 0.03   # ±3cm
        self.angle_threshold = 0.087     # ±5도 (0.087 rad)
        
        # 속도 제한
        self.max_linear_speed = 0.12
        self.max_angular_speed = 0.25
        
        # 최소 속도 (Dead Zone 방지)
        self.min_linear_speed = 0.02
        self.min_angular_speed = 0.05
        
        # ========================================
        # 상태 변수
        # ========================================
        
        self.is_enabled = False
        self.current_marker = None
        self.servoing_complete = False
        self.marker_lost_count = 0
        self.max_marker_lost = 10  # 10번 연속 안 보이면 경고
        
        # ========================================
        # Subscriber
        # ========================================
        
        # ArUco 검출 결과 수신
        self.create_subscription(
            MarkerArray, 
            '/vision/front_markers', 
            self.marker_callback, 
            10
        )
        
        # On/Off 제어 (토픽 이름 변경)
        self.create_subscription(
            Bool, 
            '/visual_servo/enable', 
            self.enable_callback, 
            10
        )
        
        # ========================================
        # Publisher
        # ========================================
        
        # 속도 명령 출력
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 완료 신호 출력
        self.complete_pub = self.create_publisher(Bool, '/visual_servo/complete', 10)
        
        # 상태 피드백
        self.status_pub = self.create_publisher(String, '/visual_servo/status', 10)
        
        # ========================================
        # 제어 루프 타이머 (20Hz)
        # ========================================
        
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("✅ Visual Servoing Node Ready")
        self.get_logger().info(f"   Target Distance: {self.target_distance}m")
        self.get_logger().info(f"   Target Marker ID: {self.target_marker_id}")
        self.get_logger().info(f"   Enable: ros2 topic pub /visual_servo/enable std_msgs/msg/Bool \"data: true\" -1")

    # ========================================
    # 콜백 함수들
    # ========================================

    def enable_callback(self, msg):
        """비주얼 서보잉 시작/정지"""
        if msg.data and not self.is_enabled:
            self.get_logger().info("🟢 Visual Servoing STARTED")
            self.is_enabled = True
            self.servoing_complete = False
            self.current_marker = None
            self.marker_lost_count = 0
            
        elif not msg.data and self.is_enabled:
            self.get_logger().info("🔴 Visual Servoing STOPPED")
            self.is_enabled = False
            self.stop_robot()
            self.servoing_complete = False

    def marker_callback(self, msg):
        """ArUco 검출 결과 수신"""
        if not self.is_enabled:
            return
        
        # 목표 마커 찾기
        found = False
        for marker in msg.markers:
            if marker.id == self.target_marker_id:
                self.current_marker = marker
                self.marker_lost_count = 0
                found = True
                break
        
        # 목표 마커를 찾지 못함
        if not found:
            self.current_marker = None
            self.marker_lost_count += 1

    # ========================================
    # 제어 로직
    # ========================================

    def control_loop(self):
        """메인 제어 루프"""
        
        if not self.is_enabled:
            return
        
        # 완료 상태면 정지 유지
        if self.servoing_complete:
            self.stop_robot()
            return
        
        # 마커가 안 보이면 정지
        if self.current_marker is None:
            self.stop_robot()
            
            if self.marker_lost_count > self.max_marker_lost:
                self.publish_status(f"⚠️ Marker {self.target_marker_id} Lost!")
                self.get_logger().warn(f"Marker {self.target_marker_id} not detected!")
            
            return
        
        # 마커 포즈 추출 (base_link 기준)
        pose = self.current_marker.pose
        
        # 위치
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        # ========================================
        # [핵심 수정] 좌표계 확인 및 제어 목표 계산
        # ========================================
        
        # base_link 기준: 
        # - x축: 전방
        # - y축: 좌측
        # - z축: 상방
        
        # 1. 현재 거리 (XY 평면상 거리)
        current_distance = np.sqrt(x**2 + y**2)
        
        # 2. 목표까지 가야 할 거리
        distance_error = current_distance - self.target_distance
        
        # 3. 마커 방향 각도 (로봇이 마커를 바라보기 위해 회전해야 할 각도)
        # atan2(y, x): y가 양수면 왼쪽, 음수면 오른쪽
        angle_to_marker = np.arctan2(y, x)
        
        # 디버깅 정보 출력 (처음 몇 번만)
        if self.marker_lost_count == 0 and not self.servoing_complete:
            self.get_logger().info(
                f"📊 Marker Pos: x={x:.3f}, y={y:.3f}, z={z:.3f} | "
                f"Dist={current_distance:.3f}m, Angle={np.degrees(angle_to_marker):.1f}°"
            )
        
        # ========================================
        # 도착 판정
        # ========================================
        
        if (abs(distance_error) < self.distance_threshold and 
            abs(angle_to_marker) < self.angle_threshold):
            
            self.servoing_complete = True
            self.stop_robot()
            
            # 완료 신호 전송
            complete_msg = Bool()
            complete_msg.data = True
            self.complete_pub.publish(complete_msg)
            
            self.get_logger().info("🎯 Visual Servoing COMPLETE!")
            self.get_logger().info(f"   Final Distance: {current_distance:.3f}m")
            self.get_logger().info(f"   Final Angle: {np.degrees(angle_to_marker):.2f}°")
            self.publish_status("✅ Docking Complete")
            return
        
        # ========================================
        # 속도 계산 (P 제어)
        # ========================================
        
        # 전진 속도: 거리 오차에 비례
        # - 양수: 앞으로 이동
        # - 음수: 뒤로 이동 (목표 거리보다 가까울 때)
        linear_x = self.kp_linear * distance_error
        
        # 회전 속도: 각도 오차에 비례
        # - 양수: 반시계 방향 (CCW, 왼쪽으로 회전)
        # - 음수: 시계 방향 (CW, 오른쪽으로 회전)
        angular_z = self.kp_angular * angle_to_marker
        
        # ========================================
        # 속도 제한 및 Dead Zone 처리
        # ========================================
        
        # 속도 제한
        linear_x = np.clip(linear_x, -self.max_linear_speed, self.max_linear_speed)
        angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
        
        # Dead Zone 방지 (너무 작은 속도는 최소값으로)
        if abs(linear_x) > 0.001 and abs(linear_x) < self.min_linear_speed:
            linear_x = np.sign(linear_x) * self.min_linear_speed
        
        if abs(angular_z) > 0.001 and abs(angular_z) < self.min_angular_speed:
            angular_z = np.sign(angular_z) * self.min_angular_speed
        
        # ========================================
        # 속도 명령 전송
        # ========================================
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        
        # 상태 피드백 (0.5초에 한 번만 출력)
        if int(self.get_clock().now().nanoseconds / 1e9 * 2) % 1 == 0:
            status = (
                f"🎯 Dist: {current_distance:.3f}m (err: {distance_error:+.3f}m) | "
                f"Angle: {np.degrees(angle_to_marker):+.1f}° | "
                f"Cmd: v={linear_x:+.2f}, w={angular_z:+.2f}"
            )
            self.publish_status(status)

    # ========================================
    # 유틸리티 함수
    # ========================================

    def stop_robot(self):
        """로봇 정지"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self, message):
        """상태 메시지 발행"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = VisualServoingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()