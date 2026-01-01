#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class SimpleDocker(Node):
    def __init__(self):
        super().__init__('simple_docker')
        
        # [설정]
        self.target_id = 4       
        self.tag_size = 0.25     # 25cm
        self.stop_distance = 1.35 # 목표 도킹 거리 (35cm 앞에서 멈춤)

        # [제어 게인] 로봇 속도 조절 (너무 빠르면 줄이세요)
        self.k_linear = 0.5   # 전진 속도 계수
        self.k_angular = 2.0  # 회전 속도 계수
        self.max_linear_speed = 0.2 # 최대 전진 속도 (m/s)
        self.max_angular_speed = 0.5 # 최대 회전 속도 (rad/s)

        # 변수 초기화
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 통신 설정
        self.create_subscription(CameraInfo, '/front_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        
        # 로봇 바퀴 제어 (cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("🚀 Simple Docker Started! Looking for Tag 4...")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def detection_callback(self, msg):
        if self.camera_matrix is None:
            return

        target_found = False

        for detection in msg.detections:
            det_id = detection.id[0] if isinstance(detection.id, (list, tuple)) else detection.id
            
            if det_id == self.target_id:
                target_found = True
                
                # 1. 태그 위치 계산 (solvePnP)
                image_points = np.array([
                    [detection.corners[0].x, detection.corners[0].y],
                    [detection.corners[1].x, detection.corners[1].y],
                    [detection.corners[2].x, detection.corners[2].y],
                    [detection.corners[3].x, detection.corners[3].y]
                ], dtype=np.float32)

                s = self.tag_size / 2.0
                object_points = np.array([
                    [-s, -s, 0], [ s, -s, 0],
                    [ s,  s, 0], [-s,  s, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    object_points, image_points, self.camera_matrix, self.dist_coeffs
                )

                if success:
                    # Camera Frame 기준 좌표
                    # Z = 전방 거리 (Distance)
                    # X = 좌우 위치 (Right+, Left-)
                    raw_x = tvec[0][0] 
                    raw_z = tvec[2][0]

                    # 2. 제어 로직 (Visual Servoing)
                    twist = Twist()

                    # (A) 회전 제어: 태그가 화면 중앙(X=0)에 오도록
                    # 태그가 오른쪽(X>0)에 있으면 -> 오른쪽으로 회전(-Z)해야 함
                    error_yaw = -raw_x 
                    twist.angular.z = error_yaw * self.k_angular

                    # (B) 전진 제어: 목표 거리까지 전진
                    error_dist = raw_z - self.stop_distance
                    
                    if error_dist > 0:
                        twist.linear.x = error_dist * self.k_linear
                    else:
                        twist.linear.x = 0.0 # 도착함

                    # 속도 제한 (안전장치)
                    twist.linear.x = min(twist.linear.x, self.max_linear_speed)
                    twist.angular.z = np.clip(twist.angular.z, -self.max_angular_speed, self.max_angular_speed)

                    # (C) 도착 판정
                    if abs(error_dist) < 0.05 and abs(raw_x) < 0.05:
                        self.get_logger().info("✅ Docking Complete! Stopping.")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    
                    # 명령 전송
                    self.cmd_pub.publish(twist)
                    
                    # 로그
                    self.get_logger().info(f"Dist: {raw_z:.2f}m | ErrX: {raw_x:.2f} | Vel: Lin={twist.linear.x:.2f}, Ang={twist.angular.z:.2f}")

        # 태그를 놓쳤을 때 (안전을 위해 정지)
        if not target_found:
            self.get_logger().warn("❌ Tag lost! Stopping robot.")
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()