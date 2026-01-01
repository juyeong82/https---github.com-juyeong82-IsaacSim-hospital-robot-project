#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

class DockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')
        
        self.target_id = 4       
        self.tag_size = 0.25     # 태그 크기 25cm

        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.create_subscription(CameraInfo, '/front_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        
        self.get_logger().info(f"🚀 Final Mode: Mapping corrected based on user observation")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("✅ Camera Info Received!")

    def detection_callback(self, msg):
        if self.camera_matrix is None:
            return

        for detection in msg.detections:
            det_id = detection.id[0] if isinstance(detection.id, (list, tuple)) else detection.id
            
            if det_id == self.target_id:
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

                # 기본 Solver 사용 (Fisheye 등 왜곡 보정 포함)
                success, rvec, tvec = cv2.solvePnP(
                    object_points, 
                    image_points, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )

                if success:
                    # OpenCV Raw 좌표 (사용자 관측 기준)
                    # raw_x: 좌우 (오른쪽+)
                    # raw_y: 상하 (아래+)
                    # raw_z: 거리 (앞+)
                    raw_x, raw_y, raw_z = tvec[0][0], tvec[1][0], tvec[2][0]

                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = "Camera"
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # ---------------------------------------------------------
                    # [최종 수정] 사용자 관측 기반 매핑
                    # ---------------------------------------------------------
                    pose_msg.pose.position.x = raw_z      # 거리 (Z -> X)
                    pose_msg.pose.position.y = -raw_x     # 좌우 (X -> Y, 부호반대)
                    pose_msg.pose.position.z = -raw_y     # 상하 (Y -> Z, 부호반대)

                    # 방향 초기화 (정면 응시)
                    pose_msg.pose.orientation.w = 1.0
                    
                    self.publisher.publish(pose_msg)
                    
                    # 디버깅 로그: 실제 ROS로 들어가는 X(거리)와 Y(좌우) 확인
                    self.get_logger().info(
                        f"👁️ RAW: X={raw_x:.2f}, Y={raw_y:.2f}, Z={raw_z:.2f}  ==>  🤖 ROS: Dist(X)={pose_msg.pose.position.x:.2f}, Side(Y)={pose_msg.pose.position.y:.2f}"
                    )
                return

def main(args=None):
    rclpy.init(args=args)
    node = DockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()