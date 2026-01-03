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
        
        # 마지막 인식 시간 저장을 위한 변수
        self.last_detection_time = self.get_clock().now()

        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.create_subscription(CameraInfo, '/front_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        
        self.get_logger().info(f"🚀 Dock Pose Publisher Started (Target ID: {self.target_id})")

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

                success, rvec, tvec = cv2.solvePnP(
                    object_points, 
                    image_points, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )

                if success:
                    # ============================================
                    # OpenCV 카메라 좌표계 그대로 전달
                    # ============================================
                    # X: 오른쪽 +
                    # Y: 아래 +
                    # Z: 전방 +
                    raw_x, raw_y, raw_z = tvec[0][0], tvec[1][0], tvec[2][0]

                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = "Camera"  # ← Camera 프레임
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # 카메라 좌표 그대로 전달 (YAML에서 회전 처리)
                    pose_msg.pose.position.x = raw_x
                    pose_msg.pose.position.y = raw_y
                    pose_msg.pose.position.z = raw_z

                    # Orientation: 정면 응시 (회전 없음)
                    pose_msg.pose.orientation.w = 1.0
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = 0.0
                    
                    self.publisher.publish(pose_msg)
                    
                    self.get_logger().info(
                        f"🔍 [TRACKING] ID:{self.target_id} | X:{raw_x:6.2f} Y:{raw_y:6.2f} Z:{raw_z:6.2f}",
                        throttle_duration_sec=3.0  # 출력 빈도 1초로 제한
                    )
                    self.last_detection_time = self.get_clock().now()
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