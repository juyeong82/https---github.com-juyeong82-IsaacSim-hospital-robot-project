#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class DockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')
        
        self.target_id = 4       
        self.tag_size = 0.25     # 태그 크기 25cm
        
        # [추가] object_points는 변하지 않으므로 초기화 때 한 번만 선언 (최적화 핵심)
        s = self.tag_size / 2.0
        self.object_points = np.array([
            [-s, -s, 0], [ s, -s, 0],
            [ s,  s, 0], [-s,  s, 0]
        ], dtype=np.float32)

        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.create_subscription(CameraInfo, '/front_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        
        self.get_logger().info(f"🚀 Dock Pose Publisher Started (Target ID: {self.target_id})")
        
        self.bridge = CvBridge()
        self.latest_image = None
        
        # 디버깅 모드 설정 (True일 때만 화면 띄움)
        self.SHOW_DEBUG_IMAGE = True 
        
        # 초기 상태는 비활성화 (Docking 노드가 켜줘야 작동)
        self.is_active = False
        
        self.create_subscription(Image, '/front_camera/rgb', self.image_callback, 10)     
        
        # 활성화/비활성화 트리거 구독
        self.create_subscription(Bool, '/docking/trigger', self.trigger_callback, 10)   

    # [추가] 도킹 노드에서 보내는 신호(True/False)를 받아 상태 변경
    def trigger_callback(self, msg):
        self.is_active = msg.data
        status = "ON" if self.is_active else "OFF"
        self.get_logger().info(f"🔄 Tracking Status Changed: {status}")
        
        # 비활성화 신호를 받으면 즉시 창 닫기
        if not self.is_active:
            cv2.destroyAllWindows()
        
    def image_callback(self, msg):
        try:
            # ROS Image -> OpenCV Image 변환 (부하 적음)
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    # ---------------------------------------------------------
    # 회전 행렬을 쿼터니언으로 변환하는 헬퍼 함수
    # ---------------------------------------------------------
    def get_quaternion_from_rotation_matrix(self, R):
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        # Euler (x, y, z) -> Quaternion (x, y, z, w)
        cy = np.cos(z * 0.5)
        sy = np.sin(z * 0.5)
        cp = np.cos(y * 0.5)
        sp = np.sin(y * 0.5)
        cr = np.cos(x * 0.5)
        sr = np.sin(x * 0.5)

        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy
        
        return q_x, q_y, q_z, q_w

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("✅ Camera Info Received!")

    def detection_callback(self, msg):
        # [수정] 카메라 정보가 없거나, 비활성화(is_active=False) 상태면 즉시 리턴
        if self.camera_matrix is None or not self.is_active:
            return
        
        # 시각화용 이미지 복사 (이미지가 없으면 빈 화면 생성 방지)
        debug_image = self.latest_image.copy() if (self.latest_image is not None and self.SHOW_DEBUG_IMAGE) else None

        for detection in msg.detections:
            det_id = detection.id[0] if isinstance(detection.id, (list, tuple)) else detection.id
            
            if det_id == self.target_id:
                image_points = np.array([
                    [detection.corners[0].x, detection.corners[0].y],
                    [detection.corners[1].x, detection.corners[1].y],
                    [detection.corners[2].x, detection.corners[2].y],
                    [detection.corners[3].x, detection.corners[3].y]
                ], dtype=np.float32)

                # s = self.tag_size / 2.0
                # object_points = np.array([
                #     [-s, -s, 0], [ s, -s, 0],
                #     [ s,  s, 0], [-s,  s, 0]
                # ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    self.object_points, 
                    image_points, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )

                if success:
                    # Translation & Rotation
                    raw_x, raw_y, raw_z = tvec[0][0], tvec[1][0], tvec[2][0]
                    # rvec -> Rotation Matrix
                    R, _ = cv2.Rodrigues(rvec)

                    # Quaternion 변환 (Publisher용)
                    
                    # 로그 출력을 위해 오일러 각도 별도 계산
                    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
                    singular = sy < 1e-6

                    if not singular:
                        eul_x = np.arctan2(R[2, 1], R[2, 2])
                        eul_y = np.arctan2(-R[2, 0], sy)
                        eul_z = np.arctan2(R[1, 0], R[0, 0])
                    else:
                        eul_x = np.arctan2(-R[1, 2], R[1, 1])
                        eul_y = np.arctan2(-R[2, 0], sy)
                        eul_z = 0
                    
                    # 라디안 -> 도(Degree) 변환
                    deg_x = np.degrees(eul_x)
                    deg_y = np.degrees(eul_y)
                    deg_z = np.degrees(eul_z)

                    # Matrix -> Quaternion
                    qx, qy, qz, qw = self.get_quaternion_from_rotation_matrix(R)

                    # Pose Publish
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = "Camera"
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    pose_msg.pose.position.x = raw_x
                    pose_msg.pose.position.y = raw_y
                    pose_msg.pose.position.z = raw_z

                    # 계산된 쿼터니언 입력
                    pose_msg.pose.orientation.x = qx
                    pose_msg.pose.orientation.y = qy
                    pose_msg.pose.orientation.z = qz
                    pose_msg.pose.orientation.w = qw
                    
                    self.publisher.publish(pose_msg)
                    
                    # [로그 수정] 각도(Degree) 정보 포함 (Y축 회전이 주로 Yaw임)
                    self.get_logger().info(
                        f"🔍 [TRACKING] ID:{self.target_id} | Dist:{raw_z:4.2f}m | "
                        f"Deg X:{deg_x:6.2f} Y:{deg_y:6.2f} Z:{deg_z:6.2f}",
                        throttle_duration_sec=0.5
                    )
                    self.last_detection_time = self.get_clock().now()
                    
                    # 저부하 시각화 (Low-Overhead Visualization)
                    if self.SHOW_DEBUG_IMAGE and debug_image is not None:
                        # 좌표축 그리기 (길이 0.15m) -> X:빨강, Y:초록, Z:파랑 자동 생성
                        cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.15)
                        
                        # 상태 텍스트 표시 (거리, Yaw 각도 등)
                        yaw_deg = deg_y
                        
                        info_text = f"ID:{det_id} Dist:{raw_z:.2f}m Yaw:{yaw_deg:.1f}deg"
                        
                        # 텍스트 배경 박스 (가독성)
                        cv2.rectangle(debug_image, (10, 10), (450, 40), (0, 0, 0), -1)
                        cv2.putText(debug_image, info_text, (20, 32), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        # 마커 테두리 (녹색)
                        pts = image_points.astype(int).reshape((-1, 1, 2))
                        cv2.polylines(debug_image, [pts], True, (0, 255, 0), 2)
        
        # 화면 표시 (이미지가 있을 때만 1ms 대기)
        if debug_image is not None:
            cv2.imshow("Docking Debug", debug_image)
            cv2.waitKey(1)

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