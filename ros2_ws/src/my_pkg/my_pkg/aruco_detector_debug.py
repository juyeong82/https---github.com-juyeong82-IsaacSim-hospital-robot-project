import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_v2')
        
        # [수정] 마커 및 검출기 설정 (OpenCV 4.7+ 대응)
        self.marker_size = 0.13
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)
        
        self.bridge = CvBridge()
        
        # TF 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # gripper
        self.create_subscription(Image, '/gripper_camera/rgb', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/gripper_camera/camera_info', self.info_callback, 10)

        # Right Camera
        # self.create_subscription(Image, '/right_camera/rgb', self.image_callback, 10)
        # self.create_subscription(CameraInfo, '/right_camera/camera_info', self.info_callback, 10)
        
        # left
        # self.create_subscription(Image, '/left_camera/rgb', self.image_callback, 10)
        # self.create_subscription(CameraInfo, '/left_camera/camera_info', self.info_callback, 10)
        
        # front
        # self.create_subscription(Image, '/front_camera/rgb', self.image_callback, 10)
        # self.create_subscription(CameraInfo, '/front_camera/camera_info', self.info_callback, 10)
        
        # RMPFlow 타겟 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/rmp_target_pose', 10)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 그리퍼 Orientation (바닥 보기)
        euler = np.array([0, np.pi/2, 0])  # roll, pitch, yaw
        rot = Rotation.from_euler('xyz', euler)
        self.default_quat = rot.as_quat()  # [x, y, z, w]
        
        self.get_logger().info("Ready for ArUco detection (Debug Mode).")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None: return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            marker_half = self.marker_size / 2.0
            obj_points = np.array([
                [-marker_half, marker_half, 0],
                [marker_half, marker_half, 0],
                [marker_half, -marker_half, 0],
                [-marker_half, -marker_half, 0]
            ], dtype=np.float32)

            for i in range(len(ids)):
                _, rvec, tvec = cv2.solvePnP(
                    obj_points, 
                    corners[i][0], 
                    self.camera_matrix, 
                    self.dist_coeffs
                )

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                try:
                    # =========================================================
                    # [수정] 로컬 오프셋 적용 로직 (Matrix 연산)
                    # =========================================================
                    
                    # 1. 회전 벡터(rvec) -> 3x3 회전 행렬(R) 변환
                    R, _ = cv2.Rodrigues(rvec)
                    
                    # 2. 카메라 기준 마커의 변환 행렬 (4x4)
                    T_cam_marker = np.eye(4)
                    T_cam_marker[:3, :3] = R
                    T_cam_marker[:3, 3] = tvec.squeeze()
                    
                    # 3. 마커 기준 오프셋 행렬 (Local Offset)
                    # 좌표계: X(우), Y(하), Z(전) -> OpenCV 기준
                    T_offset = np.eye(4)
                    
                    # [튜닝 포인트] 
                    T_offset[0, 3] = 0.0      # X (좌우)
                    T_offset[1, 3] = 0.03    # Y (위아래, 위가 -)
                    T_offset[2, 3] = -0.04    # Z (앞뒤, 뒤가 -)
                    
                    # 4. 최종 목표 위치 계산 (행렬 곱)
                    T_cam_target = T_cam_marker @ T_offset
                    
                    # =========================================================

                    # PoseStamped 설정
                    source_frame = "gripper_Camera" # 사용중인 카메라 프레임
                    # source_frame = "right_Camera" # 사용중인 카메라 프레임
                    target_frame = "base_link"
                    # source_frame = "left_Camera"
                    # source_frame = "Camera"
                    
                    
                    p_cam = PoseStamped()
                    p_cam.header.frame_id = source_frame
                    p_cam.header.stamp = msg.header.stamp
                    
                    # 계산된 T_cam_target에서 위치 추출
                    p_cam.pose.position.x = T_cam_target[0, 3]
                    p_cam.pose.position.y = T_cam_target[1, 3]
                    p_cam.pose.position.z = T_cam_target[2, 3]
                    p_cam.pose.orientation.w = 1.0

                    # TF 변환: 카메라 -> UR10 베이스
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    p_robot_pose = tf2_geometry_msgs.do_transform_pose(p_cam.pose, transform)
                    
                    # 결과 좌표 (이미 오프셋 적용됨)
                    robot_x = p_robot_pose.position.x
                    robot_y = p_robot_pose.position.y
                    robot_z = p_robot_pose.position.z

                    self.get_logger().info(f"ID {ids[i][0]}: Target -> X:{robot_x:.3f}, Y:{robot_y:.3f}, Z:{robot_z:.3f}")

                    # RMPFlow로 전송
                    target_msg = PoseStamped()
                    target_msg.header.frame_id = target_frame
                    target_msg.header.stamp = self.get_clock().now().to_msg()
                    target_msg.pose.position.x = robot_x
                    target_msg.pose.position.y = robot_y
                    target_msg.pose.position.z = robot_z
                    
                    # Orientation 고정
                    target_msg.pose.orientation.x = self.default_quat[0]
                    target_msg.pose.orientation.y = self.default_quat[1]
                    target_msg.pose.orientation.z = self.default_quat[2]
                    target_msg.pose.orientation.w = self.default_quat[3]
                    
                    self.pose_pub.publish(target_msg)

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                    continue

        cv2.imshow("Aruco View", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()