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
        
        # 1. 파라미터 생성자 변경 (_create 함수 삭제됨)
        self.params = cv2.aruco.DetectorParameters()
        
        # 2. ArucoDetector 객체 생성 (검출 속도 및 최적화 유리)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)
        
        self.bridge = CvBridge()
        
        # TF 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 카메라 구독
        self.create_subscription(Image, '/front_stereo_camera/left/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/front_stereo_camera/left/camera_info', self.info_callback, 10)
        
        # RMPFlow 타겟 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/rmp_target_pose', 10)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 그리퍼가 바닥을 향하는 orientation (원래 코드와 동일)
        euler = np.array([0, np.pi/2, 0])  # roll, pitch, yaw
        rot = Rotation.from_euler('xyz', euler)
        self.default_quat = rot.as_quat()  # [x, y, z, w]
        
        self.get_logger().info("Ready for ArUco detection.")

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
        
        # [수정됨] ArucoDetector 객체 사용 (이전 단계에서 수정한 부분 유지)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            # [삭제됨] rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(...) <- 이 함수가 삭제됨
            
            # [추가됨] 마커 객체 포인트 정의 (3D 좌표계에서 마커의 코너 위치)
            # 마커 크기의 절반
            marker_half = self.marker_size / 2.0
            # 마커의 4개 코너 좌표 (좌상단부터 시계방향: top-left, top-right, bottom-right, bottom-left)
            obj_points = np.array([
                [-marker_half, marker_half, 0],
                [marker_half, marker_half, 0],
                [marker_half, -marker_half, 0],
                [-marker_half, -marker_half, 0]
            ], dtype=np.float32)

            for i in range(len(ids)):
                # [수정됨] solvePnP를 사용하여 각 마커의 포즈 계산
                # corners[i]는 (1, 4, 2) 형태이므로 (4, 2)로 변환 필요
                _, rvec, tvec = cv2.solvePnP(
                    obj_points, 
                    corners[i][0], 
                    self.camera_matrix, 
                    self.dist_coeffs
                )

                # 시각화 처리 (차원 문제로 인해 rvec, tvec 형태 주의)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                try:
                    # UR10 베이스 프레임 (RMPFlow가 사용하는 좌표계)
                    target_frame = "base_link"
                    source_frame = "front_stereo_camera_left_rgb"
                    
                    # PoseStamped 설정 (카메라 좌표계)
                    p_cam = PoseStamped()
                    p_cam.header.frame_id = source_frame
                    p_cam.header.stamp = msg.header.stamp
                    
                    # tvec은 (3, 1) 형태이므로 인덱싱 주의
                    p_cam.pose.position.x = float(tvec[0][0])
                    p_cam.pose.position.y = float(tvec[1][0])
                    p_cam.pose.position.z = float(tvec[2][0])
                    p_cam.pose.orientation.w = 1.0

                    # TF 변환: 카메라 -> UR10 베이스
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # 좌표 변환
                    p_robot_pose = tf2_geometry_msgs.do_transform_pose(p_cam.pose, transform)
                    
                    # 결과 좌표 추출 (로봇 베이스 기준)
                    robot_x = p_robot_pose.position.x
                    robot_y = p_robot_pose.position.y + 0.04
                    robot_z = p_robot_pose.position.z + 0.04

                    self.get_logger().info(f"ID {ids[i][0]}: Robot Base -> X:{robot_x:.3f}, Y:{robot_y:.3f}, Z:{robot_z:.3f}")

                    # RMPFlow로 전송
                    target_msg = PoseStamped()
                    target_msg.header.frame_id = target_frame
                    target_msg.header.stamp = self.get_clock().now().to_msg()
                    target_msg.pose.position.x = robot_x
                    target_msg.pose.position.y = robot_y
                    target_msg.pose.position.z = robot_z
                    
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