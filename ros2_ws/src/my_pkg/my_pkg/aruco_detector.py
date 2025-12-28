import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moma_interfaces.msg import MarkerArray, MarkerInfo
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_left')
        
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
        self.create_subscription(Image, '/left_camera/rgb', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/left_camera/camera_info', self.info_callback, 10)
        
        # RMPFlow 타겟 퍼블리셔(디버깅용)
        # self.pose_pub = self.create_publisher(PoseStamped, '/rmp_target_pose', 10)
        
        # [On/Off 스위치] 외부에서 True를 보내면 검출 시작
        self.create_subscription(Bool, '/vision/enable_left', self.enable_callback, 10)
        
        # [결과 송신] 직접 제어(/rmp_target_pose) 대신 정보만 제공
        self.result_pub = self.create_publisher(MarkerArray, '/vision/left_markers', 10)
        
        # 상태 변수
        self.is_enabled = False  # 기본값: 꺼짐
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 그리퍼가 바닥을 향하는 orientation (원래 코드와 동일)
        euler = np.array([0, np.pi/2, 0])  # roll, pitch, yaw
        rot = Rotation.from_euler('xyz', euler)
        self.default_quat = rot.as_quat()  # [x, y, z, w]
        
        self.get_logger().info("✅ Front Camera Detector Ready (Waiting for Enable Signal...)")
        
    
    def enable_callback(self, msg):
        """On/Off 스위치 콜백"""
        if msg.data and not self.is_enabled:
            self.get_logger().info("🟢 Detector STARTED")
            self.is_enabled = True
        elif not msg.data and self.is_enabled:
            self.get_logger().info("🔴 Detector STOPPED")
            self.is_enabled = False

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        # 1. 꺼져있거나 카메라 정보가 없으면 패스
        if not self.is_enabled or self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            # 보낼 메시지 객체 생성
            marker_array = MarkerArray()
            marker_array.header.stamp = self.get_clock().now().to_msg()
            marker_array.header.frame_id = "base_link" # 최종 좌표계 기준
            
            # 3D 좌표 계산을 위한 마커 정의
            marker_half = self.marker_size / 2.0
            obj_points = np.array([
                [-marker_half, marker_half, 0],
                [marker_half, marker_half, 0],
                [marker_half, -marker_half, 0],
                [-marker_half, -marker_half, 0]
            ], dtype=np.float32)

            # 모든 검출된 마커에 대해 수행
            for i in range(len(ids)):
                current_id = int(ids[i][0])
                # solvePnP를 사용하여 각 마커의 포즈 계산
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
                    # ---------------------------------------------------------
                    # [수정] 로컬 오프셋 적용을 위한 행렬 연산
                    # ---------------------------------------------------------
                    
                    # (1) rvec(회전벡터) -> R(3x3 회전행렬) 변환
                    R, _ = cv2.Rodrigues(rvec)
                    
                    # (2) 마커의 변환 행렬 (4x4) 생성 (카메라 기준 마커 위치)
                    T_cam_marker = np.eye(4)
                    T_cam_marker[:3, :3] = R
                    T_cam_marker[:3, 3] = tvec.squeeze()
                    
                    # (3) 오프셋 행렬 생성 (마커 기준 로컬 오프셋)
                    # OpenCV Aruco 좌표계 기준: X(우), Y(하), Z(전방)
                    # 목표: 마커 뒤쪽(튜브 중심) + 마커 위쪽(튜브 상단)
                    T_offset = np.eye(4)
                    
                    T_offset[0, 3] = 0.0      # X (좌우)
                    T_offset[1, 3] = 0.03    # Y (위아래, 위가 -)
                    T_offset[2, 3] = -0.04    # Z (앞뒤, 뒤가 -)
                    
                    # (4) 마커 위치에 오프셋을 곱함 -> 최종 잡아야 할 위치 (카메라 기준)
                    T_cam_target = T_cam_marker @ T_offset
                    
                    # ---------------------------------------------------------
                    
                    # UR10 베이스 프레임 변환 준비
                    target_frame = "base_link"
                    source_frame = "left_Camera" # TF 트리에 등록된 정확한 카메라 프레임 이름 확인 필요
                    
                    # PoseStamped 설정 (위에서 계산한 T_cam_target 사용)
                    p_cam = PoseStamped()
                    p_cam.header.frame_id = source_frame
                    p_cam.header.stamp = msg.header.stamp
                    
                    p_cam.pose.position.x = T_cam_target[0, 3]
                    p_cam.pose.position.y = T_cam_target[1, 3]
                    p_cam.pose.position.z = T_cam_target[2, 3]
                    p_cam.pose.orientation.w = 1.0 # 위치만 변환할 것이므로 회전은 일단 무시

                    # TF 변환: 카메라 -> UR10 베이스
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # 좌표 변환 수행
                    p_robot_pose = tf2_geometry_msgs.do_transform_pose(p_cam.pose, transform)
                    
                    info = MarkerInfo()
                    info.id = int(ids[i][0])
                    
                    # 변환된 좌표를 그대로 사용 (이미 마커 기준 오프셋이 적용됨)
                    info.pose.position.x = p_robot_pose.position.x
                    info.pose.position.y = p_robot_pose.position.y
                    info.pose.position.z = p_robot_pose.position.z
                    
                    # 그리퍼 Orientation (항상 바닥을 보거나 특정 방향 고정)
                    # 사용자가 설정한 고정값 사용
                    info.pose.orientation.x = self.default_quat[0]
                    info.pose.orientation.y = self.default_quat[1]
                    info.pose.orientation.z = self.default_quat[2]
                    info.pose.orientation.w = self.default_quat[3]
                    
                    marker_array.markers.append(info)

                    self.get_logger().info(f"ID {ids[i][0]}: Target(Base) -> X:{p_robot_pose.position.x:.3f}, Y:{p_robot_pose.position.y:.3f}, Z:{p_robot_pose.position.z:.3f}")

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(f"TF Error: {e}")
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