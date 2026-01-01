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
import math
from collections import deque

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_front')
        
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
        self.create_subscription(Image, '/front_camera/rgb', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/front_camera/camera_info', self.info_callback, 10)
        
        # [On/Off 스위치] 외부에서 True를 보내면 검출 시작
        self.create_subscription(Bool, '/vision/enable_front', self.enable_callback, 10)
        
        # [결과 송신] 직접 제어(/rmp_target_pose) 대신 정보만 제공
        self.result_pub = self.create_publisher(MarkerArray, '/vision/front_markers', 10)
        
        # 상태 변수
        self.is_enabled = False  # 기본값: 꺼짐
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # =========================================================
        # [추가] Yaw 오프셋 설정
        # =========================================================
        self.yaw_offset = 90.0  # -90°를 0°로 만들기 위한 오프셋
        # =========================================================
        
        # =========================================================
        # [추가] 스무딩 필터 (이동 평균)
        # =========================================================
        self.marker_history = {}  # {marker_id: deque of (x, y, z, yaw)}
        self.history_size = 5     # 최근 5프레임 평균
        # =========================================================
        
        # 그리퍼가 바닥을 향하는 orientation (원래 코드와 동일)
        euler = np.array([0, np.pi/2, 0])  # roll, pitch, yaw
        rot = Rotation.from_euler('xyz', euler)
        self.default_quat = rot.as_quat()  # [x, y, z, w]
        
        self.get_logger().info("✅ Front Camera Detector Ready (with Smoothing)")
        self.get_logger().info(f"🔧 Yaw Offset: {self.yaw_offset}° | History Size: {self.history_size}")
        
    
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

    def smooth_pose(self, marker_id, x, y, z, yaw):
        """위치 및 각도 스무딩"""
        if marker_id not in self.marker_history:
            self.marker_history[marker_id] = deque(maxlen=self.history_size)
        
        self.marker_history[marker_id].append((x, y, z, yaw))
        
        # 평균 계산
        history = list(self.marker_history[marker_id])
        avg_x = sum(h[0] for h in history) / len(history)
        avg_y = sum(h[1] for h in history) / len(history)
        avg_z = sum(h[2] for h in history) / len(history)
        
        # Yaw 각도 discontinuity 처리
        yaws = [h[3] for h in history]
        normalized_yaws = [yaws[0]]
        for y in yaws[1:]:
            diff = y - yaws[0]
            while diff > 180: diff -= 360
            while diff < -180: diff += 360
            normalized_yaws.append(yaws[0] + diff)
        
        avg_yaw = sum(normalized_yaws) / len(normalized_yaws)
        
        # -180~180 정규화
        while avg_yaw > 180: avg_yaw -= 360
        while avg_yaw < -180: avg_yaw += 360
        
        return avg_x, avg_y, avg_z, avg_yaw

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

                # 시각화 처리
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                try:
                    target_frame = "base_link"
                    source_frame = "Camera"

                    # 회전 벡터 -> 회전 행렬
                    R_mat, _ = cv2.Rodrigues(rvec)
                    
                    # 카메라 기준 마커 변환 행렬
                    T_cam_marker = np.eye(4)
                    T_cam_marker[:3, :3] = R_mat
                    T_cam_marker[:3, 3] = tvec.squeeze()
                    
                    # 오프셋 (필요시 조정)
                    T_offset = np.eye(4)
                    T_offset[0, 3] = 0.0
                    T_offset[1, 3] = 0.0
                    T_offset[2, 3] = 0.0
                    
                    T_cam_target = T_cam_marker @ T_offset
                    
                    # 회전 행렬 -> Quaternion
                    rot_target = Rotation.from_matrix(T_cam_target[:3, :3])
                    quat_target = rot_target.as_quat()

                    # PoseStamped 설정
                    p_cam = PoseStamped()
                    p_cam.header.frame_id = source_frame
                    p_cam.header.stamp = msg.header.stamp
                    
                    p_cam.pose.position.x = T_cam_target[0, 3]
                    p_cam.pose.position.y = T_cam_target[1, 3]
                    p_cam.pose.position.z = T_cam_target[2, 3]
                    
                    p_cam.pose.orientation.x = quat_target[0]
                    p_cam.pose.orientation.y = quat_target[1]
                    p_cam.pose.orientation.z = quat_target[2]
                    p_cam.pose.orientation.w = quat_target[3]

                    # TF 변환 (Camera -> Base Link)
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    p_robot_pose = tf2_geometry_msgs.do_transform_pose(p_cam.pose, transform)
                    
                    # Raw Yaw 계산
                    q = p_robot_pose.orientation
                    rot_base = Rotation.from_quat([q.x, q.y, q.z, q.w])
                    raw_yaw_deg = rot_base.as_euler('xyz', degrees=True)[2]
                    
                    # 오프셋 적용
                    corrected_yaw_deg = raw_yaw_deg + self.yaw_offset
                    
                    # -180 ~ 180 정규화
                    while corrected_yaw_deg > 180:
                        corrected_yaw_deg -= 360
                    while corrected_yaw_deg < -180:
                        corrected_yaw_deg += 360
                    
                    # =========================================================
                    # [추가] 스무딩 적용
                    # =========================================================
                    raw_x = p_robot_pose.position.x
                    raw_y = p_robot_pose.position.y
                    raw_z = p_robot_pose.position.z
                    
                    smooth_x, smooth_y, smooth_z, smooth_yaw = self.smooth_pose(
                        current_id, raw_x, raw_y, raw_z, corrected_yaw_deg
                    )
                    # =========================================================
                    
                    # 거리 계산
                    dist = math.sqrt(smooth_x**2 + smooth_y**2 + smooth_z**2)
                    
                    # 화면 표시 (스무딩된 값)
                    text = f"ID:{current_id} Dist:{dist:.2f}m Yaw:{smooth_yaw:.1f}deg"
                    cv2.putText(frame, text, (10, 30 + i*30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 콘솔 로그 (첫 번째 마커만)
                    if i == 0:
                        print(f"👁️ [ArUco] ID:{current_id} | "
                              f"X:{smooth_x:.2f}, Y:{smooth_y:.2f}, Z:{smooth_z:.2f} | "
                              f"Yaw:{smooth_yaw:.1f}°")
                    
                    # 스무딩된 Yaw -> Quaternion
                    corrected_rot = Rotation.from_euler('xyz', [0, 0, math.radians(smooth_yaw)])
                    corrected_quat = corrected_rot.as_quat()
                    
                    info = MarkerInfo()
                    info.id = current_id
                    
                    # 스무딩된 위치
                    info.pose.position.x = smooth_x
                    info.pose.position.y = smooth_y
                    info.pose.position.z = smooth_z
                    
                    # 스무딩된 회전
                    info.pose.orientation.x = corrected_quat[0]
                    info.pose.orientation.y = corrected_quat[1]
                    info.pose.orientation.z = corrected_quat[2]
                    info.pose.orientation.w = corrected_quat[3]
                    
                    marker_array.markers.append(info)

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                    continue

            if len(marker_array.markers) > 0:
                self.result_pub.publish(marker_array)
                
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