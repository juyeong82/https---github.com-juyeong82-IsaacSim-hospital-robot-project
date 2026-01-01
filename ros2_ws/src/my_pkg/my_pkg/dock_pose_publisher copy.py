#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

# [엄격한 상태 정의]
STATE_SEARCH = 0           # 태그 찾는 중
STATE_ROTATE_TO_POINT = 1  # 진입점(2.0m)을 향해 제자리 회전
STATE_DRIVE_TO_POINT = 2   # 진입점까지 직진
STATE_ALIGN_YAW = 3        # 태그와 평행하게 제자리 회전 (Yaw 정렬)
STATE_CHECK_LATERAL = 4    # [핵심] 위치가 정말 맞는지 검사 (틀리면 빠꾸)
STATE_FINAL_APPROACH = 5   # 최종 직진
STATE_DONE = 6             # 완료

class StrictRunwayDocker(Node):
    def __init__(self):
        super().__init__('strict_runway_docker')
        
        # [설정]
        self.target_id = 4
        self.tag_size = 0.25
        
        # [목표 지점]
        self.entry_dist = 2.0      # 활주로 시작점 (2.0m)
        self.dock_dist = 1.3      # 최종 정지 거리 (55cm)
        
        # [허용 오차 (엄격함)]
        self.yaw_tolerance = 0.05     # 각도 오차 (약 3도)
        self.dist_tolerance = 0.1     # 거리 오차 (10cm)
        self.lateral_tolerance = 0.05 # [중요] 좌우 오차 (5cm) - 이거 넘으면 재시도

        # [속도]
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.state = STATE_SEARCH
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.latest_image = None
        
        self.create_subscription(CameraInfo, '/front_camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/front_camera/rgb', self.image_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("👮 Strict Runway Docker Started! (Turn-Drive-Turn Logic)")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def detection_callback(self, msg):
        if self.camera_matrix is None or self.latest_image is None:
            return

        frame = self.latest_image.copy()
        target_det = None

        for detection in msg.detections:
            det_id = detection.id[0] if isinstance(detection.id, (list, tuple)) else detection.id
            if det_id == self.target_id:
                target_det = detection
                break
        
        twist = Twist()
        status_text = "Searching..."
        color_status = (0, 0, 255) # Red

        if target_det:
            # PnP 계산
            tvec, rvec = self.calculate_pose(target_det)
            
            # [좌표 변환] 사용자 관측 기반 (Z=거리, X=좌우, Y=상하)
            # ROS 기준:
            dist = tvec[2][0]      # 전방 거리
            lateral = -tvec[0][0]  # 좌우 (왼쪽+, 오른쪽-)
            
            # Yaw 계산 (태그가 나를 보는 각도)
            rot_mat, _ = cv2.Rodrigues(rvec)
            yaw_error = math.atan2(rot_mat[0][2], rot_mat[2][2])

            # 시각화 (축 그리기)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.2)

            # -----------------------------------------------------------
            # [엄격한 상태 머신]
            # -----------------------------------------------------------
            
            # 목표점 (활주로 입구): (Lateral=0, Dist=2.0)
            target_x = 0.0
            target_z = self.entry_dist

            if self.state == STATE_SEARCH:
                self.state = STATE_ROTATE_TO_POINT
            
            # [STEP 1] 진입점을 향해 회전 (제자리)
            elif self.state == STATE_ROTATE_TO_POINT:
                status_text = "1. Rotate to Entry Point"
                
                # 내 위치에서 (0, 2.0)을 보려면 몇 도 돌려야 하나?
                # 목표점까지의 벡터: (target_x - lateral, target_z - dist)
                # target_x는 0이므로 (-lateral, target_z - dist)
                
                dx = -lateral
                dz = target_z - dist
                target_heading = math.atan2(dx, dz) # 로봇 기준 목표점 방위각
                
                if abs(target_heading) < self.yaw_tolerance:
                    self.state = STATE_DRIVE_TO_POINT
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = np.clip(target_heading * 2.0, -self.angular_speed, self.angular_speed)
                    twist.linear.x = 0.0 # 제자리 회전

            # [STEP 2] 진입점까지 직진
            elif self.state == STATE_DRIVE_TO_POINT:
                status_text = "2. Drive to Entry Point"
                
                # 남은 거리 (유클리드 거리)
                dx = -lateral
                dz = target_z - dist
                distance_remaining = math.sqrt(dx*dx + dz*dz)
                
                # 도착 판정 (10cm 이내)
                if distance_remaining < self.dist_tolerance:
                    self.state = STATE_ALIGN_YAW
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = self.linear_speed
                    # 가면서 살짝 틀어지면 방향 보정 (약하게)
                    heading = math.atan2(dx, dz)
                    twist.angular.z = heading * 1.5

            # [STEP 3] 태그 정면 보기 (Yaw 정렬)
            elif self.state == STATE_ALIGN_YAW:
                status_text = "3. Align Yaw (Face Tag)"
                
                if abs(yaw_error) < self.yaw_tolerance:
                    self.state = STATE_CHECK_LATERAL
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = np.clip(yaw_error * 2.0, -self.angular_speed, self.angular_speed)
                    twist.linear.x = 0.0 # 제자리 회전

            # [STEP 4] 위치 검증 (사용자님 우려 해소!)
            elif self.state == STATE_CHECK_LATERAL:
                status_text = "4. Checking Lateral Alignment..."
                
                # 각도는 맞췄는데, 위치가 옆으로 비껴나 있으면?
                if abs(lateral) > self.lateral_tolerance:
                    self.get_logger().warn(f"⚠️ Misaligned! Lateral Error: {lateral:.2f}m. Retrying...")
                    # 다시 진입점 조준 단계로 강등 (재시도)
                    self.state = STATE_ROTATE_TO_POINT
                else:
                    self.get_logger().info("✅ Perfect Alignment! Landing...")
                    self.state = STATE_FINAL_APPROACH

            # [STEP 5] 최종 착륙 (직진)
            elif self.state == STATE_FINAL_APPROACH:
                status_text = "5. Final Approach (Runway)"
                color_status = (0, 255, 0) # Green
                
                remaining = dist - self.dock_dist
                
                if remaining < 0.02:
                    self.state = STATE_DONE
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = self.linear_speed * 0.8 # 천천히
                    twist.angular.z = yaw_error * 1.0 # 각도만 살짝 유지

            elif self.state == STATE_DONE:
                status_text = "DOCKED COMPLETE"
                color_status = (255, 255, 0)
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # -----------------------------------------------------------
            # [GUI 디버깅] 화면에 수치 표시
            # -----------------------------------------------------------
            # 중앙 십자선
            h, w, _ = frame.shape
            cv2.line(frame, (w//2, 0), (w//2, h), (100, 100, 100), 1)
            cv2.line(frame, (0, h//2), (w, h//2), (100, 100, 100), 1)

            # 텍스트
            cv2.putText(frame, f"[{status_text}]", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_status, 2)
            cv2.putText(frame, f"Dist: {dist:.2f}m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Lateral 오차 (중요)
            lat_color = (0, 255, 0) if abs(lateral) < self.lateral_tolerance else (0, 0, 255)
            cv2.putText(frame, f"Lateral: {lateral:.2f}m", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, lat_color, 2)
            
            # Yaw 오차
            yaw_color = (0, 255, 0) if abs(yaw_error) < self.yaw_tolerance else (0, 0, 255)
            cv2.putText(frame, f"Yaw: {math.degrees(yaw_error):.1f} deg", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, yaw_color, 2)

            self.cmd_pub.publish(twist)

        else:
            cv2.putText(frame, "Waiting for Tag...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.cmd_pub.publish(Twist())

        cv2.imshow("Strict Runway Docker", frame)
        cv2.waitKey(1)

    def calculate_pose(self, detection):
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
        return tvec, rvec

def main(args=None):
    rclpy.init(args=args)
    node = StrictRunwayDocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()