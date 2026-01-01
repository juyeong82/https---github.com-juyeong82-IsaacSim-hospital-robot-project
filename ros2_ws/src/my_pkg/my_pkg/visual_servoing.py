import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moma_interfaces.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

STATE_APPROACH = 0
STATE_ALIGN_YAW = 1
STATE_VERIFY = 2
STATE_SEARCH = 3

class SimplifiedDockingNode(Node):
    def __init__(self):
        super().__init__('simplified_docking_node')
        
        # ========================================
        # 설정
        # ========================================
        self.target_distance = 2.0
        
        # 허용 오차
        self.tolerance_lateral = 0.15
        self.tolerance_dist = 0.08
        self.tolerance_yaw = 8.0
        
        # 제어 게인
        self.k_lateral = 1.5
        self.k_yaw = 0.8
        self.k_linear = 0.5
        
        self.max_angular = 0.6
        self.max_linear = 0.3
        
        # 진동 감지용
        self.yaw_history = []
        self.history_size = 10
        
        # ========================================
        # [추가] 마커 소실 복구용
        # ========================================
        self.last_angular_vel = 0.0   # 마지막 회전 속도
        self.lost_from_state = None   # 어느 상태에서 놓쳤는지
        # ========================================
        
        self.bridge = CvBridge()
        
        # Subscribe
        self.sub_markers = self.create_subscription(
            MarkerArray, '/vision/front_markers', self.marker_callback, 10
        )
        self.sub_image = self.create_subscription(
            Image, '/front_camera/rgb', self.image_callback, 10
        )
        
        # Publish
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 상태
        self.marker_data = None
        self.last_marker_time = self.get_clock().now()
        
        self.state = STATE_APPROACH
        self.docking_active = False
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        print("✅ Simplified Docking Started (Smart Recovery)")
        print(f"🎯 Tolerances: Lat={self.tolerance_lateral}m, Dist={self.tolerance_dist}m, Yaw={self.tolerance_yaw}°")

    def marker_callback(self, msg):
        """마커 데이터 (base_link 기준)"""
        if len(msg.markers) == 0:
            self.marker_data = None
            return
        
        marker = msg.markers[0]
        
        # base_link 기준 마커 좌표
        x = marker.pose.position.x
        y = marker.pose.position.y
        
        # Yaw 추출
        from scipy.spatial.transform import Rotation as R
        q = marker.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        yaw = rot.as_euler('xyz', degrees=True)[2]
        
        self.marker_data = (x, y, yaw)
        self.last_marker_time = self.get_clock().now()

    def image_callback(self, msg):
        """디버깅용 시각화"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return
        
        state_text = ["APPROACH", "ALIGN_YAW", "VERIFY", "SEARCH"][self.state]
        
        if self.marker_data:
            x, y, yaw = self.marker_data
            
            # 오차 계산
            lateral_error, dist_error, yaw_error_deg = self.calculate_errors()
            
            if lateral_error is not None:
                text = f"{state_text} | Lat:{lateral_error:+.2f}m | Dist:{dist_error:+.2f}m | Yaw:{yaw_error_deg:+.1f}deg"
                cv2.putText(frame, text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                text2 = f"Marker (base_link): X={x:.2f}m Y={y:.2f}m Yaw={yaw:.1f}deg"
                cv2.putText(frame, text2, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # 마지막 회전 방향 표시
                text3 = f"Last AngVel: {self.last_angular_vel:+.2f} rad/s"
                cv2.putText(frame, text3, (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            text = f"{state_text} | Waiting for marker..."
            cv2.putText(frame, text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        cv2.imshow("Simplified Docking", frame)
        cv2.waitKey(1)

    def calculate_errors(self):
        """base_link 기준 오차 계산"""
        if self.marker_data is None:
            return None, None, None
        
        x, y, yaw_deg = self.marker_data
        
        lateral_error = y
        dist_error = x - self.target_distance
        yaw_error_deg = yaw_deg
        
        return lateral_error, dist_error, yaw_error_deg

    def check_oscillation(self, yaw_error):
        """진동 감지"""
        self.yaw_history.append(yaw_error)
        if len(self.yaw_history) > self.history_size:
            self.yaw_history.pop(0)
        
        if len(self.yaw_history) < self.history_size:
            return False
        
        # 부호가 계속 바뀌면 진동
        sign_changes = 0
        for i in range(1, len(self.yaw_history)):
            if (self.yaw_history[i] * self.yaw_history[i-1]) < 0:
                sign_changes += 1
        
        return sign_changes >= 5

    def control_loop(self):
        # ========================================
        # 마커 소실 처리
        # ========================================
        time_since_marker = (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9
        
        if time_since_marker > 0.5:
            if self.state != STATE_SEARCH:
                # 어느 상태에서 놓쳤는지 기억
                self.lost_from_state = self.state
                print(f"⚠️ Marker Lost from {['APPROACH', 'ALIGN_YAW', 'VERIFY'][self.lost_from_state]}")
                print(f"   Last angular velocity: {self.last_angular_vel:+.2f} rad/s")
                self.state = STATE_SEARCH
            
            cmd = Twist()
            
            # ========================================
            # [핵심] 마지막 회전 방향의 반대로 회전
            # ========================================
            if abs(self.last_angular_vel) > 0.01:
                # 마지막 회전 속도가 있으면 반대로
                search_direction = -1.0 * math.copysign(1, self.last_angular_vel)
                cmd.angular.z = 0.5 * search_direction
                print(f"🔍 Reversing rotation: {cmd.angular.z:+.2f} rad/s (was {self.last_angular_vel:+.2f})")
            else:
                # 회전 없었으면 기본 회전
                cmd.angular.z = 0.4
                print(f"🔍 Default search rotation: {cmd.angular.z:+.2f} rad/s")
            
            self.pub_cmd.publish(cmd)
            return
        
        # 마커 재발견
        if self.state == STATE_SEARCH:
            print("👀 Marker Regained!")
            # 놓친 상태로 복귀
            if self.lost_from_state is not None:
                self.state = self.lost_from_state
                print(f"   Resuming {['APPROACH', 'ALIGN_YAW', 'VERIFY'][self.state]}")
            else:
                self.state = STATE_APPROACH
            self.lost_from_state = None
        
        if self.marker_data is None:
            self.pub_cmd.publish(Twist())
            return
        
        # 오차 계산
        lateral_error, dist_error, yaw_error_deg = self.calculate_errors()
        
        if lateral_error is None:
            self.pub_cmd.publish(Twist())
            return
        
        if not self.docking_active:
            print("🎯 Docking Started")
            self.docking_active = True
        
        cmd = Twist()
        
        # ========================================
        # 상태 머신
        # ========================================
        
        if self.state == STATE_APPROACH:
            # 1단계: 좌우 + 거리
            if abs(lateral_error) < self.tolerance_lateral and abs(dist_error) < self.tolerance_dist:
                print(f"✅ Position OK! Aligning Yaw...")
                self.yaw_history.clear()
                self.state = STATE_ALIGN_YAW
                self.pub_cmd.publish(Twist())
                return
            
            # 좌우 보정
            cmd.angular.z = -self.k_lateral * lateral_error
            cmd.angular.z = max(min(cmd.angular.z, self.max_angular), -self.max_angular)
            
            # 전진
            cmd.linear.x = self.k_linear * dist_error
            cmd.linear.x = max(min(cmd.linear.x, self.max_linear), 0.05)
            
            # ========================================
            # [중요] 현재 회전 속도 기억
            # ========================================
            self.last_angular_vel = cmd.angular.z
            
            print(f"🚀 Approach | Lat:{lateral_error:+.2f}m | Dist:{dist_error:+.2f}m | Yaw:{yaw_error_deg:+.1f}°")
        
        elif self.state == STATE_ALIGN_YAW:
            # 2단계: Yaw 정렬
            
            # 진동 감지
            if self.check_oscillation(yaw_error_deg):
                print(f"⚠️ Oscillation detected! Accepting Yaw={yaw_error_deg:.1f}°")
                self.state = STATE_VERIFY
                self.pub_cmd.publish(Twist())
                return
            
            # 성공 조건
            if abs(yaw_error_deg) < self.tolerance_yaw:
                print(f"✅ Yaw aligned! Verifying...")
                self.state = STATE_VERIFY
                self.pub_cmd.publish(Twist())
                return
            
            # 제자리 회전
            cmd.linear.x = 0.0
            cmd.angular.z = self.k_yaw * math.radians(yaw_error_deg)
            cmd.angular.z = max(min(cmd.angular.z, self.max_angular), -self.max_angular)
            
            # ========================================
            # [중요] 현재 회전 속도 기억
            # ========================================
            self.last_angular_vel = cmd.angular.z
            
            print(f"🔄 Aligning Yaw | Yaw:{yaw_error_deg:+.1f}° | AngVel:{cmd.angular.z:+.2f}")
        
        elif self.state == STATE_VERIFY:
            # 3단계: 최종 확인
            if (abs(lateral_error) < self.tolerance_lateral and 
                abs(dist_error) < self.tolerance_dist and 
                abs(yaw_error_deg) < self.tolerance_yaw):
                print(f"🏁 Docking Complete!")
                print(f"   Lat={lateral_error:+.2f}m | Dist={dist_error:+.2f}m | Yaw={yaw_error_deg:+.1f}°")
                self.pub_cmd.publish(Twist())
                self.docking_active = False
                return
            
            # 큰 오차만 재시작
            if abs(lateral_error) > 0.25 or abs(dist_error) > 0.15:
                print(f"⚠️ Large drift detected. Restarting...")
                self.state = STATE_APPROACH
            else:
                # 작은 오차는 무시하고 성공
                print(f"✅ Minor drift OK. Completing...")
                print(f"   Lat={lateral_error:+.2f}m | Dist={dist_error:+.2f}m | Yaw={yaw_error_deg:+.1f}°")
                self.pub_cmd.publish(Twist())
                self.docking_active = False
            return
        
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    node = SimplifiedDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()