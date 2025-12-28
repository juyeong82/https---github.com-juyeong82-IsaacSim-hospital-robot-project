import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moma_interfaces.msg import MarkerArray
from std_msgs.msg import Bool
import math
import time

class VisualServoBase(Node):
    def __init__(self):
        super().__init__('visual_servo_base')

        # ---------------------------------------------------------
        # 1. 제어 파라미터 (튜닝)
        # ---------------------------------------------------------
        self.target_dist = 1.0  # 목표 거리 (m)
        self.dist_tolerance = 0.02 # 거리 허용 오차 (m)
        self.angle_tolerance = 0.02 # 각도 허용 오차 (rad)
        
        # PID 게인
        self.k_v = 0.5  # 속도 게인
        self.k_w = 0.8  # 회전 게인 (반응성을 위해 약간 낮춤)
        
        # 속도 제한
        self.max_v = 0.15 
        self.max_w = 0.3

        # 안전 장치: 데이터가 이 시간보다 오래되면 정지
        self.watchdog_timeout = 0.5 # 초

        # ---------------------------------------------------------
        # 2. 통신
        # ---------------------------------------------------------
        self.sub_markers = self.create_subscription(
            MarkerArray, '/vision/front_markers', self.marker_callback, 10
        )
        self.sub_enable = self.create_subscription(
            Bool, '/visual_servo/enable', self.enable_callback, 10
        )
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_result = self.create_publisher(Bool, '/visual_servo/done', 10)

        # ---------------------------------------------------------
        # 3. 상태 관리 (Timer 루프 방식)
        # ---------------------------------------------------------
        # 마커가 안 와도 주기적으로 판단하기 위해 타이머 사용
        self.create_timer(0.1, self.control_loop) 

        self.is_enabled = False
        self.last_marker_time = 0.0
        self.latest_marker = None
        
        self.get_logger().info("✅ Visual Servo V2 Ready (Watchdog & Log Added)")

    def enable_callback(self, msg):
        if msg.data:
            self.is_enabled = True
            self.get_logger().info("🟢 [START] Visual Servoing Enabled")
        else:
            self.is_enabled = False
            self.stop_robot()
            self.get_logger().info("🔴 [STOP] Visual Servoing Disabled")

    def marker_callback(self, msg):
        """데이터 수신 및 타임스탬프 갱신"""
        if len(msg.markers) > 0:
            self.latest_marker = msg.markers[0]
            self.last_marker_time = time.time()

    def control_loop(self):
        """0.1초마다 실행되는 메인 제어 루프"""
        if not self.is_enabled:
            return

        # 1. Watchdog: 데이터가 끊겼는지 확인
        time_diff = time.time() - self.last_marker_time
        
        if time_diff > self.watchdog_timeout:
            # [중요] 마커를 놓치면 즉시 정지
            self.get_logger().warn(f"⚠️ Marker Lost! (Last seen {time_diff:.1f}s ago) -> STOPPING")
            self.stop_robot()
            return

        # 2. 제어 로직 수행
        if self.latest_marker:
            self.process_servoing(self.latest_marker)

    def process_servoing(self, marker):
        # 좌표 추출 (Base Link 기준)
        # x: 전방 거리, y: 좌우 편차
        curr_x = marker.pose.position.x
        curr_y = marker.pose.position.y
        
        # 오차 계산
        error_dist = curr_x - self.target_dist
        error_angle = math.atan2(curr_y, curr_x)

        # --- 로그 출력 (상태 모니터링) ---
        # 현재 거리, 각도, 오차를 한눈에 보이게 출력
        log_msg = (
            f"👀 Marker at X:{curr_x:.3f}m | "
            f"Err_Dist: {error_dist:.3f}m | "
            f"Err_Ang: {error_angle:.3f}rad"
        )
        # -------------------------------

        # 완료 조건 확인
        if abs(error_dist) < self.dist_tolerance and abs(error_angle) < self.angle_tolerance:
            self.get_logger().info(f"✅ Success! Reached Target. ({log_msg})")
            self.stop_robot()
            self.pub_result.publish(Bool(data=True))
            self.is_enabled = False # 작업 종료
            return

        # P 제어 계산
        v_cmd = self.k_v * error_dist
        w_cmd = self.k_w * error_angle

        # 속도 제한 (Clamping)
        v_cmd = max(min(v_cmd, self.max_v), -self.max_v)
        w_cmd = max(min(w_cmd, self.max_w), -self.max_w)
        
        # [특수 조건] 각도가 너무 틀어졌으면 제자리 회전만 수행
        if abs(error_angle) > 0.15: # 약 8.5도
            v_cmd = 0.0
            log_msg += " [Rotating First]"
        
        # [특수 조건] 너무 가까우면(30cm 이내) 후진 허용하되 천천히
        if curr_x < 0.3:
            log_msg += " [Too Close! Backing up]"
            # 후진은 그대로 v_cmd가 음수가 되므로 자동 처리됨

        self.get_logger().info(log_msg) # 로그 출력

        # 명령 전송
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_vel.publish(cmd)

    def stop_robot(self):
        cmd = Twist() # 0,0,0
        self.pub_vel.publish(cmd)

def main():
    rclpy.init()
    node = VisualServoBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()