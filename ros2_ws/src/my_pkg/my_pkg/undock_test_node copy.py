#!/usr/bin/env python3
"""
Undock Test Node (Active Correction during Reverse)
- Phase 1: 제자리 회전으로 목표 각도(Orientation) 맞춤
- Phase 2: 후진하면서도 해당 각도를 유지하도록 조향 보정 (Swivel Caster Drift 방지)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import math
import time
import numpy as np
import threading
import sys
import select
import termios
import tty

# [설정]
TEST_REVERSE_DIST = 3.0   # 후진 거리 (m)
TARGET_ANGLE_DEG = 10.0   # 목표 회전 각도 (도)
P_GAIN = 4.0              # 회전 속도 게인
MAX_ROT_SPEED = 0.5       
MIN_ROT_SPEED = 0.1       
REVERSE_SPEED = -0.15     # 후진 속도 (조향과 동시에 하므로 약간 낮춤)

# [변환 함수]
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class UndockTestNode(Node):
    def __init__(self):
        super().__init__('undock_test_node')
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_dock_trigger = self.create_publisher(Bool, '/docking/trigger', 10)
        
        self.latest_dock_pose = None
        self.latest_pose_time = self.get_clock().now()
        
        self.create_subscription(PoseStamped, 'detected_dock_pose', self.dock_pose_callback, 10)
        
        self.get_logger().info("✅ Undock Test Node Ready (Active Correction Mode).")
        self.get_logger().info("⚠️ Controls based on Marker ORIENTATION")
        self.get_logger().info("⌨️  Press 'L' for Left Approach (Target > +10°)")
        self.get_logger().info("⌨️  Press 'R' for Right Approach (Target < -10°)")
        self.get_logger().info("⌨️  Press 'Q' to Quit")

    def dock_pose_callback(self, msg):
        self.latest_dock_pose = msg
        self.latest_pose_time = self.get_clock().now()

    def get_marker_orientation_yaw(self):
        """
        마커의 Position이 아닌 Orientation(자세)을 기반으로 Yaw 계산
        """
        if self.latest_dock_pose:
            q = self.latest_dock_pose.pose.orientation
            if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
                return None
            
            _, yaw, _ = euler_from_quaternion(q.x, q.y, q.z, q.w)
            return yaw
        return None

    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())

    def run_undock_test(self, side):
        self.get_logger().info(f"\n🚀 Starting Undock Test: {side} Side Approach")
        
        # 1. 마커 인식 켜기
        for _ in range(3):
            self.pub_dock_trigger.publish(Bool(data=True))
            time.sleep(0.05)
        time.sleep(1.0) 

        # ------------------------------------------------------------------
        # [목표 설정]
        # ------------------------------------------------------------------
        target_angle_rad = math.radians(TARGET_ANGLE_DEG)
        
        if side == 'L':
            # Left Approach -> 로봇 우회전 필요 -> 마커 기준 각도(Yaw) 감소(또는 증가, 좌표계 따름)
            # 기존 로직: Target < -10도 (또는 +10도) 
            # (사용자 기존 코드 기준: L -> Target +10도 이하로 내려가거나 올라가는 로직)
            # 여기서는 원본 코드의 로직(L -> current <= target)을 따름
            target_yaw = -target_angle_rad 
            self.get_logger().info(f"🎯 Goal: Marker Orientation <= {math.degrees(target_yaw):.1f}°")
        else:
            target_yaw = target_angle_rad
            self.get_logger().info(f"🎯 Goal: Marker Orientation >= {math.degrees(target_yaw):.1f}°")

        # === Phase 1: 제자리 회전 (Orientation P-Control) ===
        # 이 부분은 요청하신 대로 건드리지 않음
        # === Phase 1: 제자리 회전 (Orientation P-Control) ===
        start_time = self.get_clock().now()
        rate = self.create_rate(20)

        while rclpy.ok():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 60.0:
                self.get_logger().warn("⏰ Rotation Timeout!")
                break

            if (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 0.5:
                self.stop_robot()
                continue
            
            current_yaw = self.get_marker_orientation_yaw()
            if current_yaw is None: continue

            current_deg = math.degrees(current_yaw)
            target_deg = math.degrees(target_yaw)

            # 종료 조건 체크
            done = False
            if side == 'L': 
                if current_yaw <= target_yaw: done = True 
            else:           
                if current_yaw >= target_yaw: done = True 
            
            if done:
                self.get_logger().info(f"✅ Rotation Done! (Cur: {current_deg:.2f}°)")
                break

            # 제어 로직
            error = current_yaw - target_yaw
            speed = np.clip(-P_GAIN * error, -MAX_ROT_SPEED, MAX_ROT_SPEED)
            
            # 최소 속도 보장
            if abs(speed) < MIN_ROT_SPEED:
                speed = MIN_ROT_SPEED if speed > 0 else -MIN_ROT_SPEED

            cmd = Twist()
            cmd.angular.z = speed
            self.pub_cmd_vel.publish(cmd)
            
            # [수정된 부분] Phase 1 변수에 맞춰 로그 출력
            if self.latest_dock_pose:
                curr_dist = self.latest_dock_pose.pose.position.z  # 여기서 거리 계산
                
                self.get_logger().info(
                    # ang_speed -> speed 로 변경
                    f"🔄 Rot | Dist: {curr_dist:.2f}m | Orient: {current_deg:.2f}° -> Goal: {target_deg:.1f}° | Cmd: {speed:.2f}", 
                    throttle_duration_sec=0.2
                )
            
            rate.sleep()

        self.stop_robot()
        time.sleep(0.5)

        # === Phase 2: 후진하면서 자세 유지 (Active Correction) ===
        self.get_logger().info("🔙 Reversing with Active Yaw Correction...")
        
        # 데이터 대기
        while (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 0.5:
            time.sleep(0.1)

        start_dist = self.latest_dock_pose.pose.position.z
        target_dist = start_dist + TEST_REVERSE_DIST
        
        rev_start = self.get_clock().now()
        
        while rclpy.ok():
            # 타임아웃
            if (self.get_clock().now() - rev_start).nanoseconds / 1e9 > 60.0:
                self.get_logger().warn("⏰ Reverse Timeout!")
                break
            
            # 마커 놓침 체크
            if (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 0.5:
                # 마커 놓치면 위험하므로 정지 (혹은 직진만 할 수도 있으나 안전상 정지 권장)
                self.stop_robot() 
                self.get_logger().warn("⚠️ Marker lost during reverse. Stopping.")
                continue
                
            curr_dist = self.latest_dock_pose.pose.position.z
            current_yaw = self.get_marker_orientation_yaw()
            
            # 거리 도달 체크
            if curr_dist >= target_dist:
                self.get_logger().info(f"✅ Distance Reached: {curr_dist:.2f}m")
                break
            
            # [핵심 변경 사항] 후진 중 각도 보정 로직 (Phase 1과 동일 로직 적용)
            ang_speed = 0.0
            if current_yaw is not None:
                # 목표 각도(target_yaw)를 계속 유지하도록 오차 계산
                error = current_yaw - target_yaw
                
                # P-Control 적용 (Phase 1과 동일한 게인 사용)
                # 후진 중이므로 급격한 회전은 지양 -> MAX_ROT_SPEED 대신 조금 더 제한 가능
                # 여기서는 동일하게 적용하되 클리핑을 타이트하게 잡을 수도 있음
                ang_speed = np.clip(-3.0 * error, -0.3, 0.3)
                
                # 후진 중 미세 진동 방지를 위해 오차가 매우 작으면 무시 (Deadzone)
                if abs(error) < math.radians(1.0):
                    ang_speed = 0.0

            cmd = Twist()
            cmd.linear.x = REVERSE_SPEED  # -0.15 m/s
            cmd.angular.z = ang_speed     # 각도 보정값
            
            self.pub_cmd_vel.publish(cmd)
            
            self.get_logger().info(
                f"🔙 Rev | Dist: {curr_dist:.2f}m | YawErr: {math.degrees(current_yaw - target_yaw):.1f}° | AngZ: {ang_speed:.2f}", 
                throttle_duration_sec=0.2
            )
            
            rate.sleep()

        self.stop_robot()
        for _ in range(3):
            self.pub_dock_trigger.publish(Bool(data=False))
            time.sleep(0.05)
            
        self.get_logger().info("🏁 Test Complete.\n")

# --- Helper ---
def get_key():
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = UndockTestNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok():
            key = get_key()
            if key in ['l', 'L']:
                node.run_undock_test('L')
            elif key in ['r', 'R']:
                node.run_undock_test('R')
            elif key in ['q', 'Q']:
                print("Quitting...")
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()