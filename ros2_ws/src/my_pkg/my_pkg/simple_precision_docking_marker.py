#!/usr/bin/env python3
"""
Simple Precision Docking Controller (Optimized)
- 정밀 회전 시 부드러운 감속 로직 추가
- 도킹 완료 후 자동 종료 기능 추가
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger
import numpy as np
import math
from enum import Enum

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

class DockingState(Enum):
    IDLE = 0
    ROTATE_TO_TARGET = 1    
    APPROACH = 2            
    FINAL_ALIGN = 3         
    ALIGN_TO_MARKER = 4
    VERIFY_ALIGNMENT = 5  
    DOCKED = 6        

class SimplePrecisionDocking(Node):
    def __init__(self):
        super().__init__('simple_precision_docking')
        
        # Parameters
        self.declare_parameter('docking_distance_threshold', 2.0)
        self.declare_parameter('rotation_threshold', 0.087)
        self.declare_parameter('approach_speed', 0.4)
        self.declare_parameter('rotation_speed', 0.5)
        self.declare_parameter('final_speed', 0.15)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        
        self.docking_threshold = self.get_parameter('docking_distance_threshold').value
        self.rotation_threshold = self.get_parameter('rotation_threshold').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.final_speed = self.get_parameter('final_speed').value
        self.auto_start = self.get_parameter('auto_start').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # State variables
        self.state = DockingState.IDLE
        self.latest_dock_pose = None
        self.latest_pose_time = None
        self.docking_enabled = self.auto_start
        
        # [추가] Yaw 필터링을 위한 변수 (EMA 필터)
        self.filtered_yaw = None
        self.alpha = 0.6  # 0.0~1.0 사이. 클수록 최신값 반영 비율 높음 (반응성 좋음)
        
        # [추가] 정렬 중 마커 놓침 방지용 카운터
        self.marker_lost_count = 0
        
        # 재정렬 카운터 (최대 2번 재시도)
        self.realignment_count = 0
        self.verification_start_time = None
        self.align_start_time = None
        
        # Subscribers/Publishers
        self.create_subscription(PoseStamped, 'detected_dock_pose', self.dock_pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Services
        self.create_service(Trigger, 'start_docking', self.start_docking_callback)
        self.create_service(Trigger, 'stop_docking', self.stop_docking_callback)
        
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('🎯 Simple Precision Docking Started (Optimized)')

    def start_docking_callback(self, request, response):
        self.docking_enabled = True
        self.state = DockingState.IDLE
        
        self.realignment_count = 0
        self.verification_start_time = None
        
        response.success = True
        response.message = "Docking enabled"
        return response
        
    def stop_docking_callback(self, request, response):
        self.docking_enabled = False
        self.stop_robot()
        response.success = True
        response.message = "Docking stopped"
        return response
        
    def dock_pose_callback(self, msg):
        self.latest_dock_pose = msg
        self.latest_pose_time = self.get_clock().now()
        
        # IDLE 상태에서만 자동 시작 체크
        if self.docking_enabled and self.state == DockingState.IDLE:
            distance = msg.pose.position.z
            if distance > 0.5:
                self.state = DockingState.ROTATE_TO_TARGET
                self.get_logger().info(f'🚀 Auto-start Triggered! (Dist={distance:.2f}m)')
                
    def _finish_docking(self, success):
        """
        도킹 프로세스 종료 처리 헬퍼 메서드
        - success: True(성공), False(실패/포기)
        """
        # 1. 로봇 즉시 정지
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        # 3. 도킹 활성화 플래그 끄기
        self.docking_enabled = False
        
        # 4. 상태 전환
        # 성공이든 실패든 프로세스가 끝났으므로 DOCKED 상태로 전환하여 IDLE 자동 시작 방지
        # (실패 시 IDLE로 보내면 마커 인식되자마자 다시 시작될 위험 있음)
        self.state = DockingState.DOCKED
        
        if success:
            self.get_logger().info("🏁 Docking Sequence Completed Successfully.")
        else:
            self.get_logger().warn("🛑 Docking Sequence Ended (Failed or Cancelled).")
    
    def control_loop(self):
        if not self.docking_enabled:
            return
        
        # 데이터 신선도 체크: 0.2초 이상 된 데이터는 '과거 정보'로 간주
        if self.latest_pose_time is not None:
            pose_age = (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9
            # 데이터가 오래된 경우 로봇을 멈추되, 상태(State)는 유지하여 다음 데이터를 기다림
            if pose_age > 0.2 and self.state not in [DockingState.IDLE, DockingState.ALIGN_TO_MARKER, DockingState.VERIFY_ALIGNMENT, DockingState.DOCKED]:
                self.get_logger().warn(f"⌛ Stale data ({pose_age:.2f}s)! Holding position...", throttle_duration_sec=1.0)
                hold_cmd = Twist()
                hold_cmd.linear.x = 0.0
                hold_cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(hold_cmd)
                return

        if self.state == DockingState.IDLE:
            self.get_logger().info("💤 IDLE: Waiting for marker...", throttle_duration_sec=2.0)
            return

        # Marker 기반 데이터 계산
        if self.state not in [DockingState.ALIGN_TO_MARKER, DockingState.VERIFY_ALIGNMENT, DockingState.DOCKED, DockingState.IDLE]:
            if self.latest_dock_pose is None: return
            
            # Marker Loss 체크 (1초)
            if (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 1.0:
                self.get_logger().warn('⚠️ Marker lost - STOPPING!')
                hold_cmd = Twist()
                hold_cmd.linear.x = 0.0
                hold_cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(hold_cmd)
                return

            lateral = -self.latest_dock_pose.pose.position.x
            distance = self.latest_dock_pose.pose.position.z
            bearing_angle = np.arctan2(lateral, distance)
        
        cmd = Twist()
        
        if self.state == DockingState.ROTATE_TO_TARGET:
            self.get_logger().info(
                f"🔄 ROTATING | Cur: {math.degrees(bearing_angle):.1f}° / Thresh: {math.degrees(self.rotation_threshold):.1f}°", 
                throttle_duration_sec=0.5
            )
            
            if abs(bearing_angle) > self.rotation_threshold:
                cmd.angular.z = np.clip(3.0 * bearing_angle, -self.rotation_speed, self.rotation_speed)
            else:
                self.state = DockingState.APPROACH
                self.get_logger().info("✅ Rotation aligned. Moving to APPROACH.")
                
        elif self.state == DockingState.APPROACH:
            self.get_logger().info(
                f"➡️ APPROACH | Dist: {distance:.2f}m | Drift: {math.degrees(bearing_angle):.1f}°", 
                throttle_duration_sec=0.5
            )
            
            if abs(bearing_angle) > 0.25: # 약 14도 이상 틀어지면 다시 회전
                self.state = DockingState.ROTATE_TO_TARGET
                return
            
            if distance > (self.docking_threshold + 0.5):
                cmd.linear.x = self.approach_speed
                cmd.angular.z = np.clip(4.0 * bearing_angle, -0.6, 0.6)
            else:
                self.state = DockingState.FINAL_ALIGN
                
        elif self.state == DockingState.FINAL_ALIGN:
            if distance > self.docking_threshold:
                cmd.linear.x = self.final_speed
                cmd.angular.z = np.clip(2.5 * bearing_angle, -0.2, 0.2)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                    
                # 상태 전환 시 시작 시간 기록
                self.align_start_time = self.get_clock().now()
                self.state = DockingState.ALIGN_TO_MARKER
                self.get_logger().info(f"🎯 Distance Reached. Starting Grid Snap.")

        elif self.state == DockingState.ALIGN_TO_MARKER:
            if self.latest_dock_pose is None:
                return

            # 마커 데이터 유효성 검사
            q = self.latest_dock_pose.pose.orientation
            if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
                self.get_logger().warn("⚠️ Invalid Quaternion Detected!")
                return
            
            # 오일러 변환 (roll, pitch, yaw)
            # OpenCV 좌표계(Z전방, X우측, Y하방) 기준, Y축 회전이 로봇의 Yaw 편차임
            _, current_marker_yaw, _ = euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # EMA 필터 적용 (노이즈/튀는 값 억제)
            if self.filtered_yaw is None:
                self.filtered_yaw = current_marker_yaw
            else:
                self.filtered_yaw = (self.alpha * current_marker_yaw) + ((1 - self.alpha) * self.filtered_yaw)
            
            # 제어에는 필터된 값 사용
            yaw_error = self.filtered_yaw
            
            self.get_logger().info(
                f"📐 ALIGNING | Marker Yaw: {math.degrees(yaw_error):.2f}°",
                throttle_duration_sec=0.2
            )
            
            # 허용 오차 (약 1.5도)
            if abs(yaw_error) > 0.02:  # 약 1도                    
                # ============ 재정렬 시 속도 감소 ============
                if self.realignment_count > 0:
                    # 재정렬 중: 더 느리고 부드럽게
                    if abs(yaw_error) > 0.05:
                        gain = 4.0  
                        limit = 0.15  
                    else:
                        gain = 3.0   
                        limit = 0.15  
                    min_speed = 0.02  
                else:
                    # 첫 정렬: 기존 속도
                    # 1. 오차가 큰 경우 (예: 2.8도/0.05rad 이상): 강한 P-제어
                    if abs(yaw_error) > 0.05:  # 5.7도 이상
                        gain = 8.0
                        limit = 0.3
                        
                    # 2. 중간 오차 (예: 1.0도/0.017rad ~ 2.8도 사이): 부드러운 감속 제어
                    else:
                        gain = 4.0
                        limit = 0.15
                    min_speed = 0.03
                    
                
                speed = -np.clip(gain * yaw_error, -limit, limit)
                
                # 최소 회전 속도 보장 (Dead zone 극복)
                if abs(speed) < min_speed:
                    speed = min_speed if yaw_error > 0 else -min_speed
                
                cmd.linear.x = 0.0
                cmd.angular.z = speed
            else:
                # 정렬 완료 -> 검증 단계로
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
                self.filtered_yaw = None
                
                self.verification_start_time = self.get_clock().now()
                self.state = DockingState.VERIFY_ALIGNMENT
                self.get_logger().info("⏸️ Marker Alignment Done. Verifying...")

        elif self.state == DockingState.VERIFY_ALIGNMENT:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            if self.latest_dock_pose is not None:
                q = self.latest_dock_pose.pose.orientation
                _, current_yaw, _ = euler_from_quaternion(q.x, q.y, q.z, q.w)
                
                if self.filtered_yaw is None:
                    self.filtered_yaw = current_yaw
                else:
                    self.filtered_yaw = (self.alpha * current_yaw) + ((1 - self.alpha) * self.filtered_yaw)
            
            wait_time = (self.get_clock().now() - self.verification_start_time).nanoseconds / 1e9
            
            if wait_time < 0.5:
                self.get_logger().info(f"⏳ Stabilizing... ({wait_time:.1f}/0.5s)", throttle_duration_sec=0.5)
            else:
                final_yaw_rad = self.filtered_yaw
                
                # 2. 판단을 위해 도로 변환
                final_deg_error = math.degrees(abs(final_yaw_rad))
                
                TARGET_TOLERANCE_DEG = 1.2 
                
                if final_deg_error > TARGET_TOLERANCE_DEG:
                    if self.realignment_count < 3:
                        # 재시도 횟수 남아있으면 -> 다시 정렬 상태로 복귀
                        self.realignment_count += 1
                        self.state = DockingState.ALIGN_TO_MARKER
                        self.get_logger().warn(
                            f"🔄 Drift detected! Error: {math.degrees(final_deg_error):.2f}° -> Re-aligning (Retry {self.realignment_count}/3)"
                        )
                    else:                       
                        if final_deg_error <= 5.0:
                            # Case A: 5도 이내 -> 허용 범위 성공 처리
                            self.get_logger().warn(
                                f"⚠️ Alignment acceptable (Retries exhausted). Final Error: {final_deg_error:.2f}° (Target < 5.0°)"
                            )
                            self._finish_docking(success=True) # 성공으로 처리하여 종료
                        else:
                            # Case B: 5도 초과 -> 실제 실패
                            self.get_logger().error(
                                f"❌ Alignment Failed. Deviation too large. Final Error: {final_deg_error:.2f}°"
                            )
                            self._finish_docking(success=False) # 실패 처리
                else:
                    # 정렬 성공
                    self.get_logger().info(
                        f"✅ VERIFIED! Stable. Final Error: {final_deg_error:.2f}°"
                    )
                    self._finish_docking(success=True)
                    
        elif self.state == DockingState.DOCKED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        self.state = DockingState.IDLE
        
        # 재시도 카운터 초기화
        self.realignment_count = 0
        self.verification_start_time = None
        
        self.get_logger().info("🛑 Robot Stopped and Controller Reset to IDLE")

def main(args=None):
    rclpy.init(args=args)
    node = SimplePrecisionDocking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()