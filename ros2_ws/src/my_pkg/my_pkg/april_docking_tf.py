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
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
    ALIGN_TO_GRID = 4
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
        
        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_yaw = 0.0
        
        # 재정렬 카운터 (최대 2번 재시도)
        self.realignment_count = 0
        self.verification_start_time = None
        # [추가] 정렬 시작 시간 (TF 안정화 대기용)
        self.align_start_time = None
        
        self.tf_update_timer = None  # TF 타이머 참조 저장
        
        # Subscribers/Publishers
        self.create_subscription(PoseStamped, 'detected_dock_pose', self.dock_pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Services
        self.create_service(Trigger, 'start_docking', self.start_docking_callback)
        self.create_service(Trigger, 'stop_docking', self.stop_docking_callback)
        
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('🎯 Simple Precision Docking Started (Optimized)')

    def get_robot_yaw_from_tf(self):
        try:            
            # 최신 TF를 기다림 (최대 0.1초)
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_frame, 
                rclpy.time.Time(),  # 최신 시간
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            return yaw, True
        except Exception as e:
            return 0.0, False

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
    
    def update_tf_yaw(self):
        """TF 기반 yaw 업데이트 (20Hz)"""
        # 검증 상태(VERIFY_ALIGNMENT) 추가 필수
        if self.state in [DockingState.ALIGN_TO_GRID, DockingState.VERIFY_ALIGNMENT, DockingState.DOCKED]:
            yaw, success = self.get_robot_yaw_from_tf()
            if success:
                self.current_yaw = yaw
                
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
        
        # 2. TF 업데이트 타이머 정리 (리소스 확보)
        if self.tf_update_timer is not None:
            self.tf_update_timer.cancel()
            self.tf_update_timer = None
            self.get_logger().info("⏸️  TF updates stopped (Docking Finished)")
            
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
            if pose_age > 0.2 and self.state not in [DockingState.IDLE, DockingState.ALIGN_TO_GRID, DockingState.VERIFY_ALIGNMENT, DockingState.DOCKED]:
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
        if self.state not in [DockingState.ALIGN_TO_GRID, DockingState.VERIFY_ALIGNMENT, DockingState.DOCKED, DockingState.IDLE]:
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
                
                if self.tf_update_timer is None:
                    # 반응성을 위해 20Hz(0.05s)로 주기 단축
                    self.tf_update_timer = self.create_timer(0.05, self.update_tf_yaw) 
                    self.get_logger().info("🔄 TF updates started (20Hz)")
                    
                # [수정] 상태 전환 시 시작 시간 기록
                self.align_start_time = self.get_clock().now()
                self.state = DockingState.ALIGN_TO_GRID
                self.get_logger().info(f"🎯 Distance Reached. Waiting for TF stabilization...")
                
                self.state = DockingState.ALIGN_TO_GRID
                self.get_logger().info(f"🎯 Distance Reached. Starting Grid Snap.")

        elif self.state == DockingState.ALIGN_TO_GRID:
            # TF 안정화 대기 (1.0초)
            time_since_start = (self.get_clock().now() - self.align_start_time).nanoseconds / 1e9
            if time_since_start < 1.0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f"⏳ Waiting for TF... ({time_since_start:.1f}/1.0s)", throttle_duration_sec=0.5)
                return
            
            target_yaw = round(self.current_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
            yaw_error = target_yaw - self.current_yaw
            while yaw_error > math.pi: yaw_error -= 2 * math.pi
            while yaw_error < -math.pi: yaw_error += 2 * math.pi
            
            self.get_logger().info(
                f"🧭 SNAPPING | Cur: {math.degrees(self.current_yaw):.1f}° -> Tgt: {math.degrees(target_yaw):.0f}° | Err: {math.degrees(yaw_error):.2f}°",
                throttle_duration_sec=0.2
            )
            
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
                    
                
                speed = np.clip(gain * yaw_error, -limit, limit)
                
                # 최소 회전 속도 보장 (Dead zone 극복)
                if abs(speed) < min_speed:
                    speed = min_speed if yaw_error > 0 else -min_speed
                
                cmd.linear.x = 0.0
                cmd.angular.z = speed
            
            else:
                # 정렬 오차 범위 진입 -> 즉시 정지 명령 후 검증 상태로 전환
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
                # 검증 시작 시간 기록
                self.verification_start_time = self.get_clock().now()
                self.state = DockingState.VERIFY_ALIGNMENT
                
                self.get_logger().info(f"⏸️ Alignment within tolerance. Stopping for verification... (Retry {self.realignment_count}/3)")

        # [신규 추가 코드] ALIGN_TO_GRID 블록과 DOCKED 블록 사이에 삽입
        elif self.state == DockingState.VERIFY_ALIGNMENT:
            # 1. 검증 중에는 무조건 정지 명령 유지 (확실한 정지)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            # 2. 충분한 안정화 시간 대기 (1.5초)
            wait_time = (self.get_clock().now() - self.verification_start_time).nanoseconds / 1e9
            
            if wait_time < 1.5:
                # 아직 대기 중이면 로그만 간헐적으로 출력하고 리턴 (cmd=0 전송됨)
                self.get_logger().info(f"⏳ Stabilizing... ({wait_time:.1f}/1.5s)", throttle_duration_sec=0.5)
            else:
                # 3. 대기 완료 -> 현재 각도 최종 확인
                # TF 업데이트는 백그라운드 타이머에서 self.current_yaw를 계속 갱신 중임
                target_yaw = round(self.current_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
                final_error = target_yaw - self.current_yaw
                while final_error > math.pi: final_error -= 2 * math.pi
                while final_error < -math.pi: final_error += 2 * math.pi
                
                # 4. 결과 판단
                if abs(final_error) > 0.02: # 여전히 1도 이상 틀어져 있음
                    if self.realignment_count < 3:
                        # 재시도 횟수 남아있으면 -> 다시 정렬 상태로 복귀
                        self.realignment_count += 1
                        self.state = DockingState.ALIGN_TO_GRID
                        self.get_logger().warn(
                            f"🔄 Drift detected! Error: {math.degrees(final_error):.2f}° -> Re-aligning (Retry {self.realignment_count}/3)"
                        )
                    else:
                        # [핵심 수정] 재시도 횟수 초과 시, 5도 이내인지 확인
                        final_deg_error = math.degrees(abs(final_error))
                        
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
                        f"✅ VERIFIED! Stable. Final Error: {math.degrees(final_error):.2f}°"
                    )
                    self._finish_docking(success=True)

        elif self.state == DockingState.DOCKED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # TF 타이머 중지
        if self.tf_update_timer is not None:
            self.tf_update_timer.cancel()
            self.tf_update_timer = None
        
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