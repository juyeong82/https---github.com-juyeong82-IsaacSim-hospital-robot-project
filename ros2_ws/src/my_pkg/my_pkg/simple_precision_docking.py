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
    DOCKED = 5

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
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            return yaw, True
        except Exception as e:
            return 0.0, False

    def start_docking_callback(self, request, response):
        self.docking_enabled = True
        self.state = DockingState.IDLE
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
        
    def control_loop(self):
        if not self.docking_enabled:
            return
        
        if self.state == DockingState.IDLE:
            self.get_logger().info("💤 IDLE: Waiting for marker...", throttle_duration_sec=2.0)
            return

        # TF 기반 Yaw 업데이트 (필요한 상태에서만)
        if self.state in [DockingState.ALIGN_TO_GRID, DockingState.DOCKED]:
            yaw, success = self.get_robot_yaw_from_tf()
            if success: self.current_yaw = yaw

        # Marker 기반 데이터 계산
        if self.state not in [DockingState.ALIGN_TO_GRID, DockingState.DOCKED, DockingState.IDLE]:
            if self.latest_dock_pose is None: return
            
            # Marker Loss 체크 (1초)
            if (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 1.0:
                self.get_logger().warn('⚠️ Marker lost - STOPPING!')
                self.stop_robot()
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
                cmd.angular.z = np.clip(2.5 * bearing_angle, -self.rotation_speed, self.rotation_speed)
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
                cmd.angular.z = np.clip(3.0 * bearing_angle, -0.2, 0.2)
            else:
                # 여기서 stop_robot()을 호출하면 IDLE로 가버리므로 속도만 0으로 설정
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.state = DockingState.ALIGN_TO_GRID
                self.get_logger().info(f"🎯 Distance Reached. Starting Grid Snap.")

        elif self.state == DockingState.ALIGN_TO_GRID:
            target_yaw = round(self.current_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
            yaw_error = target_yaw - self.current_yaw
            while yaw_error > math.pi: yaw_error -= 2 * math.pi
            while yaw_error < -math.pi: yaw_error += 2 * math.pi
            
            self.get_logger().info(
                f"🧭 SNAPPING | Cur: {math.degrees(self.current_yaw):.1f}° -> Tgt: {math.degrees(target_yaw):.0f}° | Err: {math.degrees(yaw_error):.2f}°",
                throttle_duration_sec=0.2
            )
            
            # 진동 방지를 위한 정밀 감속 P-제어
            if abs(yaw_error) > 0.017:  # 약 1.0도 임계값
                # 오차가 클수록 빠르고, 작을수록 아주 느리게 (최소 0.05 rad/s 보장)
                speed = np.clip(4.0 * yaw_error, -0.3, 0.3)
                if abs(speed) < 0.05: speed = 0.05 if yaw_error > 0 else -0.05
                cmd.angular.z = speed
            else:
                # 도킹 완료 및 "완전 종료"
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.state = DockingState.DOCKED
                self.docking_enabled = False # 프로세스 자동 중단
                self.get_logger().info(f"🎉 DOCKED & FINISHED at Yaw: {math.degrees(self.current_yaw):.1f}°")

        elif self.state == DockingState.DOCKED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.state = DockingState.IDLE
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