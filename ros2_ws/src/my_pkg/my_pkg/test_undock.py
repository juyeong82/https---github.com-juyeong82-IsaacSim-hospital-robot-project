#!/usr/bin/env python3
"""
언도킹 테스트 - 도킹 노드와 동일한 타이머 구조
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import math
import numpy as np
from enum import Enum

class UndockState(Enum):
    IDLE = 0
    ROTATE = 1
    REVERSE = 2
    DONE = 3

class UndockTester(Node):
    def __init__(self):
        super().__init__('undock_tester')
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_dock_trigger = self.create_publisher(Bool, '/docking/trigger', 10)
        
        self.latest_dock_pose = None
        self.create_subscription(
            PoseStamped, 'detected_dock_pose', 
            self.dock_pose_callback, 10,
            callback_group=self.cb_group
        )
        
        # 상태 변수
        self.state = UndockState.IDLE
        self.target_yaw_rad = 0.0
        self.reverse_dist = 0.6
        self.start_dist = None
        
        # 타이머 (도킹 노드와 동일하게 0.05초)
        self.create_timer(0.05, self.control_loop, callback_group=self.cb_group)
        
        self.get_logger().info("🧪 UNDOCK TESTER READY (Timer-based)")
        
    def dock_pose_callback(self, msg):
        self.latest_dock_pose = msg

    def start_undock(self, approach_side="Left", reverse_dist=0.6):
        """외부에서 호출하여 언도킹 시작"""
        self.reverse_dist = reverse_dist
        
        # 목표 각도 설정
        target_yaw_deg = -10.0 if approach_side == "Left" else 10.0
        self.target_yaw_rad = math.radians(target_yaw_deg)
        
        self.get_logger().info(f"🔙 Starting Undock: Side={approach_side}, Target={target_yaw_deg}°")
        
        # 트리거 ON
        for _ in range(5):
            self.pub_dock_trigger.publish(Bool(data=True))
        
        self.state = UndockState.ROTATE
        self.start_dist = None

    def control_loop(self):
        """도킹 노드의 control_loop와 동일한 구조"""
        
        if self.state == UndockState.IDLE:
            return
        
        if self.state == UndockState.DONE:
            return
        
        # 마커 체크
        if self.latest_dock_pose is None:
            self.get_logger().warn("⚠️ No marker!", throttle_duration_sec=1.0)
            self.pub_cmd_vel.publish(Twist())
            return
        
        # bearing 계산 (도킹 노드와 동일)
        lateral = -self.latest_dock_pose.pose.position.x
        distance = self.latest_dock_pose.pose.position.z
        bearing_angle = np.arctan2(lateral, distance)
        
        cmd = Twist()
        
        # ========== ROTATE 상태 ==========
        if self.state == UndockState.ROTATE:
            yaw_error = self.target_yaw_rad - bearing_angle
            
            self.get_logger().info(
                f"🔄 ROTATE | Target:{math.degrees(self.target_yaw_rad):+.1f}° "
                f"Current:{math.degrees(bearing_angle):+.1f}° Error:{math.degrees(yaw_error):+.1f}°",
                throttle_duration_sec=0.2
            )
            
            # 완료 체크 (2도 이내)
            if abs(yaw_error) < math.radians(2.0):
                self.get_logger().info(f"✅ ROTATION DONE! Final:{math.degrees(bearing_angle):+.1f}°")
                self.state = UndockState.REVERSE
                self.start_dist = distance
                return
            
            # 회전 제어 (도킹 노드의 ROTATE_TO_TARGET과 유사하지만 목표가 다름)
            # 도킹: bearing → 0 수렴 → cmd.angular.z = K * bearing
            # 언도킹: bearing → target 수렴 → cmd.angular.z = -K * (target - bearing)
            angular_speed = np.clip(-3.0 * yaw_error, -0.3, 0.3)
            
            # 최소 속도 보장
            if 0 < abs(angular_speed) < 0.05:
                angular_speed = 0.05 if angular_speed > 0 else -0.05
            
            cmd.angular.z = angular_speed
        
        # ========== REVERSE 상태 ==========
        elif self.state == UndockState.REVERSE:
            if self.start_dist is None:
                self.start_dist = distance
            
            target_dist = self.start_dist + self.reverse_dist
            
            self.get_logger().info(
                f"🔙 REVERSE | Dist:{distance:.2f}m / Target:{target_dist:.2f}m",
                throttle_duration_sec=0.3
            )
            
            if distance >= target_dist:
                self.get_logger().info(f"✅ REVERSE DONE! Final:{distance:.2f}m")
                self.finish_undock()
                return
            
            # 순수 후진
            cmd.linear.x = -0.15
            cmd.angular.z = 0.0
        
        self.pub_cmd_vel.publish(cmd)

    def finish_undock(self):
        self.pub_cmd_vel.publish(Twist())
        for _ in range(3):
            self.pub_dock_trigger.publish(Bool(data=False))
        self.state = UndockState.DONE
        self.get_logger().info("🏁 UNDOCK COMPLETE!")


def main():
    rclpy.init()
    node = UndockTester()
    
    # MultiThreadedExecutor (도킹 노드와 동일)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    print("\n" + "="*40)
    print("🧪 UNDOCK TESTER (Timer-based)")
    print("="*40)
    
    side = input("Side [L]eft/[R]ight (L): ").strip().upper()
    approach_side = "Right" if side == "R" else "Left"
    
    dist = input("Reverse dist (0.6): ").strip()
    reverse_dist = float(dist) if dist else 0.6
    
    input(f"\n▶ Side={approach_side}, Dist={reverse_dist}m\nENTER to start...")
    
    # 언도킹 시작
    node.start_undock(approach_side, reverse_dist)
    
    try:
        # 완료될 때까지 spin
        while rclpy.ok() and node.state != UndockState.DONE:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.pub_cmd_vel.publish(Twist())
        print("\n🛑 Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()