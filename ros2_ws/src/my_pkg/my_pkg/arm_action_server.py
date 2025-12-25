import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
import time
import numpy as np

# 커스텀 액션 임포트
from moma_interfaces.action import MoveManipulator

class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')
        
        # ---------------------------------------------------------
        # 1. Publishers (Isaac Sim 통신용)
        # ---------------------------------------------------------
        # RMPFlow 좌표 제어용
        self.pose_pub = self.create_publisher(PoseStamped, '/rmp_target_pose', 10)
        # 관절 직접 제어용 (새로 추가됨)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_command', 10)
        # 그리퍼 제어용
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)

        # ---------------------------------------------------------
        # 2. Action Server 설정
        # ---------------------------------------------------------
        self._action_server = ActionServer(
            self,
            MoveManipulator,
            'move_manipulator',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # ---------------------------------------------------------
        # 3. 기본 설정값
        # ---------------------------------------------------------
        # Home Pose (Joint Angles in Radians)
        # 요청했던 [0, 0, 90, -90, -90, 90] 도 -> 라디안 변환
        self.home_joints = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, -1.5708]

        self.get_logger().info('✅ Arm Action Server Ready (Hybrid Control)')

    def execute_callback(self, goal_handle):
        action_type = goal_handle.request.action_type
        self.get_logger().info(f'📩 Action Request: {action_type}')
        
        feedback_msg = MoveManipulator.Feedback()
        result = MoveManipulator.Result()

        try:
            # =====================================================
            # Case A: Home (이동 전 안전 자세 - Joint Control)
            # =====================================================
            if action_type == 'home':
                self.publish_joint(self.home_joints)
                self.wait_for_execution(3.0, feedback_msg, goal_handle, "Moving to Home (Joints)")

            # =====================================================
            # Case B: Pick (물체 잡기 - RMPFlow + Gripper)
            # =====================================================
            elif action_type == 'pick':
                target_pose = goal_handle.request.target_pose
                
                # 1. 그리퍼 열기
                self.control_gripper("open")
                time.sleep(0.5)

                # 2. 접근 (Approach)
                self.publish_pose(target_pose)
                self.wait_for_execution(3.0, feedback_msg, goal_handle, "Approaching Target")
                
                # 3. 잡기 (Grasp)
                self.control_gripper("close")
                self.wait_for_execution(1.0, feedback_msg, goal_handle, "Grasping")
                
                # 4. 들어올리기 (Lift) - Z축 + 20cm
                lift_pose = target_pose
                lift_pose.pose.position.z += 0.2
                self.publish_pose(lift_pose)
                self.wait_for_execution(2.0, feedback_msg, goal_handle, "Lifting Object")

            # =====================================================
            # Case C: Place (물체 놓기)
            # =====================================================
            elif action_type == 'place':
                target_pose = goal_handle.request.target_pose
                
                # 1. 이동
                self.publish_pose(target_pose)
                self.wait_for_execution(3.0, feedback_msg, goal_handle, "Moving to Place")
                
                # 2. 놓기
                self.control_gripper("open")
                self.wait_for_execution(1.0, feedback_msg, goal_handle, "Releasing Object")

            # =====================================================
            # Case D: Custom Joint Move (임의 각도 이동)
            # =====================================================
            elif action_type == 'move_to_joint':
                joints = goal_handle.request.joint_angles
                if len(joints) == 6:
                    self.publish_joint(joints)
                    self.wait_for_execution(3.0, feedback_msg, goal_handle, "Moving Joints")
                else:
                    raise ValueError("Joint angles must be length 6")

            # 완료 처리
            goal_handle.succeed()
            result.success = True
            result.message = f"Action {action_type} completed."
            
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f"Failed: {str(e)}"
            self.get_logger().error(f"❌ Error: {str(e)}")

        return result

    # --- Helper Functions ---

    def publish_pose(self, pose_stamped):
        """RMPFlow 좌표 제어"""
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(pose_stamped)

    def publish_joint(self, joints):
        """관절 직접 제어"""
        msg = Float64MultiArray()
        msg.data = joints
        self.joint_pub.publish(msg)

    def control_gripper(self, command):
        """그리퍼 제어"""
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def wait_for_execution(self, duration, feedback, goal_handle, state_text):
        """단순 대기 (추후 Isaac Feedback 연동 시 수정 가능)"""
        feedback.current_state = state_text
        goal_handle.publish_feedback(feedback)
        time.sleep(duration)

def main(args=None):
    rclpy.init(args=args)
    server = ArmActionServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()