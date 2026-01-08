import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor # [핵심] 멀티스레드

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
from moma_interfaces.msg import MarkerArray

from tf2_ros import Buffer, TransformListener
import time
import math
import copy

from moma_interfaces.action import MoveManipulator

from scipy.spatial.transform import Rotation  # 이미 있음

class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')
        
        # 콜백 그룹 설정 (병렬 처리를 위해 Reentrant 사용)
        self.callback_group = ReentrantCallbackGroup()

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/rmp_target_pose', 10, callback_group=self.callback_group)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_command', 10, callback_group=self.callback_group)
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10, callback_group=self.callback_group)
        
        # [수정 1] 마커 저장 변수 분리
        self.left_markers = []
        self.right_markers = []
        
        # [수정 2] 콜백 함수 분리하여 구독
        self.create_subscription(MarkerArray, '/vision/left_markers', self.vision_callback_left, 10, callback_group=self.callback_group)
        self.create_subscription(MarkerArray, '/vision/right_markers', self.vision_callback_right, 10, callback_group=self.callback_group)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Server
        self._action_server = ActionServer(
            self,
            MoveManipulator,
            'move_manipulator',
            self.execute_callback,
            callback_group=self.callback_group # [핵심] 액션도 병렬 처리 그룹에 포함
        )
        
        self.home_joints = [0.0, -1.5708, -1.5708, -1.5708, 1.5708, 0.0]
        
        # Left Verify Pose (Target Y > 0 일 때 사용)
        self.verify_pose_left = PoseStamped()
        self.verify_pose_left.header.frame_id = "base_link"
        self.verify_pose_left.pose.position.x = -0.4
        self.verify_pose_left.pose.position.y = 0.7  # 좌측
        self.verify_pose_left.pose.position.z = 1.1
        self.verify_pose_left.pose.orientation.w = 0.707
        self.verify_pose_left.pose.orientation.y = 0.707
        self.verify_pose_left.pose.orientation.x = 0.0
        self.verify_pose_left.pose.orientation.z = 0.0
        
        # Right Verify Pose (Target Y < 0 일 때 사용)
        self.verify_pose_right = PoseStamped()
        self.verify_pose_right.header.frame_id = "base_link"
        self.verify_pose_right.pose.position.x = -0.4
        self.verify_pose_right.pose.position.y = -0.7 # 우측
        self.verify_pose_right.pose.position.z = 1.1
        self.verify_pose_right.pose.orientation.x = -0.707
        self.verify_pose_right.pose.orientation.y = 0.0
        self.verify_pose_right.pose.orientation.z = 0.707
        self.verify_pose_right.pose.orientation.w = 0.0
        
        # 현재 선택된 검증 위치를 담을 변수
        self.current_verify_pose = None
        
        self.fix_quat = Rotation.from_quat([0.707, 0.0, 0.707, 0.0])

        self.get_logger().info('✅ Arm Action Server Ready (Multi-Threaded)')

    # [수정 3] 분리된 콜백 함수 정의
    def vision_callback_left(self, msg):
        self.left_markers = msg.markers

    def vision_callback_right(self, msg):
        self.right_markers = msg.markers

    def get_current_tip_pose(self):
        try:
            # 타임아웃 0.0 -> 즉시 리턴 (block 방지)
            t = self.tf_buffer.lookup_transform('base_link', 'suction_cup', rclpy.time.Time())
            return t.transform
        except Exception as e:
            return None

    def get_orientation_error(self, target_quat, current_quat):
        # 1. 메시지 타입 -> Scipy Rotation 변환
        r_target = Rotation.from_quat([target_quat.x, target_quat.y, target_quat.z, target_quat.w])
        r_current = Rotation.from_quat([current_quat.x, current_quat.y, current_quat.z, current_quat.w])
        
        # 2. [핵심] 현재 자세에 보정값(Offset) 적용
        # 공식: q_corrected = q_current * q_fix
        # (TF 상의 suction_cup 좌표계를 제어 좌표계와 일치시킴)
        r_current_corrected = r_current * self.fix_quat
        
        # 3. 두 Rotation 간의 각도 차이 계산 (magnitude)
        # diff = target * current_corrected^-1
        diff = r_target * r_current_corrected.inv()
        
        # 회전량을 라디안으로 추출
        angle_error = diff.magnitude()
        
        return angle_error
    
    # [수정] tolerance_angle 인자 추가 (기본값: 0.05 라디안 ≈ 2.8도)
    def wait_until_reached(self, target_pose, timeout=60.0, tolerance=0.04, tolerance_angle=0.08):
        start_time = time.time()
        
        # 목표 위치
        tx = target_pose.pose.position.x
        ty = target_pose.pose.position.y
        tz = target_pose.pose.position.z
        
        # 목표 회전 (Quaternion)
        t_rot = target_pose.pose.orientation

        self.get_logger().info(f"   ⏳ [Move Start] Goal: ({tx:.2f}, {ty:.2f}, {tz:.2f})")
        last_log_time = time.time()

        while time.time() - start_time < timeout:
            current_tf = self.get_current_tip_pose()
            
            # TF 못 받아오면 대기
            if current_tf is None:
                if time.time() - last_log_time > 1.0:
                    self.get_logger().warn("      ⚠️ Waiting for TF update...")
                    last_log_time = time.time()
                time.sleep(0.1)
                continue
            
            # 1. 위치 오차 계산 (기존 코드)
            curr_pos = current_tf.translation
            dx = tx - curr_pos.x
            dy = ty - curr_pos.y
            dz = tz - curr_pos.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            # 2. [추가] 회전 오차 계산
            curr_rot = current_tf.rotation
            angle_diff = self.get_orientation_error(t_rot, curr_rot)

            # # 로그 출력 (1초마다)
            # if time.time() - last_log_time > 1.0:
            #     # 라디안 -> 도로 변환하여 로그 출력 (가독성)
            #     deg_diff = math.degrees(angle_diff)
            #     self.get_logger().info(
            #         f"      📉 Err: Dist={dist:.3f}m (Tol:{tolerance}), "
            #         f"Angle={deg_diff:.2f}° (Tol:{math.degrees(tolerance_angle):.1f}°)"
            #     )
            #     last_log_time = time.time()
            
            if time.time() - last_log_time > 1.0:
                deg_diff = math.degrees(angle_diff)
                
                # [디버깅] 보정된 쿼터니언 확인
                r_curr = Rotation.from_quat([curr_rot.x, curr_rot.y, curr_rot.z, curr_rot.w])
                r_corr = r_curr * self.fix_quat
                qx_c, qy_c, qz_c, qw_c = r_corr.as_quat()

                self.get_logger().info(
                    f"      📉 Err: Dist={dist:.3f}m, Angle={deg_diff:.2f}°\n"
                    f"         👉 [Raw Quat] x={curr_rot.x:.3f}, y={curr_rot.y:.3f}, z={curr_rot.z:.3f}, w={curr_rot.w:.3f}\n"
                    f"         ✅ [Fix Quat] x={qx_c:.3f}, y={qy_c:.3f}, z={qz_c:.3f}, w={qw_c:.3f}"
                )
                last_log_time = time.time()

            # 3. [핵심] 위치와 각도 둘 다 만족해야 True 리턴
            if dist < tolerance and angle_diff < tolerance_angle:
                self.get_logger().info(f"   ✅ Reached! Dist: {dist:.3f}m, Angle: {math.degrees(angle_diff):.2f}°")
                return True
            
            time.sleep(0.05)
        
        self.get_logger().warn(f"   ⚠️ Timeout! Final Dist: {dist:.3f}m, Angle: {math.degrees(angle_diff):.2f}°")
        return False

    def verify_grasp_success(self, timeout=15.0, tolerance=0.1):
        
        # [변경] self.verify_pose 대신 self.current_verify_pose 사용
        if self.current_verify_pose is None:
            self.get_logger().error("❌ No verify pose selected!")
            return False
        
        target_x = self.current_verify_pose.pose.position.x
        target_y = self.current_verify_pose.pose.position.y
        target_z = self.current_verify_pose.pose.position.z

        start_time = time.time()
        
        self.get_logger().info(f"🔎 Verifying Grasp... Target Area: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")

        while time.time() - start_time < timeout:
            # [핵심] 방향에 맞는 마커 리스트 가져오기
            markers_to_check = []
            if self.current_verify_side == "Left":
                markers_to_check = self.left_markers
            elif self.current_verify_side == "Right":
                markers_to_check = self.right_markers
                
            if len(markers_to_check) > 0:
                for marker in markers_to_check:
                    # 마커 좌표 (Robot Base 기준)
                    mx = marker.pose.position.x
                    my = marker.pose.position.y
                    mz = marker.pose.position.z
                    
                    dx = target_x - mx
                    dy = target_y - my
                    dz = target_z - mz
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    
                    if distance < tolerance:
                        self.get_logger().info(f"👁️ Success! Marker found. (Dist: {distance:.3f}m)")
                        return True
                    else:
                        pass 
                        # self.get_logger().warn(f"Marker seen but far: {distance:.2f}m", throttle_duration_sec=1)
            
            time.sleep(0.1)
            
        self.get_logger().warn("❌ Grasp Verification Failed: Marker not found near gripper.")
        return False

    def execute_callback(self, goal_handle):
        action_type = goal_handle.request.action_type
        self.get_logger().info(f'📩 Action: {action_type}')
        
        feedback = MoveManipulator.Feedback()
        result = MoveManipulator.Result()
        
        try:
            if action_type == 'pick':
                target_pose = goal_handle.request.target_pose
                    
                # [수정 5] 검증 방향(Side) 설정 추가
                tgt_y = target_pose.pose.position.y
                if tgt_y > 0:
                    self.current_verify_pose = self.verify_pose_left
                    self.current_verify_side = "Left" # 변수 업데이트
                    self.get_logger().info(f"🧭 Target Y={tgt_y:.2f} -> Verify: LEFT")
                else:
                    self.current_verify_pose = self.verify_pose_right
                    self.current_verify_side = "Right" # 변수 업데이트
                    self.get_logger().info(f"🧭 Target Y={tgt_y:.2f} -> Verify: RIGHT")
                
                self.control_gripper("open")
                
                # Pre-Approach
                pre_pose = copy.deepcopy(target_pose)
                pre_pose.pose.position.z += 0.20  
                self.publish_pose(pre_pose)
                
                if not self.wait_until_reached(pre_pose, timeout=60.0, tolerance=0.03):
                     self.get_logger().warn("⚠️ Pre-approach incomplete, trying descent...")

                # Final Approach
                self.publish_pose(target_pose)
                if not self.wait_until_reached(target_pose, timeout=60.0, tolerance=0.007):
                    raise Exception("Final Approach Timeout or Not Close Enough")

                self.control_gripper("close")
                self.get_logger().info("✊ [Grasping] Waiting 5s for physics update...")
                time.sleep(1.0)
                
                # 5. [Lift] 수직 상승
                lift_pose = copy.deepcopy(target_pose)
                lift_pose.pose.position.z += 0.20
                
                self.get_logger().info("⬆️ Lifting Object...")
                self.publish_pose(lift_pose)
                
                # 들어 올릴 때는 오차 3cm 정도면 충분
                if not self.wait_until_reached(lift_pose, timeout=60.0, tolerance=0.03):
                    self.get_logger().warn("⚠️ Lift incomplete, but moving to verify...")

                # [Verify Move] 검증 위치로 이동
                self.publish_pose(self.current_verify_pose)
                
                if not self.wait_until_reached(self.current_verify_pose):
                    raise Exception("Verification Move Timeout")
                
                if self.verify_grasp_success(tolerance=0.1):
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Pick Success"
                else:
                    raise Exception("Grasp Failed (Marker not visible)")

            elif action_type == 'place':
                target_pose = goal_handle.request.target_pose
                
                # 1. [Pre-Place] 목표 지점 상공 20cm로 이동
                pre_place_pose = copy.deepcopy(target_pose)
                pre_place_pose.pose.position.z += 0.30
                
                self.get_logger().info("🚀 Moving to Pre-Place Position...")
                self.publish_pose(pre_place_pose)
                if not self.wait_until_reached(pre_place_pose, timeout=60.0, tolerance=0.08):
                    self.get_logger().warn("⚠️ Pre-place incomplete, but proceeding...")

                # 2. [Place Descent] 목표 지점으로 하강
                self.get_logger().info("⬇️ Descending to Place Position...")
                self.publish_pose(target_pose)
                # 놓을 때는 잡을 때만큼 초정밀일 필요는 없으나, 바닥에 닿아야 하므로 1cm 오차 허용
                if not self.wait_until_reached(target_pose, timeout=60.0, tolerance=0.08):
                    raise Exception("Place Descent Timeout")

                # 3. [Release] 놓기
                self.control_gripper("open")
                self.get_logger().info("👐 Releasing Object...")
                time.sleep(1.0) # 물체가 바닥에 안착할 시간
                
                # 4. [Retreat] 물체를 치지 않게 위로 빠져나오기 (중요!)
                self.get_logger().info("⬆️ Retreating (Safety Move)...")
                self.publish_pose(pre_place_pose) # 아까 그 상공 위치로 복귀
                if not self.wait_until_reached(pre_place_pose, timeout=60.0, tolerance=0.08):
                    self.get_logger().warn("⚠️ Retreat incomplete")

                goal_handle.succeed()
                result.success = True
                result.message = "Place Sequence Completed (with Retreat)"
            
            # =====================================================
            # 3. Home (Joint 제어 - 복구됨)
            # =====================================================
            elif action_type == 'home':
                self.get_logger().info("🏠 Moving to Home Pose (Joint Control)...")
                # Joint 값 발행 (관절 제어는 TF 확인 불가하므로 시간 대기 사용)
                self.publish_joint(self.home_joints)
                
                # 피드백 전송
                feedback.current_state = "Moving to Home"
                goal_handle.publish_feedback(feedback)
                
                # 충분한 이동 시간 대기 (3초)
                time.sleep(3.0)
                
                goal_handle.succeed()
                result.success = True
                result.message = "Home Success"

            # =====================================================
            # 4. Move to Joint (Joint 직접 제어 - 복구됨)
            # =====================================================
            elif action_type == 'move_to_joint':
                joints = goal_handle.request.joint_angles
                
                # 안전 장치: 관절 개수 확인 (UR10은 6축)
                if len(joints) == 6:
                    self.get_logger().info(f"🦾 Moving to Joint Angles: {joints}")
                    self.publish_joint(joints)
                    
                    # 피드백 전송
                    feedback.current_state = "Moving Joints"
                    goal_handle.publish_feedback(feedback)
                    
                    # 이동 시간 대기 (4초 - 관절 이동은 경로에 따라 오래 걸릴 수 있음)
                    time.sleep(3.0)
                    
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Joint Move Completed"
                else:
                    raise ValueError(f"Joint angles must be length 6 (Received: {len(joints)})")
            

        except Exception as e:
            self.get_logger().error(f"❌ Action Aborted: {e}")
            goal_handle.abort()
            result.success = False
            result.message = str(e)

        return result

    def publish_pose(self, pose):
        pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(pose)

    def publish_joint(self, joints):
        msg = Float64MultiArray()
        msg.data = joints
        self.joint_pub.publish(msg)

    def control_gripper(self, command):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    server = ArmActionServer()
    
    # [★핵심] MultiThreadedExecutor 사용
    # 이걸 써야 액션이 실행되는 동안(while loop)에도 TF Listener가 백그라운드에서 데이터를 받음
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()