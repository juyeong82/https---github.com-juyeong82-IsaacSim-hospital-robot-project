import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import Bool
import math
import time
import numpy as np
from action_msgs.msg import GoalStatus
# [Action Interfaces]
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry  # 오도메트리 추가
from moma_interfaces.action import Dock, MoveManipulator, RunDelivery
from moma_interfaces.msg import MarkerArray

from scipy.spatial.transform import Rotation 
import numpy as np

# [변환 함수 추가: 클래스 밖 전역 함수로 배치]
def euler_from_quaternion(x, y, z, w):
    """쿼터니언 -> 오일러각 변환 (Roll, Pitch, Yaw)"""
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

class HospitalOrchestrator(Node):
    def __init__(self):
        super().__init__('hospital_main_node')
        
        # ---------------------------------------------------------
        # 1. 환경 설정 (Room Database & Item Config)
        # ---------------------------------------------------------
        # [설정] 방 별 테이블 중심 좌표 (UI에서 주는 정보라 가정)
        # 형식: "Room Name": {"coords": [x, y, z], "approach": "Left" or "Right"}
        self.room_db = {
            "Nurse Station A (Base)":  {"coords": [23.129, 9.392, 0.0], "approach": "Left"},
            "Ward 102":                {"coords": [24.62435, 14.62949, 0.0], "approach": "Left"},
            "Main Pharmacy (Central)": {"coords": [-9.0, 5.07121, 0.0], "approach": "Left"},
            "Sub Pharmacy": {"coords": [-2.5, 5.07121, 0.0], "approach": "Left"},
            "Clinical Lab (Zone C)":   {"coords": [23.129, 9.392, 0.0], "approach": "Right"}, # 테스트용 (우측접근)
        }
        
        # [추가] 복귀할 홈 위치 좌표 [x, y, z] (여기만 수정하면 됨)
        self.home_coords = [0.0, 0.0, 0.0]

        # 오프셋 기준: 마커 중심으로부터 [x(우), y(하/위), z(앞/뒤)] (OpenCV 좌표계 기준 아님, 마커 자체 로컬 좌표계)
        # ---------------------------------------------------------
        self.item_db = {
            "Blood Sample": {
                "id": 0, 
                "offset": [0.0, 0.03, -0.04]  # 요청하신 블러드 튜브 옵셋
            },
            "Medicine": {
                "id": 1, 
                "offset": [0.0, 0.0, -0.06]     # (예시) 약통은 마커 정중앙 잡기
            },
            "Narcotics": {
                "id": 2, 
                "offset": [0.0, 0.05, -0.02]  # (예시) 금고 손잡이 위치 등
            },
        }

        # [설정] 도킹 오프셋 (테이블 중심 기준)
        # Left Approach 기준 (User Provided)
        # Table: (23.129, 9.392) -> Dock: (25.603, 8.400)
        # Diff: X +2.474, Y -0.992
        self.offset_x = 2.474
        self.offset_y = 1.2 # 절대값으로 저장 (Left: -y, Right: +y 적용 예정)
        
        self.quat_left = Quaternion(x=-0.000, y=-0.000, z=0.996, w=0.087)
        self.quat_right = Quaternion(x=-0.000, y=0.000, z=0.996, w=-0.087)
        
        # [추가] 방향별 그립/검증 공통 오리엔테이션 (CLI 테스트 성공 값)
        # Left Approach (Target Y > 0): 카메라가 오른쪽을 보며 파지
        self.grasp_quat_left = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)

        # Right Approach (Target Y < 0): 카메라가 왼쪽을 보며 파지
        self.grasp_quat_right = Quaternion(x=-0.707, y=0.0, z=0.707, w=0.0)
        # ---------------------------------------------------------
        # 2. ROS2 통신 설정
        # ---------------------------------------------------------
        self.cb_group = ReentrantCallbackGroup()

        # Action Clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        self.dock_client = ActionClient(self, Dock, 'dock_robot', callback_group=self.cb_group)
        self.arm_client = ActionClient(self, MoveManipulator, 'move_manipulator', callback_group=self.cb_group)
        
        # Action Server (UI와 통신)
        self._action_server = ActionServer(
            self, RunDelivery, 'run_delivery', 
            self.execute_delivery_callback, 
            callback_group=self.cb_group,
            cancel_callback=self.cancel_callback
        )

        # Vision Control Publishers
        self.pub_enable_left = self.create_publisher(Bool, '/vision/enable_left', 10)
        self.pub_enable_right = self.create_publisher(Bool, '/vision/enable_right', 10)
        
        # 후진(Undocking)을 위한 cmd_vel 퍼블리셔
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Vision Data Subscribers (일회성 수신용)
        self.detected_markers = {} # ID별 Pose 저장
        self.create_subscription(MarkerArray, '/vision/left_markers', self.vision_cb_left, 10, callback_group=self.cb_group)
        self.create_subscription(MarkerArray, '/vision/right_markers', self.vision_cb_right, 10, callback_group=self.cb_group)
        
        # 언도킹 제어를 위한 정밀 마커 포즈 구독 (april_pose_publisher 데이터)
        self.latest_dock_pose = None
        self.create_subscription(PoseStamped, 'detected_dock_pose', self.dock_pose_callback, 10, callback_group=self.cb_group)

        # 마커 인식기(april_pose_publisher) On/Off 제어용
        self.pub_dock_trigger = self.create_publisher(Bool, '/docking/trigger', 10)
        
        # 언도킹 정밀 제어를 위한 오도메트리 구독
        self.create_subscription(Odometry, '/chassis/odom', self.odom_callback, 10, callback_group=self.cb_group)
        self.current_odom_yaw = None
        self.latest_pose_time = self.get_clock().now() # 마커 데이터 타임스탬프용

        self.get_logger().info("🏥 Hospital Main Node Ready (Waiting for UI Command...)")

    # Action Server 취소 요청 수락 콜백
    def cancel_callback(self, goal_handle):
        self.get_logger().info('⚠️ Received Cancel Request!')
        return CancelResponse.ACCEPT

    # 실행 중 취소 여부 확인 헬퍼 함수
    def check_cancel(self, goal_handle, result):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Task Canceled by User"
            self.get_logger().warn("🛑 Delivery Sequence Canceled!")
            return True # 취소됨
        return False # 취소 안됨

    # PoseStamped를 받아서 frame_id를 유지하도록 변경
    def apply_grasp_offset(self, base_pose_stamped, offset_xyz):
        """
        base_pose_stamped: PoseStamped 객체 (header 포함)
        """
        # 1. Pose 정보 추출
        base_pose = base_pose_stamped.pose
        
        t = [base_pose.position.x, base_pose.position.y, base_pose.position.z]
        q = [base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w]
        
        R = Rotation.from_quat(q).as_matrix()
        T_base_marker = np.eye(4)
        T_base_marker[:3, :3] = R
        T_base_marker[:3, 3] = t
        
        # 2. Offset 행렬 생성
        T_offset = np.eye(4)
        T_offset[0, 3] = offset_xyz[0]
        T_offset[1, 3] = offset_xyz[1]
        T_offset[2, 3] = offset_xyz[2]
        
        # 3. 행렬 곱
        T_base_target = T_base_marker @ T_offset
        
        final_pos = T_base_target[:3, 3]
        final_rot = Rotation.from_matrix(T_base_target[:3, :3]).as_quat()
        
        new_pose = PoseStamped()
        
        # [핵심] 하드코딩 삭제 -> 원본 메시지의 frame_id를 그대로 사용
        new_pose.header.frame_id = base_pose_stamped.header.frame_id 
        
        new_pose.pose.position.x = final_pos[0]
        new_pose.pose.position.y = final_pos[1]
        new_pose.pose.position.z = final_pos[2]
        new_pose.pose.orientation.x = final_rot[0]
        new_pose.pose.orientation.y = final_rot[1]
        new_pose.pose.orientation.z = final_rot[2]
        new_pose.pose.orientation.w = final_rot[3]
        
        return new_pose.pose # Action Server에는 Pose 타입으로 전달
    
    # ---------------------------------------------------------
    # Helper: 좌표 계산 로직
    # ---------------------------------------------------------
    def get_docking_pose(self, room_name):
        """테이블 좌표와 접근 방향을 기반으로 도킹 좌표 계산"""
        if room_name not in self.room_db:
            self.get_logger().error(f"❌ Unknown Room: {room_name}")
            return None, None

        info = self.room_db[room_name]
        tx, ty, tz = info['coords']
        approach = info['approach']

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # 오프셋 적용
        # 현재 맵 기준 X축은 동일하게 증가, Y축만 접근 방향에 따라 반전된다고 가정
        final_x = tx + self.offset_x
        
        if approach == "Left":
            final_y = ty - self.offset_y
            pose.pose.orientation = self.quat_left
        else: # Right
            final_y = ty + self.offset_y
            pose.pose.orientation = self.quat_right
            
        pose.pose.position.x = final_x
        pose.pose.position.y = final_y
        pose.pose.position.z = 0.0
        
        self.get_logger().info(f"📍 Calculated Dock Pose for {room_name} ({approach}): ({final_x:.2f}, {final_y:.2f})")
        return pose, approach

    # ---------------------------------------------------------
    # Helper: 비전 콜백 및 제어
    # ---------------------------------------------------------
    def vision_cb_left(self, msg):
        for m in msg.markers:
            ps = PoseStamped()
            ps.header = msg.header  # 핵심: 여기서 frame_id를 받아옵니다.
            ps.pose = m.pose
            self.detected_markers[m.id] = ps

    def vision_cb_right(self, msg):
        for m in msg.markers:
            ps = PoseStamped()
            ps.header = msg.header  # 핵심: 여기서 frame_id를 받아옵니다.
            ps.pose = m.pose
            self.detected_markers[m.id] = ps
            
    def set_vision(self, side, enable):
        msg = Bool()
        msg.data = enable
        for _ in range(3):  # 3회 반복 발행
            if side == "Left":
                self.pub_enable_left.publish(msg)
            elif side == "Right":
                self.pub_enable_right.publish(msg)
            time.sleep(0.05)  # 50ms 간격

    async def wait_for_marker(self, target_id, side, timeout=5.0):
        """특정 ID 마커가 보일 때까지 대기"""
        self.detected_markers.clear()
        self.set_vision(side, True) # 카메라 켜기
        
        start_time = time.time()
        self.get_logger().info(f"👀 Scanning for Item ID {target_id} using {side} Camera...")
        
        found_pose = None
        while time.time() - start_time < timeout:
            if target_id in self.detected_markers:
                found_pose = self.detected_markers[target_id]
                self.get_logger().info(f"✅ Found Marker {target_id}!")
                break
            time.sleep(0.1)
            
        # self.set_vision(side, False) # 카메라 끄기
        
        if found_pose is None:
            self.get_logger().error("❌ Marker detection failed (Timeout)")
        
        return found_pose
    
    # [추가] 마커 포즈 콜백 (언도킹용)
    def dock_pose_callback(self, msg):
        self.latest_dock_pose = msg
        
    # [추가] 오도메트리 콜백
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_odom_yaw = yaw

    # [수정] dock_pose_callback (타임스탬프 업데이트 추가)
    def dock_pose_callback(self, msg):
        self.latest_dock_pose = msg
        self.latest_pose_time = self.get_clock().now()

    # [추가] 마커 오리엔테이션 Yaw 추출 헬퍼
    def get_marker_orientation_yaw(self):
        if self.latest_dock_pose:
            q = self.latest_dock_pose.pose.orientation
            # 쿼터니언이 유효하지 않은 경우 방지
            if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
                return None
            _, yaw, _ = euler_from_quaternion(q.x, q.y, q.z, q.w)
            return yaw
        return None
    
    # [추가] 로봇 정지 헬퍼
    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())

    async def undock_using_marker(self, approach_side, reverse_dist=2.0):
        """
        검증된 3단계 언도킹 로직 적용
        Phase 1: 제자리 회전 정렬 (Marker Orientation 기준)
        Phase 2: 후진 및 자세 보정 (Active Yaw Correction)
        Phase 3: 최종 170도 회전 (Odom 기준)
        """
        self.get_logger().info(f"\n🚀 Starting Precision Undock: {approach_side} Side")
        
        # 설정값 정의 (테스트 노드 값 준수)
        TARGET_ANGLE_DEG = 10.0
        P_GAIN = 4.0
        MAX_ROT_SPEED = 0.5
        MIN_ROT_SPEED = 0.1
        REVERSE_SPEED = -0.2
        
        # 1. 마커 인식 활성화
        for _ in range(3):
            self.pub_dock_trigger.publish(Bool(data=True))
            time.sleep(0.05)
        time.sleep(1.0) # 인식 안정화 대기

        # ------------------------------------------------------------------
        # [Phase 1] 목표 각도 설정 및 제자리 정렬
        # ------------------------------------------------------------------
        target_angle_rad = math.radians(TARGET_ANGLE_DEG)
        target_yaw = 0.0
        
        # Left/Right 문자열을 로직에 맞게 변환
        # approach_side가 "Left"면 로봇 기준 우회전 필요 -> Target Negative
        if approach_side == "Left":
            target_yaw = -target_angle_rad
            self.get_logger().info(f"🎯 Goal: Marker Orientation <= {math.degrees(target_yaw):.1f}°")
        else:
            target_yaw = target_angle_rad
            self.get_logger().info(f"🎯 Goal: Marker Orientation >= {math.degrees(target_yaw):.1f}°")

        start_time = self.get_clock().now()
        
        self.get_logger().info("🔄 Phase 1: In-Place Rotation Alignment...")
        
        while rclpy.ok():
            # 타임아웃 60초
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 30.0:
                self.get_logger().warn("⏰ Phase 1 Rotation Timeout!")
                break
            
            # 데이터 수신 지연 체크
            if (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 0.5:
                self.stop_robot()
                time.sleep(0.1)
                continue

            current_yaw = self.get_marker_orientation_yaw()
            if current_yaw is None: 
                time.sleep(0.1)
                continue

            current_deg = math.degrees(current_yaw)
            target_deg = math.degrees(target_yaw)

            # 종료 조건 체크
            done = False
            if approach_side == "Left":
                if current_yaw <= target_yaw: done = True
            else:
                if current_yaw >= target_yaw: done = True
            
            if done:
                self.get_logger().info(f"✅ Rotation Done! (Cur: {current_deg:.2f}°)")
                break

            # P 제어
            error = current_yaw - target_yaw
            speed = np.clip(-P_GAIN * error, -MAX_ROT_SPEED, MAX_ROT_SPEED)
            
            # 최소 속도 클램핑
            if abs(speed) < MIN_ROT_SPEED:
                speed = MIN_ROT_SPEED if speed > 0 else -MIN_ROT_SPEED

            cmd = Twist()
            cmd.angular.z = float(speed) # float 형변환 안전장치
            self.pub_cmd_vel.publish(cmd)
            
            # 로그 출력 (스로틀링)
            if self.latest_dock_pose:
                curr_dist = self.latest_dock_pose.pose.position.z
                self.get_logger().info(
                    f"🔄 Rot | Dist: {curr_dist:.2f}m | Orient: {current_deg:.2f}° -> Goal: {target_deg:.1f}°", 
                    throttle_duration_sec=0.5
                )
            
            time.sleep(0.05) # 루프 주기 조절

        self.stop_robot()
        time.sleep(0.5)

        # ------------------------------------------------------------------
        # [Phase 2] 후진 및 자세 유지 (Active Yaw Correction)
        # ------------------------------------------------------------------
        self.get_logger().info("🔙 Phase 2: Reversing with Active Yaw Correction...")
        
        # 데이터 최신화 대기
        while (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 0.5:
            time.sleep(0.1)
            
        start_dist = self.latest_dock_pose.pose.position.z
        # 인자로 받은 reverse_dist 사용 (기본 2.0m 권장)
        final_target_dist = start_dist + reverse_dist
        
        rev_start = self.get_clock().now()
        
        while rclpy.ok():
            if (self.get_clock().now() - rev_start).nanoseconds / 1e9 > 30.0:
                self.get_logger().warn("⏰ Phase 2 Reverse Timeout!")
                break
            
            # 마커 놓침 체크 -> 비상 정지
            if (self.get_clock().now() - self.latest_pose_time).nanoseconds / 1e9 > 0.5:
                self.stop_robot()
                self.get_logger().warn("⚠️ Marker lost during reverse. Stopping Phase 2.")
                break
            
            curr_dist = self.latest_dock_pose.pose.position.z
            current_yaw = self.get_marker_orientation_yaw()
            
            # 목표 거리 도달 확인
            if curr_dist >= final_target_dist:
                self.get_logger().info(f"✅ Distance Reached: {curr_dist:.2f}m")
                break
            
            # 각도 보정 (Phase 1과 동일 로직)
            ang_speed = 0.0
            if current_yaw is not None:
                error = current_yaw - target_yaw
                # 후진 중이므로 급격한 회전 제한 (Gain 3.0, Limit 0.3)
                ang_speed = np.clip(-3.0 * error, -0.3, 0.3)
                
                # 데드존 (1도 미만 무시)
                if abs(error) < math.radians(1.0):
                    ang_speed = 0.0

            cmd = Twist()
            cmd.linear.x = REVERSE_SPEED
            cmd.angular.z = float(ang_speed)
            
            self.pub_cmd_vel.publish(cmd)
            
            self.get_logger().info(
                f"🔙 Rev | Dist: {curr_dist:.2f}m | YawErr: {math.degrees(current_yaw - target_yaw):.1f}°",
                throttle_duration_sec=0.5
            )
            time.sleep(0.05)

        self.stop_robot()
        
        # 마커 인식 끄기 (Phase 3는 Odom 사용하므로)
        for _ in range(3):
            self.pub_dock_trigger.publish(Bool(data=False))
            time.sleep(0.05)

        # ------------------------------------------------------------------
        # [Phase 3] 90도(실제 170도) 회전 (Odom Feedback)
        # ------------------------------------------------------------------
        self.get_logger().info("🔄 Phase 3: Final Turn (Odom)...")
        time.sleep(0.5)

        # Odom 데이터 대기
        wait_cnt = 0
        while self.current_odom_yaw is None and wait_cnt < 20:
            time.sleep(0.1)
            wait_cnt += 1
            
        if self.current_odom_yaw is None:
            self.get_logger().error("❌ No Odom data! Skipping final turn.")
        else:
            start_yaw = self.current_odom_yaw
            target_rad = 3.0  # 약 170도
            target_deg = math.degrees(target_rad)
            
            # 회전 방향 설정: Left Approach -> 좌회전(+), Right Approach -> 우회전(-)
            rot_sign = 1.0 if approach_side == "Left" else -1.0
            
            cmd = Twist()
            cmd.angular.z = 0.5 * rot_sign
            
            turn_start_time = self.get_clock().now()
            
            while rclpy.ok():
                # 타임아웃 30초
                if (self.get_clock().now() - turn_start_time).nanoseconds / 1e9 > 30.0:
                    self.get_logger().warn("⏰ Phase 3 Turn Timeout!")
                    break

                if self.current_odom_yaw is None:
                    time.sleep(0.05)
                    continue

                # 각도 차이 계산 (Wrap-around 처리 포함)
                diff = self.current_odom_yaw - start_yaw
                diff = math.atan2(math.sin(diff), math.cos(diff))
                current_moved = abs(diff)

                if current_moved >= target_rad:
                    self.get_logger().info(f"✅ Turn Done: {math.degrees(current_moved):.1f}°")
                    break
                
                self.pub_cmd_vel.publish(cmd)
                
                self.get_logger().info(
                    f"🔄 Turning... {math.degrees(current_moved):.1f}° / {target_deg:.1f}°",
                    throttle_duration_sec=0.5
                )
                time.sleep(0.05)

        self.stop_robot()
        self.get_logger().info("✅ Undocking Sequence Complete.")


    # ===== 여기에 추가 =====
    async def retry_action(self, action_func, max_retries=2, *args, **kwargs):
        """액션 실패 시 자동 재시도 래퍼"""
        for attempt in range(max_retries + 1):
            result = await action_func(*args, **kwargs)
            if result:
                return True
            if attempt < max_retries:
                self.get_logger().warn(f"⚠️ Attempt {attempt+1} failed, retrying... ({max_retries - attempt} left)")
                time.sleep(1.0)  # 재시도 전 잠시 대기
        return False
    
    # ---------------------------------------------------------
    # Helper: 액션 클라이언트 래퍼 (취소 연동 수정됨)
    # ---------------------------------------------------------
    async def call_nav2(self, pose, main_goal_handle):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal)
        nav_goal_handle = await send_goal_future
        
        if not nav_goal_handle.accepted:
            self.get_logger().error("❌ Nav2 Goal Rejected!")
            return False
            
        result_future = nav_goal_handle.get_result_async()
        
        # 결과가 나올 때까지 기다리면서, 메인 취소 요청이 들어오는지 감시
        while not result_future.done():
            if main_goal_handle.is_cancel_requested:
                self.get_logger().warn("🛑 Cancelling Nav2 because Main Task was Canceled...")
                await nav_goal_handle.cancel_goal_async() # Nav2에 멈추라고 명령
                return False
            time.sleep(0.1) # CPU 점유율 방지
        
        wrapped_result = result_future.result()
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Nav2 Arrived Successfully")
            return True
        else:
            self.get_logger().error(f"❌ Nav2 Failed or Canceled status: {wrapped_result.status}")
            return False

    async def call_docking(self, main_goal_handle):
        goal = Dock.Goal()
        self.dock_client.wait_for_server()
        send_goal_future = self.dock_client.send_goal_async(goal)
        dock_handle = await send_goal_future
        if not dock_handle.accepted: return False
        
        result_future = dock_handle.get_result_async()
        while not result_future.done():
            if main_goal_handle.is_cancel_requested:
                await dock_handle.cancel_goal_async()
                return False
            time.sleep(0.1)

        res = result_future.result()
        return res.result.success

    async def call_arm(self, action_type, main_goal_handle, pose=None):
        goal = MoveManipulator.Goal()
        goal.action_type = action_type
        if pose:
            ps = PoseStamped()
            ps.header.frame_id = "base_link"
            ps.pose = pose
            goal.target_pose = ps
            
        self.arm_client.wait_for_server()
        send_goal_future = self.arm_client.send_goal_async(goal)
        arm_handle = await send_goal_future
        if not arm_handle.accepted: return False
        
        result_future = arm_handle.get_result_async()
        while not result_future.done():
            if main_goal_handle.is_cancel_requested:
                await arm_handle.cancel_goal_async()
                return False
            time.sleep(0.1)

        res = result_future.result()
        return res.result.success
    # [main_controller.py] execute_delivery_callback 메서드를 아래 코드로 덮어쓰기

    async def execute_delivery_callback(self, goal_handle):
        request = goal_handle.request
        feedback = RunDelivery.Feedback()
        result = RunDelivery.Result()
        
        # 1. 입력값 파싱 및 실행 계획 수립
        raw_mode = request.task_mode if request.task_mode else "ALL"
        item_name = request.item_type
        clean_name = item_name.split('(')[0].strip()

        # 전체 실행 순서 정의
        FULL_SEQUENCE = [
            "NAV_PICKUP", "DOCK_PICKUP", "PICK", 
            "NAV_DROPOFF", "DOCK_DROPOFF", "PLACE", "NAV_HOME"
        ]
        
        steps_to_run = set()

        # [핵심 로직] 모드에 따른 실행 단계 설정
        if raw_mode == "ALL":
            steps_to_run = set(FULL_SEQUENCE)
        elif raw_mode.endswith("_CONT"):
            # 이어하기 모드: 해당 단계부터 끝까지
            start_step = raw_mode.replace("_CONT", "")
            if start_step in FULL_SEQUENCE:
                start_idx = FULL_SEQUENCE.index(start_step)
                steps_to_run = set(FULL_SEQUENCE[start_idx:])
            else:
                steps_to_run = {start_step} # 매칭 안되면 해당 단계만
        else:
            # 수동 모드: 딱 그 단계만 실행
            steps_to_run = {raw_mode}

        # 2. 아이템 정보 로드 (기존 동일)
        if clean_name in self.item_db:
            item_info = self.item_db[clean_name]
            target_id = item_info['id']
            grasp_offset = item_info['offset']
        else:
            self.get_logger().warn(f"⚠️ Unknown Item: {clean_name}, using default.")
            target_id = 0
            grasp_offset = [0.0, 0.0, 0.0]

        pickup_pose, pickup_side = self.get_docking_pose(request.pickup_loc)
        dropoff_pose, dropoff_side = self.get_docking_pose(request.dropoff_loc)
        
        self.get_logger().info(f"🚀 TASK START [Req: {raw_mode}] | Steps: {len(steps_to_run)}")

        try:
            # =================================================
            # [STEP 1] 픽업지 이동 (NAV_PICKUP)
            # =================================================
            if "NAV_PICKUP" in steps_to_run:
                feedback.current_state = "NAVIGATING TO PICKUP"
                goal_handle.publish_feedback(feedback)
                
                if not pickup_pose: raise Exception("Invalid Pickup Location")
                self.get_logger().info(f"🚗 Navigating to {request.pickup_loc}...")
                
                if not await self.retry_action(self.call_nav2, 1, pickup_pose, goal_handle):
                    raise Exception("Navigation to Pickup Failed")

            if self.check_cancel(goal_handle, result): return result

            # =================================================
            # [STEP 2] 픽업지 도킹 (DOCK_PICKUP)
            # =================================================
            if "DOCK_PICKUP" in steps_to_run:
                feedback.current_state = "DOCKING AT PICKUP"
                goal_handle.publish_feedback(feedback)
                
                self.get_logger().info("⚓ Starting Precision Docking (Pickup)...")
                if not await self.retry_action(self.call_docking, 1, goal_handle):
                    raise Exception("Docking Failed")

            if self.check_cancel(goal_handle, result): return result

            # =================================================
            # [STEP 3] 물체 인식 및 파지 (PICK)
            # =================================================
            if "PICK" in steps_to_run:
                feedback.current_state = "SCANNING & PICKING"
                goal_handle.publish_feedback(feedback)
                
                camera_side = "Right" if pickup_side == "Left" else "Left"
                self.get_logger().info(f"👀 Approach: {pickup_side} -> Using Camera: {camera_side}")

                # [수정 포인트] 재시도 로직을 직접 구현하여, 실패 시 '마커 인식'부터 다시 수행
                pick_success = False
                max_retries = 2  # 최대 재시도 횟수 (총 3회)

                for attempt in range(max_retries + 1):
                    self.get_logger().info(f"🔄 Pick Sequence Attempt {attempt + 1}/{max_retries + 1}")
                    
                    # 1. 마커 다시 인식 (매 시도마다 새로운 위치 갱신)
                    marker_raw_pose = await self.wait_for_marker(target_id, camera_side)
                    
                    if marker_raw_pose:
                        self.get_logger().info(f"🔎 Applying Offset {grasp_offset}")
                        final_grasp_pose = self.apply_grasp_offset(marker_raw_pose, grasp_offset)

                        if pickup_side == "Left":
                            final_grasp_pose.orientation = self.grasp_quat_right
                        else:
                            final_grasp_pose.orientation = self.grasp_quat_left

                        self.get_logger().info("🦾 Sending PICK Command...")
                        
                        # 2. 팔 이동 (여기서는 retry_action 대신 직접 호출)
                        # 이미 밖에서 for문을 돌고 있으므로, 내부 재시도는 불필요
                        if await self.call_arm('pick', goal_handle, final_grasp_pose):
                            pick_success = True
                            self.get_logger().info("✅ Pick Success!")
                            break  # 성공 시 루프 탈출
                        else:
                            self.get_logger().warn("⚠️ Arm Move Failed. Retrying sequence...")
                    else:
                        self.get_logger().warn("⚠️ Marker not found. Retrying sequence...")
                    
                    # 실패 시 잠시 대기 후 재시도
                    time.sleep(1.0)

                # 모든 시도 실패 시 예외 발생
                if not pick_success:
                    raise Exception("Pick Action Failed after retries (Vision+Arm)")
                
                # ---------------------------------------------------------
                # 담기 (Stow) 동작은 비전 인식이 필요 없으므로 기존 방식 유지
                # ---------------------------------------------------------
                self.get_logger().info("📦 Stowing Item to Cargo Area...")
                stow_pose = PoseStamped()
                stow_pose.header.frame_id = "base_link"
                stow_pose.pose.position.x = -0.5
                stow_pose.pose.position.y = 0.0
                stow_pose.pose.position.z = 0.72
                stow_pose.pose.orientation = Quaternion(x=-0.5, y=0.5, z=0.5, w=0.5)

                if not await self.retry_action(self.call_arm, 1, 'place', goal_handle, stow_pose.pose):
                    raise Exception("Stowing Action Failed")
                
                self.get_logger().info("💤 Turning OFF Camera after PICK phase")
                self.set_vision(camera_side, False)
                
                self.get_logger().info("⚓ Performing Post-Pick Undocking...")
                await self.undock_using_marker(pickup_side, reverse_dist=2.0)

            if self.check_cancel(goal_handle, result): return result

            # =================================================
            # [STEP 4] 하역지 이동 (NAV_DROPOFF)
            # =================================================
            if "NAV_DROPOFF" in steps_to_run:
                await self.call_arm('home', goal_handle)
                
                feedback.current_state = "NAVIGATING TO DROPOFF"
                goal_handle.publish_feedback(feedback)
                
                self.get_logger().info(f"🚗 Navigating to {request.dropoff_loc}...")
                if not await self.retry_action(self.call_nav2, 1, dropoff_pose, goal_handle):
                    raise Exception("Navigation to Dropoff Failed")
            
            if self.check_cancel(goal_handle, result): return result

            # =================================================
            # [STEP 5] 하역지 도킹 (DOCK_DROPOFF)
            # =================================================
            if "DOCK_DROPOFF" in steps_to_run:
                feedback.current_state = "DOCKING AT DROPOFF"
                goal_handle.publish_feedback(feedback)
                
                self.get_logger().info("⚓ Docking at Drop-off...")
                if not await self.retry_action(self.call_docking, 1, goal_handle):
                    raise Exception("Docking at Drop-off Failed")

            if self.check_cancel(goal_handle, result): return result

            # =================================================
            # [STEP 6] 내려놓기 (PLACE)
            # =================================================
            if "PLACE" in steps_to_run:
                drop_camera_side = "Right" if dropoff_side == "Left" else "Left"
                self.get_logger().info(f"👀 Turning ON {drop_camera_side} Camera...")
                self.set_vision(drop_camera_side, True)
                
                self.get_logger().info("📦 Retrieving Item from Cargo Area...")
                retrieve_pose = PoseStamped()
                retrieve_pose.header.frame_id = "base_link"
                retrieve_pose.pose.position.x = -0.5
                retrieve_pose.pose.position.y = 0.0
                retrieve_pose.pose.position.z = 0.7
                retrieve_pose.pose.orientation = Quaternion(x=-0.5, y=0.5, z=0.5, w=0.5)

                if not await self.retry_action(self.call_arm, 1, 'pick', goal_handle, retrieve_pose.pose):
                    raise Exception("Retrieving Action Failed")
                
                feedback.current_state = "PLACING"
                goal_handle.publish_feedback(feedback)
                
                place_pose = PoseStamped()
                place_pose.header.frame_id = "base_link"
                place_pose.pose.position.x = -0.16
                place_pose.pose.position.z = 1.0

                if dropoff_side == "Left":
                    place_pose.pose.position.y = -0.8
                    place_pose.pose.orientation = self.grasp_quat_right
                else: 
                    place_pose.pose.position.y = 0.8
                    place_pose.pose.orientation = self.grasp_quat_left

                self.get_logger().info(f"🦾 Sending FIXED PLACE Command...")
                if not await self.retry_action(self.call_arm, 1, 'place', goal_handle, place_pose.pose):
                    raise Exception("Place Action Failed")
                
                await self.call_arm('home', goal_handle)
                
                self.get_logger().info("💤 Turning OFF Camera")
                self.set_vision(drop_camera_side, False)
                
                self.get_logger().info("⚓ Performing Post-Pick Undocking...")
                await self.undock_using_marker(dropoff_side, reverse_dist=2.0)

            # =================================================
            # [STEP 7] 시작 위치 복귀 (NAV_HOME)
            # =================================================
            if "NAV_HOME" in steps_to_run:
                feedback.current_state = "RETURNING HOME"
                goal_handle.publish_feedback(feedback)

                home_pose = PoseStamped()
                home_pose.header.frame_id = "map"
                home_pose.pose.position.x = self.home_coords[0]
                home_pose.pose.position.y = self.home_coords[1]
                home_pose.pose.position.z = self.home_coords[2]
                home_pose.pose.orientation.w = 1.0 

                self.get_logger().info(f"🏠 Returning to Home...")
                if not await self.retry_action(self.call_nav2, 1, home_pose, goal_handle):
                    raise Exception("Return to Home Failed")
            
            # [유틸리티] HOME (단독 실행용)
            if "HOME" in steps_to_run and raw_mode == "HOME":
                self.get_logger().info("🏠 Moving Arm to HOME...")
                await self.call_arm('home', goal_handle)

            # 여기까지 오면 성공
            self.get_logger().info("✅ Full Sequence or Step Complete!")
            result.success = True
            result.message = f"Tasks {list(steps_to_run)} Completed."
            goal_handle.succeed()

        except Exception as e:
            self.get_logger().error(f"🛑 Task Aborted: {str(e)}")
            result.success = False
            result.message = str(e)
            goal_handle.abort()

        return result

def main(args=None):
    rclpy.init(args=args)
    node = HospitalOrchestrator()
    
    # 멀티스레드 실행 (액션 서버와 클라이언트 동시 동작 위함)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()