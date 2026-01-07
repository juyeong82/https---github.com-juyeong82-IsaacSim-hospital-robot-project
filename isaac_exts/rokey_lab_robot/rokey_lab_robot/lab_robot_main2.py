import numpy as np
import sys
import carb
from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.core.prims import SingleArticulation 
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import isaacsim.robot_motion.motion_generation as mg
from pxr import UsdPhysics, Usd, UsdGeom, Gf
import omni.kit.viewport.utility as vp_util

from std_msgs.msg import Float64MultiArray

# ---------------------------------------------------------------------------
# RMPFlow Controller
# ---------------------------------------------------------------------------
class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: SingleArticulation, physics_dt: float = 1.0 / 60.0, attach_gripper: bool = False) -> None:
        if attach_gripper:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflowSuction")
        else:
            self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)

        self._default_position, self._default_orientation = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(robot_position=self._default_position, robot_orientation=self._default_orientation)

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(robot_position=self._default_position, robot_orientation=self._default_orientation)
    
    def set_robot_base_pose(self, position, orientation):
        self._motion_policy.set_robot_base_pose(robot_position=position, robot_orientation=orientation)

# ---------------------------------------------------------------------------
# 메인 클래스
# ---------------------------------------------------------------------------
class LabRobotMain2(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        pass

    def setup_scene(self):
        world = self.get_world()
        stage = world.stage
        
        # DLSS 및 렌더링 최적화 설정
        try:
            import omni.kit.viewport.utility as vp_util
            viewport_api = vp_util.get_active_viewport()
            if viewport_api:
                # Isaac Sim 5.0에서는 settings를 통해 DLSS 설정
                settings = carb.settings.get_settings()
                settings.set("/rtx/post/dlss/execMode", 0)  # 0=Performance, 1=Balanced, 2=Quality, 3=Auto
                # 렌더링 해상도 스케일 (0.5 ~ 0.7 추천, 낮을수록 빠름/화질저하)
                settings.set("/app/renderer/resolution/scale", 0.7)
                # 그림자 비활성화 (성능 향상 큼)
                settings.set("/rtx/directLighting/sampledLighting/enabled", False)
                # 반사 효과 비활성화 (금속 재질 등의 연산 줄임)
                settings.set("/rtx/reflections/enabled", False)
                # 간접광(GI) 비활성화
                settings.set("/rtx/indirectDiffuse/enabled", False)
                # 앰비언트 오클루전 비활성화
                settings.set("/rtx/ambientOcclusion/enabled", False)
                print("✅ Applied Performance Settings")
        except Exception as e:
            print(f"⚠️ Failed to set settings: {e}")

        
        # 1. USD 로드
        user_usd_path = "/home/jy/hospital_robot_project/assets/main_space_v3/nova v10.usd" 
        add_reference_to_stage(usd_path=user_usd_path, prim_path="/World")
        
        self.robot_prim_path = "/World/Nova_Carter_ROS_test"
        self.ee_link_path    = "/World/Nova_Carter_ROS_test/ur10/ee_link"
        gripper_path         = "/World/Nova_Carter_ROS_test/ur10/ee_link/SurfaceGripper"

        if world.scene.object_exists("nova_carter"):
            world.scene.remove_object("nova_carter")
        
        self.nova_carter = world.scene.add(
            SingleArticulation(prim_path=self.robot_prim_path, name="nova_carter")
        )

        self.gripper = SurfaceGripper(
            end_effector_prim_path=self.ee_link_path, 
            surface_gripper_path=gripper_path
        )

        # 2. SideTable 추가
        # sidetable_path = "/home/jy/hospital_robot_project/assets/Collected_SideTable/SideTable.usd"
        # add_reference_to_stage(usd_path=sidetable_path, prim_path="/World/SideTable_1")
        # sidetable_prim = stage.GetPrimAtPath("/World/SideTable")
        # if sidetable_prim.IsValid():
        #     UsdGeom.XformCommonAPI(sidetable_prim).SetTranslate(Gf.Vec3d(26.0, 7.3, 0.0))

        # 3. BloodTube 추가 (강제 Transform 설정)
        # blood_tube_path = "/home/jy/hospital_robot_project/assets/Collected_blood_tube_aruco1/blood_tube_aruco1.usd"
        # add_reference_to_stage(usd_path=blood_tube_path, prim_path="/World/BloodTube")
        
        # blood_prim = stage.GetPrimAtPath("/World/BloodTube")
        # if blood_prim.IsValid():
        #     # 기존 Transform 완전 초기화
        #     xformable = UsdGeom.Xformable(blood_prim)
        #     xformable.ClearXformOpOrder()
            
        #     # 새로운 Transform 설정
        #     xform_op = xformable.AddTranslateOp()
        #     xform_op.Set(Gf.Vec3d(25.75, 7.6, 0.8123))
            
        #     print(f"✅ BloodTube positioned at (25.75, 7.6, 0.8123)")
            
        # ---------------------------------------------------------
        # 4. SideTable #2 추가 (위치: 26.0, 9.25, 0.0)
        # ---------------------------------------------------------
        # # USD 경로 재사용
        # sidetable_path = "/home/jy/hospital_robot_project/assets/Collected_SideTable/SideTable.usd"
        # add_reference_to_stage(usd_path=sidetable_path, prim_path="/World/SideTable_2")
        
        # sidetable2_prim = stage.GetPrimAtPath("/World/SideTable_2")
        # if sidetable2_prim.IsValid():
        #     # 위치 설정
        #     UsdGeom.XformCommonAPI(sidetable2_prim).SetTranslate(Gf.Vec3d(26.0, 9.25, 0.0))
        #     print("✅ SideTable #2 added at (26.0, 9.25, 0.0)")

        # ---------------------------------------------------------
        # 5. BloodTube #2 추가 (위치: 25.75, 7.6, 0.8123 / 회전: Z 180도)
        # ---------------------------------------------------------
        # blood_tube_path = "/home/jy/hospital_robot_project/assets/Collected_blood_tube_aruco1/blood_tube_aruco1.usd"
        # add_reference_to_stage(usd_path=blood_tube_path, prim_path="/World/BloodTube_2")
        
        # blood2_prim = stage.GetPrimAtPath("/World/BloodTube_2")
        # if blood2_prim.IsValid():
        #     # 기존 Transform 초기화 후 재설정 (안전장치)
        #     xformable = UsdGeom.Xformable(blood2_prim)
        #     xformable.ClearXformOpOrder()
            
        #     # 1) 위치 설정 (Translate)
        #     xformable.AddTranslateOp().Set(Gf.Vec3d(25.75, 9.0, 0.8123))
            
        #     # 2) 회전 설정 (Rotate Z) - USD는 기본적으로 Degree(도) 단위 사용
        #     xformable.AddRotateZOp().Set(180.0)
            
        #     print("✅ BloodTube #2 added at (25.75, 9.0, 0.8123) with Z-180deg rotation")

    async def setup_post_load(self):
        # [수정됨] 여기서 import 수행 (Lazy Import)
        global rclpy, PoseStamped, String
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import String
        except ImportError:
            carb.log_error("ROS 2 모듈(rclpy)을 찾을 수 없음. 터미널에서 'source /opt/ros/humble/setup.bash' 실행 확인 필요.")
            return

        self._world = self.get_world()
        self.robots = self._world.scene.get_object("nova_carter")

        # ================================================================
        # 🎥 USD에 저장된 카메라 활성화
        # ================================================================
        try:
            viewport_api = vp_util.get_active_viewport()
            if viewport_api:
                # USD 내 카메라 경로 (본인이 추가한 카메라 경로로 변경)
                camera_path = "/World/RobotViewCamera"
                viewport_api.set_active_camera(camera_path)
                print(f"✅ Activated camera: {camera_path}")
        except Exception as e:
            print(f"⚠️ Failed to activate camera: {e}")
        # ================================================================

        # ROS 2 초기화
        if not rclpy.ok():
            rclpy.init()

        # ----------------------------------------------------------------
        # [ROS 2] Subscriber 설정
        # ----------------------------------------------------------------
        self.node = rclpy.create_node("isaac_rmp_commander")
        
        # 1. 팔 목표 좌표 수신
        self.sub_pose = self.node.create_subscription(
            PoseStamped,
            "/rmp_target_pose",
            self.ros_pose_callback,
            10
        )
        
        # 2. 그리퍼 명령 수신 (String)
        self.sub_gripper = self.node.create_subscription(
            String,
            "/gripper_command",
            self.ros_gripper_callback,
            10
        )
        
        # 3. 조인트 직접 제어 명령 수신 (추가됨)
        self.sub_joints = self.node.create_subscription(
            Float64MultiArray,
            "/joint_command",
            self.ros_joint_callback,
            10
        )
        
        # 제어 모드 변수 (기본값: pose)
        # pose: RMPFlow를 이용한 좌표 제어
        # joint: 각도 직접 제어
        self.control_mode = "pose" 
        self.target_joint_positions = None
        
        print("📡 [ROS 2] Waiting for commands...")
        print("   - Pose: /rmp_target_pose (geometry_msgs/PoseStamped)")
        print("   - Gripper: /gripper_command (std_msgs/String) -> 'open' or 'close'")

        # EE Prim (디버깅용)
        # Prim 설정
        # Prim 설정
        stage = self._world.stage

        # Robot Base Link (좌표 변환 기준)
        self.base_link_path = "/World/Nova_Carter_ROS_test/chassis_link"
        self.base_link_prim = stage.GetPrimAtPath(self.base_link_path)

        # Suction Cup (그리퍼 실제 끝 위치)
        self.suction_cup_path = "/World/Nova_Carter_ROS_test/ur10/ee_link/suction_cup"
        self.suction_cup_prim = stage.GetPrimAtPath(self.suction_cup_path)

        # 유효성 검증
        if not self.base_link_prim.IsValid():
            carb.log_error(f"Base Link not found: {self.base_link_path}")
        if not self.suction_cup_prim.IsValid():
            carb.log_error(f"Suction Cup not found: {self.suction_cup_path}")
        
        # BloodTube 위치 재확인
        blood_prim = stage.GetPrimAtPath("/World/BloodTube")
        if blood_prim.IsValid():
            xformable = UsdGeom.Xformable(blood_prim)
            matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = matrix.ExtractTranslation()
            print(f"🩸 BloodTube actual position: ({translation[0]}, {translation[1]}, {translation[2]})")
        
        self.cspace_controller = RMPFlowController(
            name="nova_carter_cspace_controller", 
            robot_articulation=self.robots, 
            attach_gripper=True
        )
        
        ur10_pos = np.array([0.06917, 0.0, 0.67383])
        ur10_rot = np.array([1.0, 0.0, 0.0, 0.0])
        self.cspace_controller.set_robot_base_pose(ur10_pos, ur10_rot)

        joint_names = self.robots.dof_names
        ur10_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        self.arm_indices = []
        for name in ur10_names:
            if name in joint_names:
                self.arm_indices.append(joint_names.index(name))

        full_dof = self.robots.num_dof
        initial_pos = np.zeros(full_dof)
        # arm_home = np.array([0, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0])
        # gripper_camera가 left_camera와 동일한 화각을 가지는 관측 자세
        arm_home = np.array([0.1745, -1.4835, -2.4435, -0.7854, 1.5708, 1.75])
        for i, idx in enumerate(self.arm_indices):
            initial_pos[idx] = arm_home[i]
        self.robots.set_joint_positions(initial_pos)

        # 초기 목표값
        # self.current_target_pos = np.array([-0.5, 0.0, 1.0]) 
        # self.current_target_rot = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
        
        # 외부 명령 대기
        self.current_target_pos = None
        self.current_target_rot = None
        
        # 도착 상태 플래그
        self.reached_target = False  
        
        self.log_timer = 0 
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
    
    def ros_pose_callback(self, msg):
        self.control_mode = "pose"
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        new_target = np.array([x, y, z])
        
        # 새 명령만 로그
        if self.current_target_pos is None or not np.allclose(self.current_target_pos, new_target, atol=0.001):
            print(f"📩 [New Command] Target: ({x:.2f}, {y:.2f}, {z:.2f})")
            self.reached_target = False  # 새 명령이므로 도착 상태 초기화
        
        self.current_target_pos = new_target
        
        rx, ry, rz = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        rw = msg.pose.orientation.w
        if (rx*rx + ry*ry + rz*rz + rw*rw) > 0.1:
            self.current_target_rot = np.array([rw, rx, ry, rz])

    # [Callback 2] 그리퍼 명령 수신
    def ros_gripper_callback(self, msg):
        command = msg.data.lower()
        if command == "open":
            self.gripper.open()
            print("👐 [Gripper] OPEN Request")
        elif command == "close":
            self.gripper.close()
            print("✊ [Gripper] CLOSE Request")
        else:
            print(f"⚠️ Unknown command: {command} (Use 'open' or 'close')")
            
    def ros_joint_callback(self, msg):
        # 메시지가 오면 제어 모드를 joint로 변경
        self.control_mode = "joint"
        # 들어온 리스트를 numpy 배열로 변환
        self.target_joint_positions = np.array(msg.data)
        print(f"🦾 [Joint Control] Mode Switched. Target: {self.target_joint_positions}")

    def physics_step(self, step_size):
        # rclpy가 import 되지 않았을 경우 방어 코드
        if 'rclpy' not in globals() or rclpy is None:
            return

        # ROS 2 메시지 처리 (여기서 콜백 함수들이 실행됨)
        rclpy.spin_once(self.node, timeout_sec=0)
        
        # Suction Cup 위치 확인 (그리퍼 실제 끝 위치)
        suction_cup_relative = None
    
        if self.suction_cup_prim.IsValid() and self.base_link_prim.IsValid():
            # Suction Cup의 World Transform 행렬
            suction_world_matrix = UsdGeom.Xformable(self.suction_cup_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            # Base Link의 World Transform 행렬
            base_world_matrix = UsdGeom.Xformable(self.base_link_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            
            # base_link 좌표계로 변환 (행렬 역변환)
            # suction_in_base = suction_world * (base_world)^-1
            base_world_inverse = base_world_matrix.GetInverse()
            suction_in_base_matrix = suction_world_matrix * base_world_inverse
            
            # 변환된 행렬에서 위치 추출
            translation = suction_in_base_matrix.ExtractTranslation()
            suction_cup_relative = np.array([translation[0], translation[1], translation[2]])
        
        # 로그 출력
        self.log_timer += 1

        if self.current_target_pos is None:
            # 대기 상태: 5초마다
            if self.log_timer % 300 == 0:
                print(f"⏸️  Idle (Waiting for command)")
        else:
            # 이동 중: 거리 계산
            if suction_cup_relative is not None:
                distance = np.linalg.norm(suction_cup_relative - self.current_target_pos)
                
                # 1. 도착 판정 (한 번만 로그)
                if distance < 0.02 and not self.reached_target:
                    print(f"✅ Reached! (Error: {distance:.3f}m)")
                    self.reached_target = True
                
                # 2. 이동 중 로그 (1초마다) + 도착 안했을 때만(not self.reached_target) 출력
                elif self.log_timer % 60 == 0 and not self.reached_target:
                    print(f"🎯 Moving... | Target: [{self.current_target_pos[0]:.2f}, {self.current_target_pos[1]:.2f}, {self.current_target_pos[2]:.2f}] | Distance: {distance:.3f}m")

            
        # 로봇 제어 (RMPFlow)
        if self.control_mode == "pose":
            # 외부 명령이 있을 때만 RMPFlow 제어 실행
            if self.current_target_pos is not None and self.current_target_rot is not None:
                rmp_action = self.cspace_controller.forward(
                    target_end_effector_position=self.current_target_pos,
                    target_end_effector_orientation=self.current_target_rot
                )
                full_action = ArticulationAction(
                    joint_positions=rmp_action.joint_positions,
                    joint_velocities=rmp_action.joint_velocities,
                    joint_indices=np.array(self.arm_indices)
                )
                self.robots.apply_action(full_action)
            # 목표값이 없으면 아무것도 하지 않음 (홈 위치 유지)
        
        elif self.control_mode == "joint" and self.target_joint_positions is not None:
            # 새로 추가된 조인트 직접 제어
            # RMPFlow를 거치지 않고 바로 관절 명령 전달
            joint_action = ArticulationAction(
                joint_positions=self.target_joint_positions,
                joint_indices=np.array(self.arm_indices)
            )
            self.robots.apply_action(joint_action)
            