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
class LabRobotMain(BaseSample):
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
        user_usd_path = "/home/jy/hospital_robot_project/assets/main_space_v1/nova v9.usd" 
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
        sidetable_path = "/home/jy/hospital_robot_project/assets/Collected_SideTable/SideTable.usd"
        add_reference_to_stage(usd_path=sidetable_path, prim_path="/World/SideTable")
        sidetable_prim = stage.GetPrimAtPath("/World/SideTable")
        if sidetable_prim.IsValid():
            UsdGeom.XformCommonAPI(sidetable_prim).SetTranslate(Gf.Vec3d(26.0, 7.3, 0.0))

        # 3. BloodTube 추가 (강제 Transform 설정)
        blood_tube_path = "/home/jy/hospital_robot_project/assets/Collected_blood_tube_aruco1/blood_tube_aruco1.usd"
        add_reference_to_stage(usd_path=blood_tube_path, prim_path="/World/BloodTube")
        
        blood_prim = stage.GetPrimAtPath("/World/BloodTube")
        if blood_prim.IsValid():
            # 기존 Transform 완전 초기화
            xformable = UsdGeom.Xformable(blood_prim)
            xformable.ClearXformOpOrder()
            
            # 새로운 Transform 설정
            xform_op = xformable.AddTranslateOp()
            xform_op.Set(Gf.Vec3d(25.75, 7.6, 0.8123))
            
            print(f"✅ BloodTube positioned at (25.75, 7.6, 0.8123)")

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
        stage = self._world.stage
        self.ee_prim = stage.GetPrimAtPath(self.ee_link_path)
        
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
        arm_home = np.array([-np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0])
        for i, idx in enumerate(self.arm_indices):
            initial_pos[idx] = arm_home[i]
        self.robots.set_joint_positions(initial_pos)

        # 초기 목표값
        self.current_target_pos = np.array([-0.8, 0.8, 0.93]) 
        self.current_target_rot = euler_angles_to_quat(np.array([0, np.pi/2, 0]))
        
        self.log_timer = 0 
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()

    # [Callback 1] 좌표 수신
    def ros_pose_callback(self, msg):
        self.control_mode = "pose"
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.current_target_pos = np.array([x, y, z])

        rx, ry, rz = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        rw = msg.pose.orientation.w
        if (rx*rx + ry*ry + rz*rz + rw*rw) > 0.1:
            self.current_target_rot = np.array([rw, rx, ry, rz])
            
        print(f"📩 [Move] To: {self.current_target_pos}")

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

        # EE 위치 확인 (디버깅)
        if self.ee_prim.IsValid():
            transform_matrix = UsdGeom.Xformable(self.ee_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = transform_matrix.ExtractTranslation()
            ee_pos = np.array([translation[0], translation[1], translation[2]])
        else:
            ee_pos = np.array([0.0, 0.0, 0.0])
        
        self.log_timer += 1
        if self.log_timer % 60 == 0:
            print(f"🎯 Target: {self.current_target_pos} | 🤖 Current EE: {ee_pos}")

        # 로봇 제어 (RMPFlow)
        if self.control_mode == "pose":
            # 기존 RMPFlow 제어 (좌표 이동)
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
        
        elif self.control_mode == "joint" and self.target_joint_positions is not None:
            # 새로 추가된 조인트 직접 제어
            # RMPFlow를 거치지 않고 바로 관절 명령 전달
            joint_action = ArticulationAction(
                joint_positions=self.target_joint_positions,
                joint_indices=np.array(self.arm_indices)
            )
            self.robots.apply_action(joint_action)