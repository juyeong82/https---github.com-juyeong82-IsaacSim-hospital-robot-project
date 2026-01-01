import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration  # [추가]

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_pkg')
    config_file = os.path.join(pkg_dir, 'config', 'nova_docking.yaml')
    
    # [추가] use_sim_time을 명시적으로 True로 고정
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. AprilTag 인식 노드
    apriltag_node = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                remappings=[
                    ('image_rect', '/front_camera/rgb'),
                    ('camera_info', '/front_camera/camera_info')
                ],
                parameters=[{'size': 0.25, 'family': '36h11'}],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # 파라미터 전달
    )

    # 2. 중계 노드
    dock_pose_publisher = Node(
        package='my_pkg',
        executable='dock_pose_publisher',
        name='dock_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. 도킹 서버 (여기가 핵심)
    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        # [수정] yaml 파일과 use_sim_time을 하나의 리스트로 확실하게 전달
        parameters=[config_file, {'use_sim_time': use_sim_time}], 
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # 4. Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}, {'use_sim_time': use_sim_time}],
    )

    # 5. TF (Static Transform)
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        apriltag_node,
        dock_pose_publisher,
        docking_server,
        lifecycle_manager,
        tf_map_odom
    ])