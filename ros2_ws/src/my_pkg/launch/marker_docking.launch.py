import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
                parameters=[{
                    'size': 0.25, 
                    'family': '36h11',
                    'max_hamming': 0,       # 오인식 줄이기
                    'qos_profile': 'sensor_data' # Best Effort로 설정하여 수신율 향상
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. 도킹 포즈 퍼블리셔 (Camera frame 출력)
    april_pose_publisher = Node(
        package='my_pkg',
        executable='april_pose_publisher_marker',
        name='april_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Simple Precision Docking (자동 도킹)
    april_docking = Node(
        package='my_pkg',
        executable='april_docking_marker',
        name='april_docking',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'docking_distance_threshold': 2.2},  
            {'rotation_threshold': 0.087},        # 5도
            {'approach_speed': 0.45},              # 접근 속도
            {'rotation_speed': 0.5},              # 회전 속도
            {'final_speed': 0.15},                # 최종 속도
            {'auto_start': False},                 # 자동 시작
        ]
    )

    # 4. TF (map → odom)
    # tf_map_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = [
    #         '--x', '0', '--y', '0', '--z', '0', 
    #         '--yaw', '0', '--pitch', '0', '--roll', '0', 
    #         '--frame-id', 'map', '--child-frame-id', 'odom'
    #     ],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    return LaunchDescription([
        apriltag_node,
        april_pose_publisher,
        april_docking,
        # tf_map_odom
    ])