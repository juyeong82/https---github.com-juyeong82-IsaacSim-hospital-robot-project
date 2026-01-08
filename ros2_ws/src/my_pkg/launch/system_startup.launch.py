import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 패키지 이름 (본인의 패키지 이름으로 수정 필요)
    package_name = 'my_pkg' 

    return LaunchDescription([
        
        # 1. 아루코 디텍터 (right Camera)
        Node(
            package=package_name,
            executable='aruco_detector_right',
            name='aruco_detector_right',
            output='screen',
            parameters=[{'use_sim_time': True}]  # 시뮬레이션 환경인 경우 True
        ),

        # 2. 아루코 디텍터 (Left Camera)
        Node(
            package=package_name,
            executable='aruco_detector_left',
            name='aruco_detector_left',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 3. 팔 제어 액션 서버 (Arm Action Server)
        Node(
            package=package_name,
            executable='arm_action_server',
            name='arm_action_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 4. 메인 오케스트레이터 (Main Controller)
        Node(
            package=package_name,
            executable='main_controller',  # setup.py 엔트리 포인트 이름 확인
            name='main_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])