from setuptools import find_packages, setup
import os
from glob import glob # [추가] 파일 패턴 매칭용

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [추가] launch 폴더의 모든 .launch.py 파일을 설치 경로로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # [추가] config 폴더의 모든 .yaml 파일을 설치 경로로 복사
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juyeong',
    maintainer_email='ju0gkorea@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 1. AprilTag Docking & Pose Publisher (도킹 관련)
            'april_docking_marker = my_pkg.april_docking_marker:main',
            'april_docking_tf = my_pkg.april_docking_tf:main',
            'april_pose_publisher_marker = my_pkg.april_pose_publisher_marker:main',
            'april_pose_publisher_tf = my_pkg.april_pose_publisher_tf:main',
            'test_undock = my_pkg.test_undock:main',
            'undock_test_node = my_pkg.undock_test_node:main',

            # 2. Arm Control (로봇팔 제어)
            'arm_action_server = my_pkg.arm_action_server:main',

            # 3. ArUco Detectors (마커 인식 - 카메라별)
            'aruco_detector_debug = my_pkg.aruco_detector_debug:main',
            'aruco_detector_front = my_pkg.aruco_detector_front:main',
            'aruco_detector_gripper = my_pkg.aruco_detector_gripper:main',
            'aruco_detector_left = my_pkg.aruco_detector_left:main',
            'aruco_detector_right = my_pkg.aruco_detector_right:main',

            # 4. Main Controllers (시스템 제어)
            'main_controller = my_pkg.main_controller:main',
            'nav_commander = my_pkg.nav_commander:main',

            # 5. UI (사용자 인터페이스)
            'ui = my_pkg.ui:main',
            'ui2 = my_pkg.ui2:main',
        ],
    },
)
