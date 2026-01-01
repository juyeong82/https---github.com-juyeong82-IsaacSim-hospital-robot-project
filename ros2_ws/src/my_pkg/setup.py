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
            'nav_commander = my_pkg.nav_commander:main',
            'visual_servoing = my_pkg.visual_servoing:main',
            'aruco_detector = my_pkg.aruco_detector:main',
            'aruco_detector_front = my_pkg.aruco_detector_front:main',
            'aruco_detector_debug = my_pkg.aruco_detector_debug:main',
            'aruco_detector_left = my_pkg.aruco_detector_left:main',
            'aruco_detector_left_org = my_pkg.aruco_detector_left_org:main',
            'aruco_detector_right = my_pkg.aruco_detector_right:main',
            'arm_action_server = my_pkg.arm_action_server:main',
            # [추가] 도킹 중계 노드 등록
            'dock_pose_publisher = my_pkg.dock_pose_publisher:main', 
            # [추가] 미션 커맨더 등록 (필요시)
            'mission_commander = my_pkg.mission_commander:main',
        ],
    },
)
