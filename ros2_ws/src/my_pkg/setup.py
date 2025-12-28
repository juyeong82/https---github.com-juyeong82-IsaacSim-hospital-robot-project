from setuptools import find_packages, setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
