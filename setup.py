import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'safe_rl_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),

        # Copies all files in the 'worlds' folder
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "params"), glob("params/*")),
        (os.path.join('share', package_name, 'description'), glob('description/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Safe RL Student Team SFU',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "monitor_node = redexp.robots.turtlebot:main",
            "robot_pose_publisher = robot_pose_publisher.robot_pose_publisher:main",
        ],
    },
)
