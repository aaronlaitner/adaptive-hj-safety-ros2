import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'safe_rl_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copies all files in the 'launch' folder
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),

    # Copies all files in the 'worlds' folder
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    (os.path.join('share', package_name, 'models', 'your_robot'), glob('models/your_robot/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Safe RL Student Team SFU',
    maintainer_email='duong.nghuu2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "safe_rl_robot_node = redexp.robots.turtlebot:main",
        ],
    },
)
