from setuptools import setup
from glob import glob

setup(
    name='arm_teleop',
    version='0.0.1',
    packages=['arm_teleop'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/arm_teleop']),
        ('share/arm_teleop', ['package.xml']),
        ('share/arm_teleop/urdf',   glob('urdf/*')),
        ('share/arm_teleop/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'tracker_ros_node = arm_teleop.tracker_ros_node:main',
        ],
    },
)