import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('arm_teleop'),
        'urdf', 'arm.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),

    ])