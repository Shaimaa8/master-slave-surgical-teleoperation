import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_dir = get_package_share_directory('surgical_robot_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')

    return LaunchDescription([
        # Master Robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='master',
            parameters=[{'robot_description': Command(['xacro ', urdf_path]), 'frame_prefix': 'master/'}]
        ),
        # Slave Robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='slave',
            parameters=[{'robot_description': Command(['xacro ', urdf_path]), 'frame_prefix': 'slave/'}]
        ),
        # Static Transforms to link them to world
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0.5', '0', '0', '0', '0', 'world', 'master/base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '-0.5', '0', '0', '0', '0', 'world', 'slave/base_link']),
        # RViz
        Node(package='rviz2', executable='rviz2', output='screen')
    ])
