import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('zayan')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'arm_3dof_gripper.xacro')

    robot_description_param = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([
        # Launch Gazebo Harmonic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description',
                       '-name', 'master_slave_arm',
                       '-z', '0.0'],
            output='screen'
        ),

        # State Publisher for slave robot (default namespace)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_param, 'use_sim_time': True}]
        ),

        # Spawn Controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen',
        ),

        # Clock Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # Master joint state publisher GUI (in /master namespace)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='master_joint_state_publisher_gui',
            namespace='master',
            parameters=[{'robot_description': robot_description_param}]
        ),

        # Master Robot State Publisher (for RViz visualization)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='master_robot_state_publisher',
            namespace='master',
            parameters=[{'robot_description': robot_description_param, 'frame_prefix': 'master_', 'use_sim_time': True}]
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
            # Note: you may need to save a custom rviz config file within RViz to see both master and slave properly.
        ),

        # IK Node
        Node(
            package='zayan',
            executable='master_slave_ik_node.py',
            name='master_slave_ik',
            output='screen'
        )
    ])
