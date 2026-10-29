import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('cpp_pubsub')

    robot_description = xacro.process_file(
        os.path.join(share, 'urdf', 'hexapod_urdf.xacro'),
        mappings={'mesh_dir': 'file://' + os.path.join(share, 'meshes')},
    ).toxml()

    return LaunchDescription([
        ExecuteProcess(cmd=['gz', 'sim', '-r', 'empty.sdf'], output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Drops the robot 15 cm above the ground
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'hexapod', '-z', '0.15'],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(share, 'config', 'gz_bridge.yaml')}],
        ),

        Node(
            package='cpp_pubsub',
            executable='udp_joint_bridge',
            output='screen',
        ),
    ])
