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
            parameters=[{'joint_signs': [
                1.0, 1.0,  # 1 , 2
                1.0, -1.0, # 3 , 4
                -1.0, 1.0, # 5 , 6
                1.0, 1.0,  # 7 , 8
                1.0, -1.0, # 9 , 10
                -1.0, 1.0, # 11, 12
                1.0, 1.0,  # 13, 14
                1.0, -1.0, # 15, 16
                -1.0, 1.0, # 17, 18
            ],
            'joint_offsets': [
                0.0, 0.0,       # 1 , 2
                0.061, 0.061,   # 3 , 4
                0.808, 0.808,   # 5 , 6
                0.0, 0.0,       # 7 , 8
                0.061, 0.061,   # 9 , 10
                0.808, 0.808,   # 11, 12
                0.0, 0.0,       # 13, 14
                0.061, 0.061,   # 15, 16
                0.808, 0.808,   # 17, 18
            ]}],
        ),
    ])
