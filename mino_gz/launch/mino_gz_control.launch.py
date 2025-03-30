import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # path to package
    mino_gz_pkg = get_package_share_directory('mino_gz')

    # path to controller config file
    mino_controller_yaml = os.path.join(mino_gz_pkg, 'config', 'mino_controller.yaml')

    # controllers nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            mino_controller_yaml,
        ]
    )


    return LaunchDescription([
        joint_state_broadcaster_spawner,
        diff_drive_base_controller_spawner,
    ])