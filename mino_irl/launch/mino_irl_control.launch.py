import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mino_irl_pkg = get_package_share_directory('mino_irl')
    controller_params_file = os.path.join(mino_irl_pkg, 'config', 'mino', 'mino_irl_control.yaml')

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controller_params_file,
        ],

        output='both',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_params_file,
        ],
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster,
        diff_drive_controller,
    ])