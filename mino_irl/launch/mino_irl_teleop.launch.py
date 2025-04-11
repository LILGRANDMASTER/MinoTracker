from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        remappings=[
            ('/cmd_vel', '/diff_drive_base_controller/cmd_vel')
        ],
        parameters=[{
            'stamped': True
        }],
        output='screen',
    )

    return LaunchDescription([
        teleop,
    ])