import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    mino_irl_pkg = get_package_share_directory('mino_irl')

    twist_mux_params = os.path.join(mino_irl_pkg, 'config', 'mino', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/stamper_in')],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time' : True}],
        remappings=[('/cmd_vel_in', '/stamper_in'),
                    ('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel'),],
    )
    return LaunchDescription([
        twist_mux,
        twist_stamper,
    ])