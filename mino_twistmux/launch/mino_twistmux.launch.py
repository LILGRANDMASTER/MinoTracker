import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    mino_twistmux_pkg = get_package_share_directory('mino_twistmux')
<<<<<<< HEAD

    twist_mux_params = os.path.join(mino_twistmux_pkg, 'config', 'mino', 'twist_mux.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': LaunchConfiguration(use_sim_time_arg)}],
=======

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    twist_mux_params = os.path.join(mino_twistmux_pkg, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
>>>>>>> navigation
        remappings=[('/cmd_vel_out', '/stamper_in')],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
<<<<<<< HEAD
        parameters=[{'use_sim_time' : LaunchConfiguration(use_sim_time_arg)}],
=======
        parameters=[{'use_sim_time' : LaunchConfiguration('use_sim_time')}],
>>>>>>> navigation
        remappings=[('/cmd_vel_in', '/stamper_in'),
                    ('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel'),],
    )
    return LaunchDescription([
        use_sim_time_arg,
        twist_mux,
        twist_stamper,
    ])