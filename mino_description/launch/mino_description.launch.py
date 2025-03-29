import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time');

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='set true if u want to use simulation clock',
    )

    mino_description_pkg = get_package_share_directory('mino_description')

    config_dir = {'config_dir' : os.path.join(mino_description_pkg, 'config', 'mino')}

    doc = xacro.process_file(
        os.path.join(mino_description_pkg, 'urdf', 'mino.urdf.xacro'),
        mappings=config_dir,
    )


    robot_description = doc.toxml()
    params = {'robot_description' : robot_description, 'publish_frequency' : 30.0, 'use_sim_time' : use_sim_time}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    

    return LaunchDescription([
        use_sim_time_arg,
        rsp,
    ])