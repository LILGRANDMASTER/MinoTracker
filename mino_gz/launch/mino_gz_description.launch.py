import xacro
import os

from launch_ros.actions import Node
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # directories
    mino_gz_pkg = get_package_share_directory('mino_gz')
    mino_description_pkg = get_package_share_directory('mino_description')
    urdf_config_dir = {'config_dir' : os.path.join(mino_description_pkg, 'config', 'mino')}

    # robot state publisher
    doc = xacro.process_file(
        os.path.join(mino_gz_pkg, 'urdf', 'mino_gz.urdf.xacro'),
        mappings=urdf_config_dir,
    )

    robot_description = doc.toxml()
    params = {'robot_description' : robot_description, 'publish_frequency' : 30.0, 'use_sim_time' : True}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        rsp,
    ])