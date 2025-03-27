import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    mino_description_pkg = get_package_share_directory('mino_description')

    config_dir = {'config_dir' : os.path.join(mino_description_pkg, 'config', 'mino')}

    doc = xacro.process_file(
        os.path.join(mino_description_pkg, 'urdf', 'mino.urdf.xacro'),
        mappings=config_dir,
    )


    robot_description = doc.toxml()
    params = {'robot_description' : robot_description, 'publish_frequency' : 30.0}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    

    return LaunchDescription([
        rsp,
    ])