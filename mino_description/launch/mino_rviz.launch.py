import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    mino_description_pkg = get_package_share_directory('mino_description')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mino_description_pkg, 'launch', 'mino_description.launch.py'),
        )
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='',      
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
    )

    return LaunchDescription([
        rsp,
        jsp_gui,
        rviz,
    ])