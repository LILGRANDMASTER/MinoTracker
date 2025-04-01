import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    mino_gz_pkg = get_package_share_directory('mino_gz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(mino_gz_pkg, 'rviz', 'rviz_config.rviz')]
    )

    return LaunchDescription([
        rviz,
    ])