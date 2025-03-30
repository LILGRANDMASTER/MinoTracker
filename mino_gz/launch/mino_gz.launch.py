import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    mino_gz_pkg = get_package_share_directory('mino_gz')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mino_gz_pkg, 'launch', 'mino_gz_description.launch.py')
        )
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mino_gz_pkg, 'launch', 'mino_gz_sim.launch.py')
        )
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mino_gz_pkg, 'launch', 'mino_gz_control.launch.py')
        )
    )


    return LaunchDescription([
        rsp,
        gz_sim,
        control,
    ])