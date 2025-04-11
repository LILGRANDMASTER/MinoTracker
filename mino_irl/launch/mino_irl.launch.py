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

    mino_irl_pkg = get_package_share_directory('mino_irl')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mino_irl_pkg, 'launch', 'mino_irl_description.launch.py')
        )
    )

    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mino_irl_pkg, 'launch', 'mino_irl_control.launch.py')
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mino_irl_pkg, 'launch', 'mino_irl_rviz.launch.py')
        )
    )


    return LaunchDescription([
        rsp,
        control,
        rviz,
    ])