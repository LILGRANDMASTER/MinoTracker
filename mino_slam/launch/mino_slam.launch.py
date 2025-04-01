import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mino_slam_pkg = get_package_share_directory('mino_slam')
    slam_parameters = os.path.join(mino_slam_pkg, 'config', 'mapper_params_online_async.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                                  'launch',
                                  'online_async_launch.py'])
        ),

        launch_arguments={
            'params_file': slam_parameters,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        slam,
    ])