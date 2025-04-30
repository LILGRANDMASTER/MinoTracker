import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mino_mapper_pkg = get_package_share_directory('mino_mapper')
    mapper_parameters = os.path.join(mino_mapper_pkg, 'config', 'mapper_params_online_async.yaml')


    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                                  'launch',
                                  'online_async_launch.py'])
        ),

        launch_arguments={
            'slam_params_file': slam_parameters,
            'slam_params_file': mapper_parameters,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        mapper,
    ])