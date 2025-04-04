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
    mino_gz_pkg = get_package_share_directory('mino_gz')

    my_world = os.path.join(mino_gz_pkg, 'worlds', 'world.sdf')

    # ros_gz_sim launch
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                  'launch',
                                  'gz_sim.launch.py'])
        ),

        launch_arguments={'gz_args': f'-r -v4 {my_world}'}.items(),
    )

    # bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                  ],
        output='screen',
    )

    # spawn entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'Minotracker', '-allow_renaming', 'true',
                   '-z', '2.0'],
    )

    return LaunchDescription([
        gz_sim_launch,
        gz_spawn_entity,
        bridge,
    ])