import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('recognition_by_lidar'),
        'config',
        'follow_me_params.yaml')

    namespace = 'follow_me'

    return LaunchDescription([
        Node(
            namespace=namespace,
            package='recognition_by_lidar',
            executable='laser_to_img',
            name='laser_to_img',
            parameters=[config],
            output='screen',
            respawn=True),
        #Node(
        #    namespace=namespace,
        #    package='recognition_by_lidar',
        #    executable='person_detector',
        #    name='person_detector',
        #    parameters=[config],
        #    output='screen',
        #    respawn=True),
        #Node(
        #    namespace=namespace,
        #    package='recognition_by_lidar',
        #    executable='base_controller',
        #    name='base_controller',
        #    parameters=[config],
        #    output='screen',
        #    respawn=True,
        #    on_exit=launch.actions.Shutdown()),
        ])
