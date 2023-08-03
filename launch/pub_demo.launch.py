from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    pub = Node(
        package = 'monitor',
        executable = 'pub_demo',
        name = 'camera',
        output = 'screen',
        parameters = [
            {"period": 30},
            {"video_path": "/home/avees/ros2_ws/video/test.mp4"},
            {"node_index": 1},
        ]
    )
    
    return LaunchDescription([
        pub
    ])