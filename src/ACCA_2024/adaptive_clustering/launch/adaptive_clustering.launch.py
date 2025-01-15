import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_clustering',
            executable='adaptive_clustering',
            name='adaptive_clustering',
            parameters=[{'print_fps': True}],
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
