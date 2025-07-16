import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
def generate_launch_description():

    mpc_node = Node(
        package='erp42_control',
        executable='mpc_node2',
        parameters=[],
        arguments=[],
        output='screen',
    )
    
    m2k_node = Node(
        package='erp42_control',
        executable='mps2kph2',
        parameters=[],
        arguments=[],
        output='screen',
    )

    return LaunchDescription(
        [
            mpc_node,
            m2k_node
        ]
    )

generate_launch_description()