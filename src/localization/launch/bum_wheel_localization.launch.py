from launch import LaunchDescription
from ament_index_python.packages import *
import launch_ros.actions
import os
import yaml
import pathlib
from launch.substitutions import *
from launch.actions import DeclareLaunchArgument
import xacro


def generate_launch_description():
    pkg_name = "localization"
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file)

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description.toxml()}],
        output="screen",
    )

    wheel_odometry_node = launch_ros.actions.Node(
        package="localization",
        executable="wheel_odometry",
        name="wheel_odometry",
        remappings=[("odometry/wheel", "/odometry/wheel")],
    )

    erp_twist_node = launch_ros.actions.Node(
        package="localization", executable="erp_twist_world", name="erp_twist_world"
    )

    return LaunchDescription(
        [
            launch_ros.actions.SetParameter(name="use_sim_time", value=False),
            # robot_state_publisher_node,
            wheel_odometry_node,
            erp_twist_node,
            # rviz_node
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
