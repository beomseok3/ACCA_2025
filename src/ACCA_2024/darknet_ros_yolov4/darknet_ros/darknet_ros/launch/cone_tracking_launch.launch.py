from launch import LaunchDescription
from launch.actions import *
import launch_ros.actions
from ament_index_python.packages import *
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml
import pathlib



def generate_launch_description():
      darknet_ros_share_dir = get_package_share_directory('darknet_ros')
      network_param_file = darknet_ros_share_dir + '/config/yolov4_cone_tracking.yaml'

      pkg_name_ground_filter = "plane_ground_filter"
      pkg_path_ground_filter = get_package_share_directory(pkg_name_ground_filter)

      pkg_name_clustering = "adaptive_clustering"
      pkg_path_clustering = get_package_share_directory(pkg_name_clustering)

      #-----------------------------------------------------------------------------------#

      image_concat_node = launch_ros.actions.Node(
            package='fusion',
            executable='image_concat',
            name='image_concat_node',
      )

      darknet_ros_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([darknet_ros_share_dir + '/launch/darknet_ros.launch.py']),
      launch_arguments={'network_param_file': network_param_file}.items()
      )

      crop_box_node = launch_ros.actions.Node(
            package='fusion',
            executable='cropbox',
            name='crop_box_node',
      )

      ground_filter_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_path_ground_filter, 'launch', 'plane_ground_filter.launch.py'))
      )

      clustering_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_path_clustering, 'launch', 'adaptive_clustering.launch.py'))
      )


      fusion_3cam_node = launch_ros.actions.Node(
            package='fusion',
            executable='fusion_3cam',
            name='fusion_3cam_node',
      )


      acca_odom_tf_node = launch_ros.actions.Node(
            package='tf_cone',
            executable='acca_odom_tf_publisher_static',
            name='acca_odom_tf_node',
      )

      cone_in_odom_frame_node = launch_ros.actions.Node(
            package='tf_cone',
            executable='cone_in_odom_frame',
            name='cone_in_odom_frame_node',
      )

      cone_in_odom_frame_count_node = launch_ros.actions.Node(
            package='tf_cone',
            executable='cone_in_odom_frame_count',
            name='cone_in_odom_frame_count_node',
      )

      path_cone_node = launch_ros.actions.Node(
            package='path_plan_cone',
            executable='path_cone',
            name='path_cone_node',
      )


      return LaunchDescription([
            launch_ros.actions.SetParameter(name='use_sim_time', value=False),
            crop_box_node,
            ground_filter_launch,
            clustering_launch,
            image_concat_node,
            darknet_ros_launch,
            fusion_3cam_node,
            # cone_in_odom_frame_node,
            # cone_in_odom_frame_count_node,
            # path_cone_node,
      ])

if __name__ == '__main__':
  generate_launch_description()