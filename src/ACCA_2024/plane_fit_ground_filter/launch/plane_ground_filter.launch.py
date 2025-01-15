from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument('input_topic', default_value='/velodyne_points'),
        # DeclareLaunchArgument('no_ground_point_topic', default_value='/points_no_ground'),
        # DeclareLaunchArgument('ground_point_topic', default_value='/points_ground'),
        # DeclareLaunchArgument('all_points_topic', default_value='/all_points'),
        # DeclareLaunchArgument('clip_height', default_value='4.0'),
        # DeclareLaunchArgument('sensor_height', default_value='1.77'),
        # DeclareLaunchArgument('min_distance', default_value='2.0'),
        # DeclareLaunchArgument('max_distance', default_value='75.0'),
        # DeclareLaunchArgument('sensor_model', default_value='32'),
        # DeclareLaunchArgument('num_iter', default_value='3'),
        # DeclareLaunchArgument('num_lpr', default_value='20'),
        # DeclareLaunchArgument('th_seeds', default_value='1.2'),
        # DeclareLaunchArgument('th_dist', default_value='0.3'),

        #2023 ACCA Mapping Parameter
        # DeclareLaunchArgument('input_topic', default_value='/velodyne_points'),
        # DeclareLaunchArgument('no_ground_point_topic', default_value='/points_no_ground'),
        # DeclareLaunchArgument('ground_point_topic', default_value='/points_ground'),
        # DeclareLaunchArgument('all_points_topic', default_value='/all_points'),
        # DeclareLaunchArgument('clip_height', default_value='0.5'),
        # DeclareLaunchArgument('sensor_height', default_value='1.0'),
        # DeclareLaunchArgument('min_distance', default_value='0.01'),
        # DeclareLaunchArgument('max_distance', default_value='25.0'),
        # DeclareLaunchArgument('sensor_model', default_value='32'),
        # DeclareLaunchArgument('num_iter', default_value='3'),
        # DeclareLaunchArgument('num_lpr', default_value='20'),
        # DeclareLaunchArgument('th_seeds', default_value='1.2'),
        # DeclareLaunchArgument('th_dist', default_value='0.3'),


        #2023 ACCA Parameter
        # DeclareLaunchArgument('input_topic', default_value='/velodyne_points'),
        # DeclareLaunchArgument('no_ground_point_topic', default_value='/points_no_ground'),
        # DeclareLaunchArgument('ground_point_topic', default_value='/points_ground'),
        # DeclareLaunchArgument('all_points_topic', default_value='/all_points'),
        # DeclareLaunchArgument('clip_height', default_value='1.0'),
        # DeclareLaunchArgument('sensor_height', default_value='1.0'),
        # DeclareLaunchArgument('min_distance', default_value='0.01'),
        # DeclareLaunchArgument('max_distance', default_value='25.0'),
        # DeclareLaunchArgument('sensor_model', default_value='32'),
        # DeclareLaunchArgument('num_iter', default_value='3'),
        # DeclareLaunchArgument('num_lpr', default_value='20'),
        # DeclareLaunchArgument('th_seeds', default_value='1.2'),
        # DeclareLaunchArgument('th_dist', default_value='0.3'),

        # DeclareLaunchArgument('input_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument('input_topic', default_value='/cropbox_filtered'),
        DeclareLaunchArgument('no_ground_point_topic', default_value='/points_no_ground'),
        DeclareLaunchArgument('ground_point_topic', default_value='/points_ground'),
        DeclareLaunchArgument('all_points_topic', default_value='/all_points'),
        DeclareLaunchArgument('clip_height', default_value='3.5'),
        DeclareLaunchArgument('sensor_height', default_value='1.0'),
        DeclareLaunchArgument('min_distance', default_value='0.01'),
        DeclareLaunchArgument('max_distance', default_value='25.0'),
        DeclareLaunchArgument('sensor_model', default_value='32'),
        DeclareLaunchArgument('num_iter', default_value='3'),
        DeclareLaunchArgument('num_lpr', default_value='30'),
        DeclareLaunchArgument('th_seeds', default_value='1.5'),
        DeclareLaunchArgument('th_dist', default_value='0.03'),


        
        Node(
            package='plane_ground_filter',
            executable='plane_ground_filter_node',
            name='plane_ground_filter_node',
            output='screen',
            parameters=[
                {'input_topic': LaunchConfiguration('input_topic')},
                {'no_ground_point_topic': LaunchConfiguration('no_ground_point_topic')},
                {'ground_point_topic': LaunchConfiguration('ground_point_topic')},
                {'all_points_topic': LaunchConfiguration('all_points_topic')},
                {'sensor_height': LaunchConfiguration('sensor_height')},
                {'clip_height': LaunchConfiguration('clip_height')},
                {'min_distance': LaunchConfiguration('min_distance')},
                {'max_distance': LaunchConfiguration('max_distance')},
                {'sensor_model': LaunchConfiguration('sensor_model')},
                {'num_iter': LaunchConfiguration('num_iter')},
                {'num_lpr': LaunchConfiguration('num_lpr')},
                {'th_seeds': LaunchConfiguration('th_seeds')},
                {'th_dist': LaunchConfiguration('th_dist')}
            ]
        )
    ])
