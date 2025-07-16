import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # cropbox 실행
        launch_ros.actions.Node(
            package='crop',
            executable='cropbox',
            name='cropbox_filter',
            output='screen'
        ),

        # z값 하나로 통일시키기
        launch_ros.actions.Node(
            package='crop',
            executable='z_crop',
            name='z_fixed_pointcloud',
            output='screen'
        ),

        # 라인만 클러스터링 하기
        launch_ros.actions.Node(
            package='crop',
            executable='line_detection',
            name='lidar_reflectivity_filter',
            output='screen'
        ),

        # 클러스터링된 점들 기준으로 라인 그리기
        launch_ros.actions.Node(
            package='crop',
            executable='draw_line',
            name='draw_line_filter',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='no_gps_pth',
            executable='costmap',
            name='costmap',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='no_gps_pth',
            executable='dwa',
            name='dwa',
            output='screen'
        )
    ])
