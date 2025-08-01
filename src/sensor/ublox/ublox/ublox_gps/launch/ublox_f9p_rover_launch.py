"""Launch the ublox gps node with c94-m8p configuration."""
import os

import ament_index_python.packages
import launch
import launch_ros.actions

def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config'
    )
    params = os.path.join(config_directory, 'zed_f9p_rover.yaml')
    ublox_gps_node = launch_ros.actions.Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[params],
        # remappings=[
        #     ('/ublox_gps_node/fix', '/smc_plus/fix'),
        #     ('/rtcm', '/smc_plus/rtcm'),
        #     ('/ublox_gps_node/fix_velocity', '/smc_plus/fix_velocity'),
        #     ('/navrelposned', '/smc_plus/relposned'),
        # ],
    )

    return launch.LaunchDescription([
        ublox_gps_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=ublox_gps_node,
                on_exit=[
                    launch.actions.EmitEvent(
                        event=launch.events.Shutdown(),
                    ),
                ],
            ),
        ),
    ])
