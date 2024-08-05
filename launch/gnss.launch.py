import os
import ament_index_python.packages
import launch
import launch_ros.actions

def get_config_path(pkg_name, config_file):
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(pkg_name),
        'config')
    return os.path.join(config_directory, config_file)

def generate_f9p_base_node():
    params = get_config_path('automatepro_bringup', 'gnss_base_params.yaml')
    node = launch_ros.actions.Node(
        name='automatepro_gnss_base_node',
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[params],
        remappings=[
            ('automatepro_gnss_base_node/fix', '/sensor/gnss/position/fix'),
            ('automatepro_gnss_base_node/fix_velocity', '/sensor/gnss/position/fix_velocity'),
            ('monhw', '/sensor/gnss/position/monhw'),
            ('rtcm', '/sensor/gnss/correction/rtcm')
        ]
    )
    return node

def generate_f9h_rover_node():
    params = get_config_path('automatepro_bringup', 'gnss_rover_params.yaml')
    node = launch_ros.actions.Node(
        name='automatepro_gnss_rover_node',
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[params],
        remappings=[
            ('automatepro_gnss_rover_node/fix', '/sensor/gnss/heading/fix'),
            ('automatepro_gnss_rover_node/fix_velocity', '/sensor/gnss/heading/fix_velocity'),
            ('monhw', '/sensor/gnss/heading/monhw'),
            ('navrelposned', '/sensor/gnss/heading/navrelposned'),
            ('navheading', '/sensor/gnss/heading/true_heading')
        ]
    )
    return node

def generate_launch_description():
    f9p_base_node = generate_f9p_base_node()
    f9h_rover_node = generate_f9h_rover_node()

    return launch.LaunchDescription([
        f9p_base_node,
        f9h_rover_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=f9p_base_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=f9h_rover_node,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
