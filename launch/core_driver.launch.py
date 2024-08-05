import os
import ament_index_python.packages
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration,  EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


enable_gnss = DeclareLaunchArgument(
    "enable_gnss", default_value=TextSubstitution(text="true")
)

enable_imu = DeclareLaunchArgument(
    "enable_imu", default_value=TextSubstitution(text="true")
)

enable_camera = DeclareLaunchArgument(
    "enable_camera", default_value=TextSubstitution(text="true")
)

enable_ntrip_client = DeclareLaunchArgument(
    "enable_ntrip_client", default_value=TextSubstitution(text="true")
)


"""
If the config file exists in the config dir of the bringup package, load that.
else load the config file located in each package.
"""
def get_config_path(pkg_name, config_file):
    bringup_config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('automatepro_bringup'),
        'config')
    bringup_config = os.path.join(bringup_config_directory, config_file)
    
    if os.path.exists(bringup_config):
        return bringup_config
    else:
        package_config_directory = os.path.join(
            ament_index_python.packages.get_package_share_directory(pkg_name),
            'config')
        return os.path.join(package_config_directory, config_file)

def generate_f9p_base_node():
    params = get_config_path('ublox_gps', 'gnss_base_params.yaml')
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
    params = get_config_path('ublox_gps', 'gnss_rover_params.yaml')
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

def generate_imu_driver_node():
    params = get_config_path('automatepro_imu_driver', 'imu_params.yaml')
    node = Node(
        package='automatepro_imu_driver',  
        executable='imu_driver',  
        name='imu_driver',
        output='screen',
        parameters=[params]
    )

    return node

def generate_camera_node():
    params = get_config_path('automatepro_camera_driver', 'camera_params.yaml')
    node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        name='camera_node',
        parameters=[
            params,
            {   
                'video_device': '/dev/video0',
                'ffmpeg_image_transport.profile': 'main',
                'ffmpeg_image_transport.preset': 'ultrafast',
                'ffmpeg_image_transport.gop': 15,
            },
        ],
    )

    return node

def generate_ntrip_client_node():
    params = get_config_path('ntrip_client', 'ntrip_params.yaml')
    container = ComposableNodeContainer(
        name='ntrip_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ntrip_client',
                plugin='ntrip_client::NTRIPClientNode',
                name='ntrip_client',
                parameters=[params],
                remappings=[
                    ('rtcm', '/sensor/gnss/correction/rtcm'),
                ],
            )
        ]
    )

    return [ container]

def configure_nodes(context, *args, **kwargs):
    nodes = []

    enable_gnss_value = LaunchConfiguration('enable_gnss').perform(context)
    enable_imu_value = LaunchConfiguration('enable_imu').perform(context)
    enable_camera_value = LaunchConfiguration('enable_camera').perform(context)
    enable_ntrip_client_value = LaunchConfiguration('enable_ntrip_client').perform(context)

    print("enable_gnss: ", enable_gnss_value)
    print("enable_imu: ", enable_imu_value)
    print("enable_camera: ", enable_camera_value)
    print("enable_ntrip_client: ", enable_ntrip_client_value)

    if enable_gnss_value == "true":
        nodes.append(generate_f9h_rover_node())
        nodes.append(generate_f9p_base_node())
    if enable_imu_value == "true":
        nodes.append(generate_imu_driver_node())
    if enable_camera_value == "true":
        nodes.append(generate_camera_node())
    if enable_ntrip_client_value == "true":
        nodes.extend(generate_ntrip_client_node())

    return nodes

def generate_launch_description():
    return LaunchDescription([
        enable_gnss,
        enable_imu,
        enable_camera,
        enable_ntrip_client,
        OpaqueFunction(function=configure_nodes),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=generate_f9p_base_node(),
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=generate_f9h_rover_node(),
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])

