import os
import ament_index_python.packages
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
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

def generate_f9h_rover_node():
    os.getenv()
    params = get_config_path('ublox_gps', 'gnss_rover_params.yaml')
    node = launch_ros.actions.Node(name='ublox_gps_rover_node',
                                   package='ublox_gps',
                                   executable='ublox_gps_node',
                                   output='both',
                                   parameters=[params])

    return node

def generate_f9p_base_node():
    params = get_config_path('ublox_gps', 'gnss_base_params.yaml')
    node = launch_ros.actions.Node(name='ublox_gps_base_node',
                                   package='ublox_gps',
                                   executable='ublox_gps_node',
                                   output='both',
                                   parameters=[params])

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
                parameters=params
            )
        ]
    )

    return [ container]

def generate_launch_description():
    nodes = []

    if LaunchConfiguration('enable_gnss'):
        nodes.append(generate_f9h_rover_node())
        nodes.append(generate_f9p_base_node())
    if LaunchConfiguration('enable_imu'):
        nodes.append(generate_imu_driver_node())
    if LaunchConfiguration('enable_camera'):
        nodes.append(generate_camera_node())
    if LaunchConfiguration('enable_ntrip_client'):
        nodes.extend(generate_ntrip_client_node())

    return LaunchDescription([
        enable_gnss,
        enable_imu,
        enable_camera,
        enable_ntrip_client,
        *nodes,
    ])
