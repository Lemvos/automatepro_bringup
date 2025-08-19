import os
import ament_index_python.packages
import launch
import yaml
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration,  EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


enable_gnss_position = DeclareLaunchArgument(
    "enable_gnss_position", default_value=TextSubstitution(text="true")
)

enable_gnss_heading = DeclareLaunchArgument(
    "enable_gnss_heading", default_value=TextSubstitution(text="true")
)

enable_imu = DeclareLaunchArgument(
    "enable_imu", default_value=TextSubstitution(text="true")
)

enable_camera = DeclareLaunchArgument(
    "enable_cam1", default_value=TextSubstitution(text="true")
)

enable_camera = DeclareLaunchArgument(
    "enable_cam2", default_value=TextSubstitution(text="true")
)

enable_ntrip_client = DeclareLaunchArgument(
    "enable_ntrip_client", default_value=TextSubstitution(text="true")
)

enable_spartn_client = DeclareLaunchArgument(
    "enable_spartn_client", default_value=TextSubstitution(text="true")
)

config_dir = DeclareLaunchArgument(
    "config_dir", default_value=TextSubstitution(text="")
)


"""
If the config file exists in the config dir of the bringup package, load that.
else load the config file located in each package.
"""
def get_config_path(config_dir_path, pkg_name, config_file):

    bringup_config = os.path.join(config_dir_path, config_file)
    
    if os.path.exists(bringup_config):
        print(f'Param File: {bringup_config}')
        return bringup_config

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

def generate_f9p_base_node(config_dir_path):
    params = get_config_path(config_dir_path, 'ublox_gps', 'gnss_position_params.yaml')
    node = launch_ros.actions.Node(
        name='automatepro_gnss_position_node',
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[params],
        remappings=[
            ('automatepro_gnss_position_node/fix', '/sensor/gnss/position/fix'),
            ('automatepro_gnss_position_node/fix_velocity', '/sensor/gnss/position/fix_velocity'),
            ('automatepro_gnss_position_node/navpvt', '/sensor/gnss/position/navpvt'),
            ('monhw', '/sensor/gnss/position/monhw'),
            ('monsys', '/sensor/gnss/position/monsys'),
            ('nmea', '/sensor/gnss/position/nmea'),
            ('rtcm', '/sensor/gnss/correction'),
        ]
    )
    return node

def generate_f9h_rover_node(config_dir_path):
    params = get_config_path(config_dir_path, 'ublox_gps', 'gnss_heading_params.yaml')
    node = launch_ros.actions.Node(
        name='automatepro_gnss_heading_node',
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[params],
        remappings=[
            ('automatepro_gnss_heading_node/fix', '/sensor/gnss/heading/fix'),
            ('automatepro_gnss_heading_node/fix_velocity', '/sensor/gnss/heading/fix_velocity'),
            ('automatepro_gnss_heading_node/navpvt', '/sensor/gnss/heading/navpvt'),
            ('monhw', '/sensor/gnss/heading/monhw'),
            ('monsys', '/sensor/gnss/heading/monsys'),
            ('navrelposned', '/sensor/gnss/heading/navrelposned'),
            ('navheading', '/sensor/gnss/heading/true_heading'),
        ]
    )
    return node

def generate_imu_driver_node(config_dir_path):
    params = get_config_path(config_dir_path, 'automatepro_imu_driver', 'imu_params.yaml')
    node = Node(
        package='automatepro_imu_driver',  
        executable='bno08x_driver',  
        name='automatepro_imu_driver',
        output='screen',
        parameters=[params]
    )

    return node

def generate_cam1_node(config_dir_path):
    params = get_config_path(config_dir_path, 'automatepro_camera_driver', 'camera_params.yaml')
    with open(params, 'r') as file:
        data = yaml.safe_load(file)
        camera_name = data.get(
            'automatepro_cam1_node', {}).get(
            'ros__parameters', {}).get('camera_name', 'cam1')
    node = Node(
        package='automatepro_camera_driver',
        executable='camera_driver',
        output='both',
        name='automatepro_cam1_node',
        parameters=[params],
        remappings=[
            ('/camera/image_raw', f'/camera/{camera_name}/image_raw'),
            ('/camera/camera_info', f'/camera/{camera_name}/camera_info'),
            ('/camera/h264/video', f'/camera/{camera_name}/h264/video'),
            ('/camera/h264/calib', f'/camera/{camera_name}/h264/calib'),
        ]
    )

    return node

def generate_cam2_node(config_dir_path):
    params = get_config_path(config_dir_path, 'automatepro_camera_driver', 'camera_params.yaml')
    with open(params, 'r') as file:
        data = yaml.safe_load(file)
        camera_name = data.get(
            'automatepro_cam2_node', {}).get(
            'ros__parameters', {}).get('camera_name', 'cam2')
    node = Node( 
        package='automatepro_camera_driver',
        executable='camera_driver',
        output='both',
        name='automatepro_cam2_node',
        parameters=[params],
        remappings=[
            ('/camera/image_raw', f'/camera/{camera_name}/image_raw'),
            ('/camera/camera_info', f'/camera/{camera_name}/camera_info'),
            ('/camera/h264/video', f'/camera/{camera_name}/h264/video'),
            ('/camera/h264/calib', f'/camera/{camera_name}/h264/calib'),
        ]
    )

    return node

def generate_ntrip_client_node(config_dir_path):
    params = get_config_path(config_dir_path ,'automatepro_ntrip_client', 'ntrip_params.yaml')
    node = Node(
        package='ntrip_client',
        executable='ntrip_client',
        name='automatepro_ntrip_client',
        output='screen',
            parameters=[params],
            remappings=[
                ('rtcm', '/sensor/gnss/correction'),
            ],
        )

    return node

def generate_spartn_client_node(config_dir_path):
    params = get_config_path(config_dir_path ,'automatepro_spartn_client', 'spartn_params.yaml')
    node = Node(
        package='spartn_client',
        executable='spartn_client',
        name='automatepro_spartn_client',
        output='screen',
            parameters=[params],
            remappings=[
                ('spartn', '/sensor/gnss/correction'),
                ('nmea', '/sensor/gnss/position/nmea'),
            ],
        )

    return node

def configure_nodes(context, *args, **kwargs):
    nodes = []

    enable_gnss_position_value = LaunchConfiguration('enable_gnss_position').perform(context)
    enable_gnss_heading_value = LaunchConfiguration('enable_gnss_heading').perform(context)
    enable_imu_value = LaunchConfiguration('enable_imu').perform(context)
    enable_cam1_value = LaunchConfiguration('enable_cam1').perform(context)
    enable_cam2_value = LaunchConfiguration('enable_cam2').perform(context)
    enable_ntrip_client_value = LaunchConfiguration('enable_ntrip_client').perform(context)
    enable_spartn_client_value = LaunchConfiguration('enable_spartn_client').perform(context)
    config_dir_value = LaunchConfiguration('config_dir').perform(context)

    print("enable_gnss_position: ", enable_gnss_position_value)
    print("enable_gnss_heading: ", enable_gnss_heading_value)
    print("enable_imu: ", enable_imu_value)
    print("enable_cam1: ", enable_cam1_value)
    print("enable_cam2: ", enable_cam2_value)
    print("enable_ntrip_client: ", enable_ntrip_client_value)
    print("enable_spartn_client: ", enable_spartn_client_value)

    if enable_gnss_position_value == "true":
        nodes.append(generate_f9p_base_node(config_dir_value))
    if enable_gnss_heading_value == "true":
        nodes.append(generate_f9h_rover_node(config_dir_value))   
    if enable_imu_value == "true":
        nodes.append(generate_imu_driver_node(config_dir_value))
    if enable_cam1_value == "true":
        nodes.append(generate_cam1_node(config_dir_value))
    if enable_cam2_value == "true":
        nodes.append(generate_cam2_node(config_dir_value))
    if enable_ntrip_client_value == "true":
        nodes.append(generate_ntrip_client_node(config_dir_value))
    if enable_spartn_client_value == "true":
        nodes.append(generate_spartn_client_node(config_dir_value))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        enable_gnss_position,
        enable_gnss_heading,
        enable_imu,
        enable_camera,
        enable_ntrip_client,
        enable_spartn_client,
        config_dir,
        OpaqueFunction(function=configure_nodes),
    ])


