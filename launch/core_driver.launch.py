import os
import ament_index_python.packages
import yaml
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetRemap


enable_gnss_position = DeclareLaunchArgument(
    "enable_gnss_position", default_value=TextSubstitution(text="true")
)

enable_gnss_heading = DeclareLaunchArgument(
    "enable_gnss_heading", default_value=TextSubstitution(text="true")
)

enable_imu = DeclareLaunchArgument(
    "enable_imu", default_value=TextSubstitution(text="true")
)

enable_cam1 = DeclareLaunchArgument(
    "enable_cam1", default_value=TextSubstitution(text="true")
)

enable_cam2 = DeclareLaunchArgument(
    "enable_cam2", default_value=TextSubstitution(text="true")
)

enable_ntrip_client = DeclareLaunchArgument(
    "enable_ntrip_client", default_value=TextSubstitution(text="true")
)

enable_spartn_client = DeclareLaunchArgument(
    "enable_spartn_client", default_value=TextSubstitution(text="true")
)

enable_driver_manager = DeclareLaunchArgument(
    "enable_driver_manager", default_value=TextSubstitution(text="true")
)

config_dir = DeclareLaunchArgument(
    "config_dir", default_value=TextSubstitution(text="")
)


def get_config_path(config_dir_path, pkg_name, config_file, pkg_config_file=None):
    """
    Resolve the parameter file of one node.

    The file in config_dir_path wins, then the one in the automatepro_bringup share
    config directory, then the one in the config directory of pkg_name. pkg_config_file
    names the file on that last step, for a package whose own copy is named differently
    from the copy seeded in config_dir_path, and defaults to config_file.

    Raises FileNotFoundError naming pkg_name and the path looked for, so a missing file
    fails here rather than inside rcl as a YAML parse error.
    """
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

    package_config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(pkg_name),
        'config')
    package_config = os.path.join(
        package_config_directory, pkg_config_file or config_file)

    if not os.path.exists(package_config):
        raise FileNotFoundError(
            f'{pkg_name} ships no parameter file at {package_config}')

    return package_config


def get_camera_log_level(params, node_name):
    """
    Read logging.level for one camera out of the parameter file.

    The camera driver does not declare logging.level as a ROS parameter, so it only
    takes effect as --log-level on the command line. Scoping it to the node's own
    logger keeps the RMW and DDS loggers at their defaults.
    """
    with open(params, 'r') as file:
        data = yaml.safe_load(file)

    return data.get(node_name, {}).get(
        'ros__parameters', {}).get('logging', {}).get('level', 'info')

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
    params = get_config_path(
        config_dir_path, 'automatepro_camera_driver', 'camera_params.yaml',
        'config.yaml')
    log_level = get_camera_log_level(params, 'automatepro_cam1_node')
    node = Node(
        package='automatepro_camera_driver',
        executable='camera_driver',
        output='both',
        name='automatepro_cam1_node',
        parameters=[params],
        arguments=['--ros-args', '--log-level', f'automatepro_cam1_node:={log_level}'],
    )

    return node

def generate_cam2_node(config_dir_path):
    params = get_config_path(
        config_dir_path, 'automatepro_camera_driver', 'camera_params.yaml',
        'config.yaml')
    log_level = get_camera_log_level(params, 'automatepro_cam2_node')
    node = Node(
        package='automatepro_camera_driver',
        executable='camera_driver',
        output='both',
        name='automatepro_cam2_node',
        parameters=[params],
        arguments=['--ros-args', '--log-level', f'automatepro_cam2_node:={log_level}'],
    )

    return node

def generate_ntrip_client_launch(config_dir_path):
    params = get_config_path(
        config_dir_path, 'automatepro_ntrip_client', 'ntrip_params.yaml',
        'params.yaml')
    launch_file = os.path.join(
        ament_index_python.packages.get_package_share_directory('automatepro_ntrip_client'),
        'launch',
        'automatepro_ntrip_client.launch.py')

    # The client publishes on an absolute /rtcm and its launch file exposes no
    # remapping argument, so the rule is set on the context instead. SetRemap
    # reaches the composable node through LoadComposableNodes, which a remapping
    # passed to IncludeLaunchDescription would not.
    group = GroupAction([
        SetRemap('/rtcm', '/sensor/gnss/correction'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'params_file': params}.items(),
        ),
    ], scoped=True)

    return group

def generate_spartn_client_node(config_dir_path):
    params = get_config_path(config_dir_path, 'spartn_client', 'spartn_params.yaml')
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

def generate_driver_manager_node(config_dir_path):
    params = get_config_path(
        config_dir_path, 'automatepro_driver_manager', 'driver_manager_params.yaml',
        'driver_manager.yaml')
    node = Node(
        package='automatepro_driver_manager',
        executable='automatepro_driver_manager_node',
        name='automatepro_driver_manager',
        output='screen',
        parameters=[params]
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
    enable_driver_manager_value = LaunchConfiguration('enable_driver_manager').perform(context)
    config_dir_value = LaunchConfiguration('config_dir').perform(context)

    print("enable_gnss_position: ", enable_gnss_position_value)
    print("enable_gnss_heading: ", enable_gnss_heading_value)
    print("enable_imu: ", enable_imu_value)
    print("enable_cam1: ", enable_cam1_value)
    print("enable_cam2: ", enable_cam2_value)
    print("enable_ntrip_client: ", enable_ntrip_client_value)
    print("enable_spartn_client: ", enable_spartn_client_value)
    print("enable_driver_manager: ", enable_driver_manager_value)

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
        nodes.append(generate_ntrip_client_launch(config_dir_value))
    if enable_spartn_client_value == "true":
        nodes.append(generate_spartn_client_node(config_dir_value))
    if enable_driver_manager_value == "true":
        nodes.append(generate_driver_manager_node(config_dir_value))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        enable_gnss_position,
        enable_gnss_heading,
        enable_imu,
        enable_cam1,
        enable_cam2,
        enable_ntrip_client,
        enable_spartn_client,
        enable_driver_manager,
        config_dir,
        OpaqueFunction(function=configure_nodes),
    ])


