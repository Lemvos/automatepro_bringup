# AutomatePro Bringup

This package is used to bring up the ROS 2 nodes of the AutomatePro.

## Arguments
- `enable_gnss_position` (bool, default: `true`): Enable GNSS position node.
- `enable_gnss_heading` (bool, default: `true`): Enable GNSS heading node.
- `enable_imu` (bool, default: `true`): Enable IMU node.
- `enable_cam1` (bool, default: `true`): Enable Camera 1 node.
- `enable_cam2` (bool, default: `true`): Enable Camera 2 node.
- `enable_ntrip_client` (bool, default: `true`): Enable NTRIP client node.
- `enable_spartn_client` (bool, default: `true`): Enable SPARTN client node.
- `enable_driver_manager` (bool, default: `true`): Enable Driver Manager node.
- `config_dir` (str, default: `""`): Optional path to a configuration directory to override default config files.

## Usage
```bash
ros2 launch automatepro_bringup core_driver.launch.py \
	enable_gnss_position:=true \
	enable_gnss_heading:=true \
	enable_imu:=true \
	enable_cam1:=true \
	enable_cam2:=true \
	enable_ntrip_client:=true \
	enable_spartn_client:=true \
	enable_driver_manager:=true \
	config_dir:="/path/to/config/dir"
```
