# AutomatePro Bringup

This package is used to bring up the ROS 2 nodes of the AutomatePro.

## Arguments
- `enable_ntrip_client` (bool, default: `true`): Enable NTRIP client node.
- `enable_gnss` (bool, default: `true`): Enable GNSS node.
- `enable_imu` (bool, default: `true`): Enable IMU node.
- `enable_cam1` (bool, default: `true`): Enable Camera 1 node.
- `enable_cam2` (bool, default: `true`): Enable Camera 2 node.
- `config_dir` (str, default: `""`): Path to the configuration directory. (Optional) Only required if you want to override the configuration files.

## Usage
```bash
ros2 launch automatepro_bringup core_driver.launch.py \
 enable_ntrip_client:=true \
 enable_gnss:=true \
 enable_imu:=true \
 enable_cam1:=true \
 enable_cam2:=true \
 config_dir:="/path/to/config/dir"
```
