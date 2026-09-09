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
- `config_dir` (str, default: `""`): Directory holding the parameter files the nodes are launched with. When empty, each node falls back to the parameter file its own package installs.

## Configuration

The runtime parameter files are owned by the `automatepro-installer` `.deb`, which installs them on the host under `/opt/automatepro/config/ros`.
The core-driver compose service mounts that directory into the container and passes `config_dir:=/automatepro/config`, so every node reads the `.deb`-owned copy.
This package installs no configuration of its own.

Without `config_dir`, each node falls back to the parameter file its own package installs under `share/<package>/config`, which is the developer path.
Where a package installs that file under a different name from the one the `.deb` seeds, the launch file names both, so the fallback resolves for every node.

## Prerequisites

`enable_ntrip_client:=true` starts `automatepro_ntrip_client` through its own launch file, which reads the caster credentials from the environment rather than from `ntrip_params.yaml`. Export `NTRIP_USERNAME` and `NTRIP_PASSWORD` before launching, or the launch aborts.

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
