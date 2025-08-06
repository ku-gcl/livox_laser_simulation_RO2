# Livox Laser Simulation for Gazebo Harmonic

This package has been updated to work with Gazebo Harmonic (formerly Ignition Gazebo). 

## Changes Made

### Dependencies
- Updated from Gazebo Classic to Gazebo Harmonic
- Changed `gazebo`, `gazebo_ros`, `gazebo_dev` → `ros_gz_sim`
- Added Gazebo Harmonic libraries: `gz-sim8`, `gz-plugin2`, `gz-sensors8`, `gz-msgs10`, `gz-transport13`

### Plugin Architecture
- Completely rewritten the plugin to use Gazebo Harmonic's System architecture
- Old files are backed up with `.bak` extension
- New plugin: `LivoxHarmonicPlugin` replaces the old `LivoxPointsPlugin`

### URDF Updates
- Created new `mid360_harmonic.xacro` for Gazebo Harmonic
- Uses `gpu_lidar` sensor type instead of `ray`
- Updated plugin configuration syntax

## Installation

### Prerequisites
Make sure you have Gazebo Harmonic installed:

```bash
# Ubuntu 22.04
sudo apt install gz-harmonic
```

### Build the Package

```bash
cd /path/to/your/workspace
colcon build --packages-select ros2_livox_simulation
source install/setup.bash
```

## Usage

### Launch Example

```bash
ros2 launch ros2_livox_simulation livox_harmonic_example.launch.py
```

### Available URDF Macros

- `mid360_harmonic` - Updated for Gazebo Harmonic

### Topics Published

- `/{topic}_PointCloud2` - sensor_msgs/PointCloud2
- `/{topic}` - livox_ros_driver2/CustomMsg (Livox format)

## Configuration Parameters

In your URDF/xacro file, you can configure:

- `csv_file_name` - Path to the CSV file with scan pattern
- `topic` - Base topic name for publishing
- `samples` - Number of sample points
- `downsample` - Downsampling factor

## Migration from Gazebo Classic

If you were using the old version with Gazebo Classic:

1. Update your URDF files to use the new `*_harmonic.xacro` macros
2. Change sensor type from `ray` to `gpu_lidar`
3. Update plugin filename and configuration
4. Use `ros2 launch` instead of `roslaunch`

## Backup Files

The original Gazebo Classic files are preserved with `.bak` extensions:
- `livox_points_plugin.cpp.bak`
- `livox_ode_multiray_shape.cpp.bak`
- `livox_points_plugin.h.bak`
- `livox_ode_multiray_shape.h.bak`

## Troubleshooting

### Plugin Not Loading
Make sure the library is properly built and installed:
```bash
colcon build --packages-select ros2_livox_simulation
```

### CSV File Not Found
Ensure the CSV file path in your URDF is correct:
```xml
<csv_file_name>$(find ros2_livox_simulation)/scan_mode/mid360.csv</csv_file_name>
```

### Gazebo Harmonic Not Starting
Check that Gazebo Harmonic is properly installed and sourced:
```bash
gz sim --version
```