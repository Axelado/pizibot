# Pizibot Navigation

Navigation and localization package for the Pizibot robot using Nav2, AMCL and Slam Toolbox.

**✅ Fully compatible with ROS 2 Jazzy and Gazebo Harmonic**

## Description

This package provides the complete navigation stack for the Pizibot robot, including:

- **Mapping**: Slam Toolbox for creating maps of the environment
- **Localization**: AMCL (Adaptive Monte Carlo Localization) for robot positioning
- **Path Planning**: Nav2 for autonomous navigation
- **Teleoperation**: Integration with joystick and keyboard teleop

## Features

- Online and offline SLAM mapping
- Autonomous navigation with obstacle avoidance
- Pre-mapped environment localization
- Multiple pre-configured launch files
- RViz visualization configurations
- Initial pose publisher for quick localization setup

## Package Contents

### Launch Files

- **full_localization.launch.py**: Complete localization setup with Nav2, teleop, and RViz
- **full_mapping.launch.py**: Complete mapping setup with Slam Toolbox, teleop, and RViz
- **localization_launch.py**: AMCL and map server for localization only
- **navigation_launch.py**: Nav2 navigation stack only
- **online_async_launch.py**: Online asynchronous SLAM mapping
- **publish_initial_pose.launch.py**: Publishes initial robot pose for localization
- **rviz2.launch.py**: RViz2 with custom configurations

### Parameters

- **mapper_params_online_async.yaml**: Slam Toolbox configuration for online mapping
- **nav2_params.yaml**: Nav2 navigation stack parameters

### Maps

Pre-built maps for different environments (`.pgm`, `.yaml`, `.posegraph` files):

- `home_1`: Home environment map
- `world_test1`: Test world map

### RViz Configurations

- **localization.rviz**: Visualization for localization mode
- **mapping.rviz**: Visualization for mapping mode

### Nodes

- **initial_pose_publisher**: C++ node that publishes an initial pose to `/initialpose` topic for quick AMCL initialization

## Usage

### Mapping a New Environment

To create a new map of your environment:

```bash
# Launch your robot (simulation or real hardware)
ros2 launch pizibot_gz launch_sim.launch.py

# In another terminal, start mapping
ros2 launch pizibot_navigation full_mapping.launch.py
```

Navigate the robot using teleoperation to explore the environment. When done, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Localization and Navigation

To navigate in a known environment:

```bash
# Launch your robot (simulation or real hardware)
ros2 launch pizibot_gz launch_sim.launch.py

# In another terminal, start localization and navigation
ros2 launch pizibot_navigation full_localization.launch.py map:=/path/to/your/map.yaml
```

The robot will automatically set its initial pose and be ready for navigation commands.

### Publishing Initial Pose Manually

If you need to manually set the robot's initial pose:

```bash
ros2 launch pizibot_navigation publish_initial_pose.launch.py initial_pose:="[x, y, yaw]"
```

Example:

```bash
ros2 launch pizibot_navigation publish_initial_pose.launch.py initial_pose:="[0.0, 0.0, 0.0]"
```

## Dependencies

This package depends on:

- `rclcpp`: ROS 2 C++ client library
- `geometry_msgs`: ROS 2 geometry message definitions
- `nav2_bringup`: Nav2 navigation stack
- `slam_toolbox`: SLAM implementation
- `pizibot_teleop`: Teleoperation package

## Important Notes

⚠️ **You must launch the simulation (Gazebo) or start the real robot before running the navigation launch files.** The navigation launch files do NOT start the robot simulation or hardware drivers.

## Author

Axel NIATO - <axelniato@gmail.com>

## License

Apache-2.0
