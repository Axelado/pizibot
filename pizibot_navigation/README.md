# Pizibot Navigation

Navigation and localization package for the Pizibot robot using Nav2, AMCL and Slam Toolbox.

**✅ Fully compatible with ROS 2 Jazzy and Gazebo Harmonic**

## Description

This package provides the complete navigation stack for the Pizibot robot, including:

- **Mapping**: Slam Toolbox for creating maps of the environment
- **Localization**: AMCL (Adaptive Monte Carlo Localization) for robot positioning
- **Sensor Fusion**: EKF (`robot_localization`) fusing wheel odometry and IMU into `/odom/filtered`, used during both mapping and localization
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
- **localization_launch.py**: AMCL and map server for localization, plus the EKF node
- **navigation_launch.py**: Nav2 navigation stack only
- **online_async_launch.py**: Online asynchronous SLAM mapping, plus the EKF node
- **ekf.launch.py**: EKF node (`robot_localization`) fusing wheel odometry and IMU
- **publish_initial_pose.launch.py**: Publishes initial robot pose for localization
- **rviz2.launch.py**: RViz2 with custom configurations

### Parameters

- **mapper_params_online_async.yaml**: Slam Toolbox configuration for online mapping
- **nav2_params.yaml**: Nav2 navigation stack parameters
- **ekf.yaml**: EKF (`robot_localization`) configuration — fuses wheel odometry and IMU into `/odom/filtered`

### Maps

Pre-built maps for different environments (`.pgm`, `.yaml`, `.posegraph` files):

- `home_1`: Home environment map
- `industrial-warehouse`: Industrial warehouse environment map
- `world_test1`: Test world 1 map
- `world_test2`: Test world 2 map

### RViz Configurations

- **localization.rviz**: Visualization for localization mode
- **mapping.rviz**: Visualization for mapping mode

### Nodes

- **initial_pose_publisher**: C++ node that waits for AMCL to subscribe to `/initialpose`, then publishes an initial pose for 3 seconds for quick AMCL initialization

## Usage

### Robot Mode (sim / real)

Both `full_mapping.launch.py` and `full_localization.launch.py` accept a `robot_mode` argument
(`sim` or `real`, default `sim`) that selects sensible defaults so you don't have to pass
`use_sim_time` (and, for localization, `map`/`initial_pose`) manually every time you switch
between simulation and the real robot:

| `robot_mode` | `use_sim_time` | `map` (localization only) | `initial_pose` (localization only) |
| ------------ | --------------- | -------------------------- | ------------------------------------ |
| `sim`        | `true`          | `industrial-warehouse.yaml` | `[0.0, 0.0, 0.0]`                     |
| `real`       | `false`         | `home_1.yaml`               | `[0.0, 0.0, -1.2566]`                 |

`use_sim_time`, `map`, and `initial_pose` can each still be overridden individually, regardless
of `robot_mode`. `map` accepts either a bare filename (resolved against the package's `map/`
directory) or a full/relative path.

### Mapping a New Environment

To create a new map of your environment:

```bash
# Launch your robot (simulation or real hardware)
ros2 launch pizibot_gz launch_sim.launch.py

# In another terminal, start mapping
ros2 launch pizibot_navigation full_mapping.launch.py

# Or, on the real robot:
ros2 launch pizibot_navigation full_mapping.launch.py robot_mode:=real
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
ros2 launch pizibot_navigation full_localization.launch.py

# Or, on the real robot, with a custom map and initial pose:
ros2 launch pizibot_navigation full_localization.launch.py \
  robot_mode:=real map:=industrial-warehouse.yaml initial_pose:="[1.0, 2.0, 0.0]"
```

The `initial_pose_publisher` node waits for AMCL to subscribe to `/initialpose` before publishing
the initial pose, so the robot is automatically localized and ready for navigation commands.

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
