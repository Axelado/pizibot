# pizibot_remote_ui

Progressive Web App (PWA) for piloting and monitoring the pizibot robot from a smartphone, with no app installation required.

## Features

- **Virtual joystick** — nipplejs, publishes `geometry_msgs/TwistStamped` on `/cmd_vel_web` at 15 Hz
- **Live camera** — MJPEG stream via `web_video_server` (15 fps, quality 50, width 400 px)
- **Telemetry panel** — linear velocity, angular velocity, wheel speeds, gyro Z at 2 Hz; toggle button to hide/show
- **ROS status indicator** — live WebSocket connection state
- **PWA installable** — "Add to home screen" on Android/iOS

## Architecture

```
Smartphone (browser)
  ├── WebSocket :9090  ←→  rosbridge_websocket  ←→  ROS 2
  └── HTTP      :8080  ←→  web_video_server     ←→  /camera/image_raw

Static files served by python3 http.server on :3000
```

## Usage

### Launch (standalone)

```bash
ros2 launch pizibot_remote_ui remote_ui.launch.py
```

Open `http://<robot-ip>:3000` on any device on the same LAN.

### Launch with the real robot

```bash
ros2 launch pizibot_hardware launch_real_robot.launch.py enable_remote_ui:=true
```

### Launch with simulation

```bash
ros2 launch pizibot_gz launch_sim.launch.py enable_remote_ui:=true
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `image_topic` | `/camera/image_raw` | ROS topic served by `web_video_server` |

## Development Workflow

```bash
cd pizibot_remote_ui/web
npm install          # first time only
npm run build        # compile → web/dist/
# no colcon rebuild needed (launch auto-resolves source via symlink)
ros2 launch pizibot_remote_ui remote_ui.launch.py
```

When `colcon build --symlink-install` is used, the launch file resolves to the source tree via symlink and serves `web/dist/` directly. A colcon rebuild is only needed if the ROS package structure changes (new launch files, new params).

## ROS Interface

### Published topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/cmd_vel_web` | `geometry_msgs/TwistStamped` | 15 Hz | Velocity commands from the joystick |

### Subscribed topics (via rosbridge)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | 2 Hz | Linear and angular velocity display |
| `/joint_states` | `sensor_msgs/JointState` | 2 Hz | Left and right wheel speeds |
| `/imu` | `sensor_msgs/Imu` | 2 Hz | Gyro Z (converted to °/s) |
| `/camera/image_raw` | `sensor_msgs/Image` | 15 fps | MJPEG stream via HTTP |

### twist_mux integration

The `/cmd_vel_web` topic is registered in `twist_mux` with priority 20 (between joystick 21 and navigation 19). The source has a 0.5 s timeout — the robot stops automatically if the WebSocket connection is lost.

## Ports

| Port | Service |
|------|---------|
| 3000 | HTTP — static PWA files |
| 9090 | WebSocket — rosbridge (ROS ↔ browser) |
| 8080 | HTTP — MJPEG video stream |

## Package Structure

```
pizibot_remote_ui/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── remote_ui.launch.py       # rosbridge + web_video_server + http.server
├── params/
│   └── rosbridge.yaml            # port 9090, address 0.0.0.0
└── web/                          # Vue 3 + Vite frontend
    ├── COLCON_IGNORE
    ├── package.json
    ├── vite.config.js
    ├── public/
    │   ├── manifest.json         # PWA manifest
    │   ├── sw.js                 # Service worker (passthrough, no cache)
    │   └── icons/
    │       ├── icon-192.png
    │       └── icon-512.png
    └── src/
        ├── main.js
        ├── App.vue               # Layout + telemetry toggle
        ├── ros.js                # ROSLIB singleton, auto-reconnect, helpers
        └── components/
            ├── Joystick.vue      # nipplejs static joystick → /cmd_vel_web
            ├── CameraView.vue    # MJPEG <img> from web_video_server
            └── Telemetry.vue     # odom / joint_states / imu display
```

## Dependencies

### ROS 2

- `rosbridge_server` — WebSocket bridge
- `web_video_server` — MJPEG HTTP stream

### JavaScript (bundled at build time)

- `vue` 3 — reactive UI framework
- `roslib` — ROSLIB.js WebSocket client
- `nipplejs` — virtual joystick

## Author

Axel NIATO <axelniato@gmail.com>

## License

Apache-2.0
