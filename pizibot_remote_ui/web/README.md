# pizibot Remote UI — Frontend

Progressive Web App (PWA) built with **Vue 3 + Vite** for piloting and monitoring the pizibot robot from a smartphone. Communicates with ROS 2 exclusively via WebSocket (rosbridge) and HTTP (MJPEG stream).

---

## Quick start

```bash
npm install        # install dependencies (first time only)
npm run dev        # dev server with hot-reload on http://localhost:5173
npm run build      # production build → dist/
npm run preview    # preview the production build locally
```

After `npm run build`, the ROS launch file serves `dist/` automatically — no extra step needed when `colcon build --symlink-install` was used.

---

## Tech stack

| Library | Version | Role |
| ------- | ------- | ---- |
| Vue 3 | ^3.4 | Reactive UI framework (`<script setup>` composition API) |
| Vite | ^5.0 | Build tool and dev server |
| roslibjs | ^1.4 | WebSocket client for ROS 2 via rosbridge |
| nipplejs | ^0.10 | Virtual joystick (touch and mouse) |

---

## Project structure

```text
web/
├── COLCON_IGNORE           # prevents colcon from treating this as a ROS package
├── package.json
├── vite.config.js          # base: './' for relative asset paths
├── index.html              # entry point, registers the service worker
├── public/
│   ├── manifest.json       # PWA manifest (name, icons, display: standalone)
│   ├── sw.js               # service worker — passthrough, no cache
│   └── icons/
│       ├── icon-192.png    # PWA icon (home screen)
│       └── icon-512.png    # PWA icon (splash screen)
└── src/
    ├── main.js             # createApp + SW registration
    ├── App.vue             # root layout + telemetry toggle
    ├── ros.js              # ROS singleton, helpers
    ├── styles/
    │   └── main.css        # global reset, dark theme, touch overrides
    └── components/
        ├── Joystick.vue    # nipplejs joystick → /cmd_vel_web
        ├── CameraView.vue  # MJPEG <img> from web_video_server
        └── Telemetry.vue   # odom / joint_states / imu display
```

---

## Architecture

```
Browser
├── WebSocket ws://<host>:9090  ──►  rosbridge_websocket  ──►  ROS 2 topics
└── HTTP      http://<host>:8080 ──►  web_video_server    ──►  /camera/image_raw

Static files: http://<host>:3000  (python3 http.server serving dist/)
```

`location.hostname` is used everywhere so the app works without any configuration — just open `http://<robot-ip>:3000`.

---

## Source files

### `src/ros.js` — ROS singleton

Central module. Imported by every component that needs ROS.

```js
import { publishTwist, subscribeTopic } from './ros.js'
import ros from './ros.js'   // ROSLIB.Ros instance for event listeners
```

**Exports:**

| Export | Signature | Description |
| ------ | --------- | ----------- |
| `ros` (default) | `ROSLIB.Ros` | Shared connection instance |
| `publishTwist` | `(linear_x, angular_z) → void` | Publishes `TwistStamped` on `/cmd_vel_web` |
| `subscribeTopic` | `(name, type, cb, throttle_rate?) → ROSLIB.Topic` | Subscribes with server-side throttling |

**Auto-reconnect:** on WebSocket close, reconnects after 3 s indefinitely.

**`throttle_rate`** (ms): rosbridge throttles the topic on the server before sending frames over WebSocket. Default is 100 ms (10 Hz). Pass a higher value to reduce bandwidth:

```js
subscribeTopic('/odom', 'nav_msgs/Odometry', callback, 500)  // 2 Hz
```

---

### `src/components/Joystick.vue`

Virtual joystick using nipplejs in **static** mode (fixed position, does not follow the finger).

**Behavior:**

- `move` event → updates `lx` (linear X) and `az` (angular Z), starts a 15 Hz publish interval
- `end` event → stops the interval, publishes `(0, 0)` immediately
- `visibilitychange` (tab hidden) and `window blur` → same safety stop
- All listeners are removed in `onBeforeUnmount`

**Velocity mapping:**

```
linear_x  =  sin(angle) × force × MAX_LINEAR   (MAX_LINEAR  = 0.08 m/s)
angular_z = -cos(angle) × force × MAX_ANGULAR  (MAX_ANGULAR = 1.0 rad/s)
```

`force` is clamped to 1.0 (nipplejs raw value can exceed 1 for fast movements).

**Props:** none — values are internal constants. Edit `MAX_LINEAR`, `MAX_ANGULAR`, and `HZ` at the top of `<script setup>` to tune behavior.

---

### `src/components/CameraView.vue`

Displays the MJPEG stream from `web_video_server` as a plain `<img>` tag.

**Props:**

| Prop | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `host` | String | `location.hostname` | Robot IP |
| `port` | Number | `8080` | web_video_server port |
| `imageTopic` | String | `/camera/image_raw` | ROS image topic |
| `quality` | Number | `50` | MJPEG quality 1–100 |
| `width` | Number | `400` | Resize server-side (px) — reduces bandwidth |
| `framerate` | Number | `10` | Max FPS throttled by web_video_server |

**Generated URL:**

```
http://<host>:8080/stream?topic=<imageTopic>&quality=50&width=400&framerate=10
```

`web_video_server` handles encoding and throttling server-side — the browser simply displays a continuous JPEG stream.

On `@error` an overlay "Caméra indisponible" is shown; it clears on `@load`.

---

### `src/components/Telemetry.vue`

Subscribes to three ROS topics and displays live values in a table.

**Subscriptions (all at 2 Hz — `throttle_rate: 500`):**

| Topic | Type | Fields used |
| ----- | ---- | ----------- |
| `/odom` | `nav_msgs/Odometry` | `twist.twist.linear.x`, `twist.twist.angular.z` |
| `/joint_states` | `sensor_msgs/JointState` | `velocity[0]` (left), `velocity[1]` (right) |
| `/imu` | `sensor_msgs/Imu` | `angular_velocity.z` × 57.296 → °/s |

**Display:**

| Row | Unit |
| --- | ---- |
| Lin. | m/s |
| Ang. | rad/s |
| Roue G | rad/s |
| Roue D | rad/s |
| Gyro Z | °/s |

Values formatted with `fmt(v)`: 3 decimal places, leading space for positive values (monospace alignment).

**ROS status indicator:** shows connection state (`● ROS connecté` / `○ ROS déconnecté`) updated from `ros.on('connection' | 'close' | 'error')`.

All subscriptions are unsubscribed in `onBeforeUnmount`.

---

### `src/App.vue` — root layout

Vertical flex column filling `100dvh`:

```
┌──────────────────────────────┐
│  topbar: logo + toggle btn   │
├──────────────────────────────┤
│  CameraView (aspect-ratio    │
│             4/3, flex-shrink)│
├──────────────────────────────┤
│  control-row (flex: 1)       │
│  ┌─────────┐ ┌─────────────┐ │
│  │ Joystick│ │ Telemetry   │ │
│  │  (flex1)│ │ (v-if)      │ │
│  └─────────┘ └─────────────┘ │
└──────────────────────────────┘
```

The **"Télémétrie / Masquer"** button in the topbar toggles `showTelemetry` (ref). When `false`, `<Telemetry v-if="showTelemetry">` is unmounted — ROS subscriptions stop and the joystick takes the full width.

---

### `public/sw.js` — service worker

**Passthrough — no caching.** The service worker exists solely to satisfy the PWA installability requirement (browsers need a registered SW to show "Add to home screen").

```js
self.addEventListener('install', () => self.skipWaiting())
self.addEventListener('activate', (e) => {
  // Delete all old caches from previous versions
  e.waitUntil(caches.keys().then(...caches.delete...))
  self.clients.claim()
})
// No fetch handler → all requests go to the network normally
```

`skipWaiting()` and `clients.claim()` are safe here because there is no fetch interception — the new SW takes over immediately without disrupting any cached resources.

---

## PWA installation

On **Android Chrome**: tap the browser menu → "Ajouter à l'écran d'accueil". The app opens full-screen (no browser UI) in portrait orientation.

On **iOS Safari**: Share → "Sur l'écran d'accueil".

The installed app is indistinguishable from a native app and opens directly to `http://<robot-ip>:3000`.

---

## Tuning guide

| Goal | Where to change |
| ---- | --------------- |
| Joystick max speed | `MAX_LINEAR`, `MAX_ANGULAR` in `Joystick.vue` |
| Joystick publish rate | `HZ` in `Joystick.vue` (default 15) |
| Telemetry update rate | `throttle_rate` argument of each `subscribeTopic` call in `Telemetry.vue` (default 500 ms = 2 Hz) |
| Camera frame rate | `framerate` prop on `<CameraView>` in `App.vue` (default 10 fps) |
| Camera quality | `quality` prop on `<CameraView>` (1–100, default 50) |
| Camera resolution | `width` prop on `<CameraView>` — server resizes before encoding (default 400 px) |
| ROS host/port | Props on `<CameraView>` and `<Joystick>` — defaults to `location.hostname` |

---

## Build output

`npm run build` produces `dist/`:

```text
dist/
├── index.html
├── manifest.json
├── sw.js
├── icons/
│   ├── icon-192.png
│   └── icon-512.png
└── assets/
    ├── index-<hash>.js     # bundled JS (Vue + roslibjs + nipplejs)
    └── index-<hash>.css    # bundled CSS
```

Vite uses `base: './'` so all asset paths are relative — the app works when served from any subdirectory or directly opened as a file.

The ROS launch file (`remote_ui.launch.py`) resolves the `dist/` path via symlink when built with `colcon build --symlink-install`, so **no colcon rebuild is needed after `npm run build`**.
