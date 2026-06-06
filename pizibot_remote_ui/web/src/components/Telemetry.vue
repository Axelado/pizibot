<template>
  <div class="telemetry">
    <h2 class="title">Télémétrie</h2>
    <table>
      <tbody>
        <tr>
          <td class="label">Lin. (m/s)</td>
          <td class="value">{{ fmt(odom.linear_x) }}</td>
        </tr>
        <tr>
          <td class="label">Ang. (rad/s)</td>
          <td class="value">{{ fmt(odom.angular_z) }}</td>
        </tr>
        <tr>
          <td class="label">Roue G (rad/s)</td>
          <td class="value">{{ fmt(wheels.left) }}</td>
        </tr>
        <tr>
          <td class="label">Roue D (rad/s)</td>
          <td class="value">{{ fmt(wheels.right) }}</td>
        </tr>
        <tr>
          <td class="label">Gyro Z (°/s)</td>
          <td class="value">{{ fmt(imu.gyro_z * 57.296) }}</td>
        </tr>
      </tbody>
    </table>
    <div class="ros-status" :class="rosState">
      {{ rosState === 'connected' ? '● ROS connecté' : '○ ROS déconnecté' }}
    </div>
  </div>
</template>

<script setup>
import { reactive, ref, onMounted, onBeforeUnmount } from 'vue'
import { subscribeTopic } from '../ros.js'
import ros from '../ros.js'

const odom   = reactive({ linear_x: 0, angular_z: 0 })
const wheels = reactive({ left: 0, right: 0 })
const imu    = reactive({ gyro_z: 0 })
const rosState = ref('disconnected')

let subs = []

function fmt(v) { return (v >= 0 ? ' ' : '') + v.toFixed(3) }

onMounted(() => {
  ros.on('connection', () => { rosState.value = 'connected' })
  ros.on('close',      () => { rosState.value = 'disconnected' })
  ros.on('error',      () => { rosState.value = 'disconnected' })

  subs.push(subscribeTopic('/odom', 'nav_msgs/Odometry', (msg) => {
    odom.linear_x  = msg.twist.twist.linear.x
    odom.angular_z = msg.twist.twist.angular.z
  }, 500))

  subs.push(subscribeTopic('/joint_states', 'sensor_msgs/JointState', (msg) => {
    if (msg.velocity.length >= 2) {
      wheels.left  = msg.velocity[0]
      wheels.right = msg.velocity[1]
    }
  }, 500))

  subs.push(subscribeTopic('/imu', 'sensor_msgs/Imu', (msg) => {
    imu.gyro_z = msg.angular_velocity.z
  }, 500))
})

onBeforeUnmount(() => {
  subs.forEach((t) => t.unsubscribe())
})
</script>

<style scoped>
.telemetry {
  padding: 12px 16px;
  background: rgba(255, 255, 255, 0.04);
  border-radius: 12px;
  margin: 8px;
}

.title {
  font-size: 0.8rem;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  color: rgba(255, 255, 255, 0.5);
  margin: 0 0 8px;
}

table {
  width: 100%;
  border-collapse: collapse;
}

td {
  padding: 4px 0;
  font-size: 0.85rem;
}

.label {
  color: rgba(255, 255, 255, 0.5);
  width: 55%;
}

.value {
  font-family: monospace;
  color: #4fc3f7;
  text-align: right;
}

.ros-status {
  margin-top: 10px;
  font-size: 0.75rem;
  text-align: center;
}

.ros-status.connected    { color: #66bb6a; }
.ros-status.disconnected { color: #ef5350; }
</style>
