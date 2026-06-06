import ROSLIB from 'roslib'

const ros = new ROSLIB.Ros({
  url: `ws://${location.hostname}:9090`,
})

ros.on('connection', () => console.log('[ROS] Connected to rosbridge'))
ros.on('error', (e) => console.error('[ROS] Error:', e))
ros.on('close', () => {
  console.warn('[ROS] Connection closed — retrying in 3 s')
  setTimeout(() => ros.connect(`ws://${location.hostname}:9090`), 3000)
})

export const cmdVelWeb = new ROSLIB.Topic({
  ros,
  name: '/cmd_vel_web',
  messageType: 'geometry_msgs/TwistStamped',
})

export function publishTwist(linear_x, angular_z) {
  cmdVelWeb.publish(new ROSLIB.Message({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
    twist: {
      linear:  { x: linear_x, y: 0.0, z: 0.0 },
      angular: { x: 0.0,      y: 0.0, z: angular_z },
    },
  }))
}

// throttle_rate (ms): rosbridge throttles on the server side before sending.
// 100 ms = 10 Hz max — enough for telemetry display, avoids flooding mobile WS.
export function subscribeTopic(name, messageType, callback, throttle_rate = 100) {
  const topic = new ROSLIB.Topic({ ros, name, messageType, throttle_rate })
  topic.subscribe(callback)
  return topic
}

export default ros
