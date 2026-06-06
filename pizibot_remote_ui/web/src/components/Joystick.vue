<template>
  <div class="joystick-wrapper">
    <div ref="zone" class="joystick-zone" />
    <div class="joystick-hint">Joystick</div>
  </div>
</template>

<script setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import nipplejs from 'nipplejs'
import { publishTwist } from '../ros.js'

const MAX_LINEAR  = 0.4   // m/s
const MAX_ANGULAR = 1.2   // rad/s
const HZ = 15

const zone = ref(null)
let joystick = null
let publishInterval = null
let lx = 0
let az = 0

function startPublishing() {
  if (publishInterval) return
  publishInterval = setInterval(() => publishTwist(lx, az), 1000 / HZ)
}

function stopPublishing() {
  clearInterval(publishInterval)
  publishInterval = null
  lx = 0
  az = 0
  publishTwist(0, 0)
}

const onVisibilityChange = () => { if (document.hidden) stopPublishing() }

onMounted(() => {
  joystick = nipplejs.create({
    zone: zone.value,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#4fc3f7',
    size: 120,
  })

  joystick.on('move', (_, data) => {
    const force = Math.min(data.force, 1.0)
    const angle = data.angle.radian  // 0=right, π/2=up
    lx =  Math.sin(angle) * force * MAX_LINEAR
    az = -Math.cos(angle) * force * MAX_ANGULAR
    startPublishing()
  })

  joystick.on('end', stopPublishing)

  document.addEventListener('visibilitychange', onVisibilityChange)
  window.addEventListener('blur', stopPublishing)
})

onBeforeUnmount(() => {
  stopPublishing()
  joystick?.destroy()
  document.removeEventListener('visibilitychange', onVisibilityChange)
  window.removeEventListener('blur', stopPublishing)
})
</script>

<style scoped>
.joystick-wrapper {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  flex: 1;
}

.joystick-zone {
  width: 180px;
  height: 180px;
  position: relative;
  border-radius: 50%;
  background: rgba(255, 255, 255, 0.06);
  border: 2px solid rgba(79, 195, 247, 0.3);
  touch-action: none;
}

.joystick-hint {
  margin-top: 10px;
  font-size: 0.75rem;
  color: rgba(255, 255, 255, 0.4);
  text-transform: uppercase;
  letter-spacing: 0.1em;
}
</style>
