<template>
  <div class="app">
    <header class="topbar">
      <span class="logo">pizibot</span>
      <span class="sub">Remote UI</span>
      <button class="telem-toggle" @click="showTelemetry = !showTelemetry">
        {{ showTelemetry ? 'Masquer' : 'Télémétrie' }}
      </button>
    </header>

    <CameraView class="camera" />

    <div class="control-row">
      <Joystick />
      <Telemetry v-if="showTelemetry" />
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import CameraView from './components/CameraView.vue'
import Joystick   from './components/Joystick.vue'
import Telemetry  from './components/Telemetry.vue'

const showTelemetry = ref(true)
</script>

<style scoped>
.app {
  display: flex;
  flex-direction: column;
  height: 100dvh;
  overflow: hidden;
}

.topbar {
  display: flex;
  align-items: baseline;
  gap: 8px;
  padding: 8px 16px;
  background: #16213e;
  flex-shrink: 0;
}

.logo {
  font-weight: 700;
  font-size: 1rem;
  color: #4fc3f7;
}

.sub {
  font-size: 0.75rem;
  color: rgba(255, 255, 255, 0.4);
  flex: 1;
}

.telem-toggle {
  padding: 4px 10px;
  font-size: 0.75rem;
  background: rgba(79, 195, 247, 0.15);
  color: #4fc3f7;
  border: 1px solid rgba(79, 195, 247, 0.4);
  border-radius: 6px;
  cursor: pointer;
  touch-action: manipulation;
}

.telem-toggle:active {
  background: rgba(79, 195, 247, 0.3);
}

.camera {
  flex-shrink: 0;
}

.control-row {
  display: flex;
  flex: 1;
  overflow: hidden;
}
</style>
