<template>
  <div class="camera-wrapper">
    <img
      :src="streamUrl"
      class="camera-feed"
      alt="Flux caméra pizibot"
      @error="onError"
      @load="onLoad"
    />
    <div v-if="error" class="camera-overlay error">
      Caméra indisponible
    </div>
  </div>
</template>

<script setup>
import { computed, ref } from 'vue'

const props = defineProps({
  host:       { type: String, default: () => location.hostname },
  port:       { type: Number, default: 8080 },
  imageTopic: { type: String, default: '/camera/image_raw' },
  quality:   { type: Number, default: 50 },   // MJPEG quality 1-100
  width:     { type: Number, default: 400 },  // resize server-side → less bandwidth
  framerate: { type: Number, default: 10 },   // max FPS throttled by web_video_server
})

const error = ref(false)
const streamUrl = computed(() =>
  `http://${props.host}:${props.port}/stream?topic=${props.imageTopic}` +
  `&quality=${props.quality}&width=${props.width}&framerate=${props.framerate}`
)

function onError() { error.value = true }
function onLoad()  { error.value = false }
</script>

<style scoped>
.camera-wrapper {
  position: relative;
  width: 100%;
  background: #000;
  aspect-ratio: 4/3;
  overflow: hidden;
}

.camera-feed {
  width: 100%;
  height: 100%;
  object-fit: cover;
  display: block;
}

.camera-overlay {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.9rem;
  color: rgba(255, 255, 255, 0.6);
  background: rgba(0, 0, 0, 0.6);
}
</style>
