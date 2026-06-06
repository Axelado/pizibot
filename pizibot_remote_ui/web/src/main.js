import { createApp } from 'vue'
import App from './App.vue'
import './styles/main.css'

if ('serviceWorker' in navigator) {
  navigator.serviceWorker.register('./sw.js')
}

createApp(App).mount('#app')
