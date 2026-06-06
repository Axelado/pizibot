// No caching — always fetch from network.
// PWA install (manifest.json) still works without caching.

self.addEventListener('install', () => self.skipWaiting())
self.addEventListener('activate', (e) => {
  e.waitUntil(
    caches.keys().then((keys) => Promise.all(keys.map((k) => caches.delete(k))))
  )
  self.clients.claim()
})
// No fetch handler → browser fetches everything from network normally
