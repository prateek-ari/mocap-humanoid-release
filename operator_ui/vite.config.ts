import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// `npm run dev` proxies the API and event socket to a locally running
// backend. The proxy rewrites Host to the backend, but the browser's Origin
// stays the dev server's, so the backend must be started with that origin on
// its allowlist, e.g. `mocap-g1-console --demo --allowed-host localhost:5173`.
const backendTarget = process.env.MOCAP_G1_CONSOLE_URL
  ?? `http://127.0.0.1:${process.env.MOCAP_G1_CONSOLE_PORT ?? '8766'}`

export default defineConfig({
  plugins: [react()],
  base: './',
  server: {
    host: '127.0.0.1',
    port: 5173,
    strictPort: true,
    proxy: {
      '/api': {
        target: backendTarget,
        changeOrigin: true,
        ws: true,
      },
    },
  },
  build: {
    outDir: '../mocap_g1/operator/static',
    emptyOutDir: true,
    sourcemap: false,
  },
})
