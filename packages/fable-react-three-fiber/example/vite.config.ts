import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import basicSsl from '@vitejs/plugin-basic-ssl';
import { fileURLToPath } from 'node:url';

// Example app for local development and the Playwright smoke test.
// The package name resolves to the library sources, and the packaged
// tracker.wasm asset is served from /tracker.wasm via publicDir.
export default defineConfig({
  root: fileURLToPath(new URL('.', import.meta.url)),
  publicDir: fileURLToPath(new URL('../assets', import.meta.url)),
  plugins: [react(), basicSsl()],
  resolve: {
    alias: {
      '@j1ngzoue/fable-react-three-fiber': fileURLToPath(new URL('../src/index.ts', import.meta.url)),
    },
  },
  server: { https: true, host: true, port: 4176 },
  preview: { https: true, host: true, port: 4176 },
  worker: { format: 'es' },
  build: {
    outDir: fileURLToPath(new URL('./dist', import.meta.url)),
    emptyOutDir: true,
    target: 'es2022',
  },
});
