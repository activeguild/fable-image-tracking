import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';

// Library build. The tracking worker (`new Worker(new URL(...), import.meta.url)`)
// is emitted as a separate ES chunk that resolves relative to dist/, so the
// package works from node_modules without consumer bundler configuration.
export default defineConfig({
  build: {
    lib: {
      entry: fileURLToPath(new URL('./src/index.ts', import.meta.url)),
      formats: ['es'],
      fileName: 'index',
    },
    rollupOptions: {
      external: [
        'react',
        'react-dom',
        'react/jsx-runtime',
        'three',
        '@react-three/fiber',
      ],
    },
    sourcemap: true,
    target: 'es2022',
  },
  worker: {
    format: 'es',
  },
});
