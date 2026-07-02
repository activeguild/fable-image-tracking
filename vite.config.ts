import { defineConfig } from 'vite';
import { fileURLToPath, URL } from 'node:url';
import basicSsl from '@vitejs/plugin-basic-ssl';

export default defineConfig({
  // Self-signed HTTPS so getUserMedia works on phones over the LAN
  // (browsers require a secure context for camera access).
  plugins: [basicSsl()],
  build: {
    rollupOptions: {
      input: {
        main: fileURLToPath(new URL('./index.html', import.meta.url)),
        target: fileURLToPath(new URL('./target.html', import.meta.url)),
      },
    },
  },
});
