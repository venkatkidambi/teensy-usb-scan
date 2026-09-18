import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';

export default defineConfig({
  root: fileURLToPath(new URL('./ui', import.meta.url)),
  clearScreen: false,
  server: { host: '127.0.0.1', port: 1420, strictPort: true, open: false },
  build: { outDir: '../dist', emptyOutDir: true, target: 'es2022' },
});
