import { defineConfig } from 'vitest/config';
import react from '@vitejs/plugin-react';

// In development, Vite serves the app and passes /api and /ws to the dashboard server, which must
// be running (see README.md). The Host header is passed through unchanged, so the server's
// same-origin check compares the browser's Origin with Vite's own address and accepts it.
const server = process.env.R5_DASHBOARD_SERVER ?? 'https://localhost:8443';

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': { target: server, secure: false },
      '/ws': { target: server, secure: false, ws: true },
    },
  },
  test: {
    environment: 'node',
  },
});
