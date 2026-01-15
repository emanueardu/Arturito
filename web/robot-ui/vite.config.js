var _a;
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
export default defineConfig({
    plugins: [react()],
    server: {
        host: '0.0.0.0',
        port: Number((_a = process.env.VITE_DEV_PORT) !== null && _a !== void 0 ? _a : 5173),
    },
    preview: {
        host: '0.0.0.0',
        port: 4173,
    },
});
