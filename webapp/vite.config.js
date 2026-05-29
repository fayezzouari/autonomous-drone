import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
// The web app talks to the Python WebSocket bridge (see sim_bridge/web_bridge.py).
// Override the bridge URL at build/dev time with VITE_BRIDGE_URL if it is not on
// localhost:8765, e.g. VITE_BRIDGE_URL=ws://10.0.0.5:8765 npm run dev
export default defineConfig({
    plugins: [react()],
    server: { host: true, port: 5173 },
});
