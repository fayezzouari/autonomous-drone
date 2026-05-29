import { useEffect, useState, useSyncExternalStore } from "react";
import { store } from "./store";
import type { StateMsg } from "./types";

// Subscribe React to the low-frequency store snapshot (connection / meta / status).
export function useSimSnapshot() {
  return useSyncExternalStore(store.subscribe, store.getSnapshot);
}

// Re-render at a capped rate (default ~12 Hz) with the latest hot state. Used by
// small text panels (HUD, legend) that should update without the 50 Hz churn.
export function useThrottledState(hz = 12): StateMsg | null {
  const [, setTick] = useState(0);
  useEffect(() => {
    const id = window.setInterval(() => setTick((n) => n + 1), 1000 / hz);
    return () => window.clearInterval(id);
  }, [hz]);
  return store.latest;
}
