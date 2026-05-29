import { useEffect } from "react";
import TopBar from "./components/TopBar";
import SimulationViewport from "./components/SimulationViewport";
import ComponentMap from "./components/ComponentMap";
import Visualizations from "./components/Visualizations";
import { store } from "./store";

export default function App() {
  useEffect(() => {
    store.connect();
  }, []);

  return (
    <div className="app">
      <TopBar />
      <div className="stage">
        <SimulationViewport />
        <ComponentMap />
      </div>
      <Visualizations />
    </div>
  );
}
