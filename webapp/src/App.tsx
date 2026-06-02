import { useEffect } from "react";
import { Route, Routes } from "react-router-dom";
import TopBar from "./components/TopBar";
import SimulationViewport from "./components/SimulationViewport";
import ComponentMap from "./components/ComponentMap";
import Visualizations from "./components/Visualizations";
import ImuView from "./components/ImuView";
import Profiling from "./components/Profiling";
import { store } from "./store";

function DashboardPage() {
  return (
    <>
      <div className="stage">
        <SimulationViewport />
        <ComponentMap />
      </div>
      <Visualizations />
    </>
  );
}

export default function App() {
  useEffect(() => {
    store.connect();
  }, []);

  return (
    <div className="app">
      <TopBar />
      <Routes>
        <Route path="/" element={<DashboardPage />} />
        <Route path="/position" element={<ImuView />} />
        <Route path="/profiling" element={<Profiling />} />
      </Routes>
    </div>
  );
}
