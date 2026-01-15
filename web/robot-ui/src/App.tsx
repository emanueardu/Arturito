import { useState } from 'react';
import Sidebar from './components/Sidebar';
import PanelTab from './components/PanelTab';
import ControlTab from './components/ControlTab';
import MapTab from './components/MapTab';
import CameraTab from './components/CameraTab';
import AudioTab from './components/AudioTab';
import TopicInspector from './components/TopicInspector';

type TabKey = 'panel' | 'control' | 'mapa' | 'camara' | 'audio' | 'inspector';

export default function App() {
  const [activeTab, setActiveTab] = useState<TabKey>('panel');

  const tabs: { id: TabKey; label: string }[] = [
    { id: 'panel', label: 'Panel' },
    { id: 'control', label: 'Control' },
    { id: 'mapa', label: 'Mapa' },
    { id: 'camara', label: 'Cámara' },
    { id: 'audio', label: 'Audio' },
    { id: 'inspector', label: 'Inspector' },
  ];

  const renderTab = () => {
    switch (activeTab) {
      case 'panel':
        return <PanelTab />;
      case 'control':
        return <ControlTab />;
      case 'mapa':
        return <MapTab />;
      case 'camara':
        return <CameraTab />;
      case 'audio':
        return <AudioTab />;
      case 'inspector':
        return <TopicInspector />;
      default:
        return null;
    }
  };

  return (
    <div className="app-shell">
      <Sidebar />
      <div className="main-area">
        <div className="tabs">
          {tabs.map((tab) => (
            <button
              key={tab.id}
              type="button"
              className={tab.id === activeTab ? 'active' : ''}
              onClick={() => setActiveTab(tab.id)}
            >
              {tab.label}
            </button>
          ))}
        </div>
        <div className="tab-content">{renderTab()}</div>
      </div>
    </div>
  );
}
