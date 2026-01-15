import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import './styles/base.css';
import { RosProvider } from './hooks/useRosConnection';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <RosProvider>
      <App />
    </RosProvider>
  </React.StrictMode>,
);
