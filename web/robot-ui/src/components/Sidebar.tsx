import { useRosConnection } from '../hooks/useRosConnection';

const statusLabels: Record<string, { text: string; className: string }> = {
  conectando: { text: 'Conectando...', className: 'status-badge warn' },
  conectado: { text: 'Conectado', className: 'status-badge ok' },
  reintentando: { text: 'Reconectando...', className: 'status-badge warn' },
  error: { text: 'Error', className: 'status-badge error' },
  desconectado: { text: 'Desconectado', className: 'status-badge error' },
};

export default function Sidebar() {
  const { status, latencyMs, nodes, topics, services, lastError, refreshGraph } =
    useRosConnection();
  const badge = statusLabels[status] ?? statusLabels.desconectado;

  return (
    <aside className="sidebar">
      <div>
        <h2>ROS</h2>
        <div className={badge.className}>
          {badge.text}
          {latencyMs && (
            <span style={{ fontWeight: 400 }}>
              {Math.round(latencyMs)} ms
            </span>
          )}
        </div>
        {lastError && <small style={{ color: '#f86f6f' }}>{lastError}</small>}
        <button className="secondary" onClick={refreshGraph} type="button">
          Refrescar gráfico
        </button>
      </div>
      <section className="sidebar-section">
        <strong>Nodos ({nodes.length})</strong>
        <ul className="sidebar-list">
          {nodes.map((node) => (
            <li key={node}>{node}</li>
          ))}
        </ul>
      </section>
      <section className="sidebar-section">
        <strong>Tópicos ({topics.length})</strong>
        <ul className="sidebar-list">
          {topics.map((topic) => (
            <li key={topic}>{topic}</li>
          ))}
        </ul>
      </section>
      <section className="sidebar-section">
        <strong>Servicios ({services.length})</strong>
        <ul className="sidebar-list">
          {services.map((srv) => (
            <li key={srv}>{srv}</li>
          ))}
        </ul>
      </section>
    </aside>
  );
}
