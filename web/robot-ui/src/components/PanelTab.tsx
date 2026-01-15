import { useRosConnection } from '../hooks/useRosConnection';

const rosbridgeUrl = import.meta.env.VITE_ROSBRIDGE_WS_URL ?? 'ws://localhost:9090';
const videoUrl = import.meta.env.VITE_VIDEO_URL ?? 'http://localhost:8080/mjpeg';
const audioUrl = import.meta.env.VITE_AUDIO_URL ?? 'http://localhost:8081/audio';

export default function PanelTab() {
  const { status, latencyMs } = useRosConnection();
  const ip =
    import.meta.env.VITE_PANEL_ROBOT_IP ??
    (typeof window !== 'undefined' ? window.location.hostname : 'robot.local');

  return (
    <>
      <section className="card">
        <h3>Estado general</h3>
        <p>
          Estado rosbridge: <strong>{status}</strong>
        </p>
        <p>Latencia estimada: {latencyMs ? `${Math.round(latencyMs)} ms` : 'N/D'}</p>
        <p>
          IP actual del panel:{' '}
          <code>{typeof window !== 'undefined' ? window.location.host : 'localhost'}</code>
        </p>
        <p>
          Sugerencia: abrir <code>http://{ip}:5173</code> desde otra PC en la misma LAN.
        </p>
      </section>
      <section className="grid">
        <article className="card">
          <h3>Rosbridge</h3>
          <p>WebSocket: <code>{rosbridgeUrl}</code></p>
          <p>Usa conexión segura solo si configuraste TLS en rosbridge.</p>
        </article>
        <article className="card">
          <h3>Video MJPEG</h3>
          <p>URL: <code>{videoUrl}</code></p>
          <p>Agrega <code>?topic=/tu/topic</code> para cambiar la cámara desde el navegador.</p>
        </article>
        <article className="card">
          <h3>Audio</h3>
          <p>URL: <code>{audioUrl}</code></p>
          <p>Formato WAV PCM continuo (solo escucha por ahora).</p>
        </article>
      </section>
    </>
  );
}
