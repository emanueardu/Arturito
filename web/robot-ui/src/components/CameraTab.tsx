import { useMemo, useState } from 'react';
import { useRosConnection } from '../hooks/useRosConnection';

const baseVideoUrl = import.meta.env.VITE_VIDEO_URL ?? 'http://localhost:8080/mjpeg';

export default function CameraTab() {
  const { topics } = useRosConnection();
  const [topic, setTopic] = useState('');
  const [cacheBust, setCacheBust] = useState(Date.now());

  const url = useMemo(() => {
    if (!topic) {
      return `${baseVideoUrl}?cb=${cacheBust}`;
    }
    const separator = baseVideoUrl.includes('?') ? '&' : '?';
    return `${baseVideoUrl}${separator}topic=${encodeURIComponent(topic)}&cb=${cacheBust}`;
  }, [cacheBust, topic]);

  const imageTopics = topics.filter((name) => name.toLowerCase().includes('image'));

  return (
    <>
      <section className="card">
        <h3>Streaming MJPEG</h3>
        <p>
          Usa el nodo <code>robot_web_bridge</code> como fuente. Si la cámara no aparece,
          verificá que exista un tópico sensor_msgs/Image o sensor_msgs/CompressedImage.
        </p>
        <label htmlFor="camera-topic">
          Tópico preferido
          <input
            list="image-topics"
            id="camera-topic"
            type="text"
            value={topic}
            onChange={(event) => setTopic(event.target.value)}
            placeholder="Ej: /camera/image_raw"
            style={{
              width: '100%',
              padding: '0.5rem',
              marginTop: '0.5rem',
              borderRadius: '0.6rem',
              border: '1px solid #1c2538',
              background: '#0f1524',
              color: '#f4f4f4',
            }}
          />
          <datalist id="image-topics">
            {imageTopics.map((name) => (
              <option key={name} value={name} />
            ))}
          </datalist>
        </label>
        <button className="secondary" type="button" onClick={() => setCacheBust(Date.now())}>
          Recargar imagen
        </button>
      </section>
      <section className="camera-frame card">
        <img src={url} alt="stream del robot" />
      </section>
    </>
  );
}
