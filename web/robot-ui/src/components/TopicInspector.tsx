import { useMemo, useState } from 'react';
import { useRosConnection } from '../hooks/useRosConnection';
import { useTopicSubscription } from '../hooks/useTopicSubscription';

export default function TopicInspector() {
  const { topics } = useRosConnection();
  const [inputTopic, setInputTopic] = useState('');
  const [activeTopic, setActiveTopic] = useState<string>();
  const subscription = useTopicSubscription(activeTopic);

  const prettyMessage = useMemo(() => {
    if (!subscription.data) {
      return '';
    }
    try {
      return JSON.stringify(subscription.data, null, 2);
    } catch {
      return String(subscription.data);
    }
  }, [subscription.data]);

  const handleSubscribe = () => {
    if (inputTopic.trim()) {
      setActiveTopic(inputTopic.trim());
    }
  };

  return (
    <section className="card topic-inspector">
      <h3>Inspector de tópicos</h3>
      <p>Selecciona un tópico y mira el último mensaje más la tasa aproximada.</p>
      <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
        <input
          type="text"
          list="topic-list"
          placeholder="/cmd_vel"
          value={inputTopic}
          onChange={(event) => setInputTopic(event.target.value)}
          style={{
            flex: 1,
            padding: '0.6rem',
            borderRadius: '0.6rem',
            border: '1px solid #1c2538',
            background: '#0f1524',
            color: '#e2e8ff',
          }}
        />
        <datalist id="topic-list">
          {topics.map((topic) => (
            <option key={topic} value={topic} />
          ))}
        </datalist>
        <button className="secondary" type="button" onClick={handleSubscribe}>
          Suscribirse
        </button>
      </div>
      {activeTopic && (
        <p>
          Leyendo <code>{activeTopic}</code> tipo{' '}
          <code>{subscription.messageType ?? 'desconocido'}</code> |{' '}
          {subscription.hz ? `${subscription.hz} Hz` : 'sin tasa'}
        </p>
      )}
      {subscription.error && <p style={{ color: '#f86f6f' }}>{subscription.error}</p>}
      <textarea
        readOnly
        value={prettyMessage}
        placeholder="Sin mensajes todavía..."
      />
    </section>
  );
}
