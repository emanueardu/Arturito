import { useTopicSubscription } from '../hooks/useTopicSubscription';
import { useRobotStatus } from '../hooks/useRobotStatus';

export default function DiagnosticsTab() {
  const battery = useTopicSubscription<{ voltage?: number; percentage?: number }>('arturito/battery');
  const ultrasonic = useTopicSubscription<{ range?: number }>('arturito/ultrasonic');
  const bumperLeft = useTopicSubscription<{ data?: boolean }>('arturito/bumper_left');
  const bumperRight = useTopicSubscription<{ data?: boolean }>('arturito/bumper_right');
  const statusRaw = useRobotStatus();

  const voltage =
    typeof statusRaw.robotStatus?.vbat === 'number'
      ? statusRaw.robotStatus.vbat
      : battery.data?.voltage;
  const current =
    typeof statusRaw.robotStatus?.ibat === 'number' ? statusRaw.robotStatus.ibat : undefined;

  return (
    <section className="grid">
      <article className="card">
        <h3>Batería</h3>
        {voltage !== undefined ? (
          <>
            <p>Tensión: {voltage.toFixed(2)} V</p>
            {typeof battery.data?.percentage === 'number' && (
              <p>Carga estimada: {(battery.data.percentage * 100).toFixed(1)}%</p>
            )}
            {current !== undefined && <p>Corriente: {current.toFixed(1)} A</p>}
            <small>
              Fuente: {statusRaw.robotStatus ? 'arturito/status_raw' : 'arturito/battery'} —{' '}
              {statusRaw.hz ? `${statusRaw.hz} Hz` : battery.hz ? `${battery.hz} Hz` : 'sin tasa'}
            </small>
            {statusRaw.parseError && !statusRaw.robotStatus && (
              <small style={{ color: '#f86f6f' }}>Error parseando status_raw: {statusRaw.parseError}</small>
            )}
          </>
        ) : (
          <p>
            Esperando datos en <code>arturito/status_raw</code> o <code>arturito/battery</code>.
          </p>
        )}
      </article>
      <article className="card">
        <h3>Ultrasonido frontal</h3>
        {ultrasonic.data ? (
          <>
            <p>
              Distancia:{' '}
              {ultrasonic.data.range !== undefined
                ? `${ultrasonic.data.range.toFixed(2)} m`
                : 'N/D'}
            </p>
            <small>{ultrasonic.hz ? `${ultrasonic.hz} Hz` : 'N/D'}</small>
          </>
        ) : (
          <p>
            Sin lecturas en <code>arturito/ultrasonic</code>.
          </p>
        )}
      </article>
      <article className="card">
        <h3>Bumpers</h3>
        <p>Izquierdo: {bumperLeft.data?.data ? 'Activado' : 'Libre'}</p>
        <p>Derecho: {bumperRight.data?.data ? 'Activado' : 'Libre'}</p>
      </article>
    </section>
  );
}
