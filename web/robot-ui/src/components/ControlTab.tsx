import { useEffect, useMemo, useRef, useState, type PointerEvent } from 'react';
import ROSLIB from 'roslib';
import { useRosConnection } from '../hooks/useRosConnection';
import { useTopicSubscription } from '../hooks/useTopicSubscription';
import { useRobotStatus } from '../hooks/useRobotStatus';

const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value));
const CMD_TOPIC = import.meta.env.VITE_CMD_VEL_TOPIC ?? '/cmd_vel';
const TILT_CMD_TOPIC = import.meta.env.VITE_TILT_CMD_TOPIC ?? '/robot_web/tilt_deg';
const VACUUM_CMD_TOPIC = import.meta.env.VITE_VACUUM_CMD_TOPIC ?? '/robot_web/vacuum_enable';
const BRUSH_CMD_TOPIC = import.meta.env.VITE_BRUSH_CMD_TOPIC ?? '/robot_web/brush_enable';
const VACUUM_STATE_TOPIC = import.meta.env.VITE_VACUUM_STATE_TOPIC ?? '/arturito/vacuum_state';
const BRUSH_STATE_TOPIC = import.meta.env.VITE_BRUSH_STATE_TOPIC ?? '/arturito/brush_state';
const TRACKER_CMD_TOPIC = import.meta.env.VITE_TRACKER_CMD_TOPIC ?? '/tracker_control';
const TRACKER_STATE_TOPIC = import.meta.env.VITE_TRACKER_STATE_TOPIC ?? '/tracker_status';
const WANDER_CMD_TOPIC = import.meta.env.VITE_WANDER_CMD_TOPIC ?? '/robot_web/wander_enable';
const WANDER_STATE_TOPIC = import.meta.env.VITE_WANDER_STATE_TOPIC ?? '/robot_web/wander_status';
const CLEAN_CMD_TOPIC = import.meta.env.VITE_CLEAN_CMD_TOPIC ?? '/robot_web/clean_enable';
const CLEAN_STATE_TOPIC = import.meta.env.VITE_CLEAN_STATE_TOPIC ?? '/robot_web/clean_status';
const baseVideoUrl = import.meta.env.VITE_VIDEO_URL ?? 'http://localhost:8080/mjpeg';
const TILT_MIN = Number(import.meta.env.VITE_TILT_MIN ?? -20);
const TILT_MAX = Number(import.meta.env.VITE_TILT_MAX ?? 45);

export default function ControlTab() {
  const { ros, status, topics } = useRosConnection();
  const [maxLinear, setMaxLinear] = useState(0.35);
  const [maxAngular, setMaxAngular] = useState(1.2);
  const [joystickAxis, setJoystickAxis] = useState({ x: 0, y: 0 });
  const [keyboardAxis, setKeyboardAxis] = useState({ x: 0, y: 0 });
  const [dragging, setDragging] = useState(false);
  const [blocked, setBlocked] = useState(false);
  const [tiltValue, setTiltValue] = useState(0);
  const [tiltPending, setTiltPending] = useState(false);
  const [vacuumState, setVacuumState] = useState(false);
  const [brushState, setBrushState] = useState(false);
  const [vacuumPending, setVacuumPending] = useState(false);
  const [brushPending, setBrushPending] = useState(false);
  const [trackerState, setTrackerState] = useState(false);
  const [trackerPending, setTrackerPending] = useState(false);
  const [wanderState, setWanderState] = useState(false);
  const [wanderPending, setWanderPending] = useState(false);
  const [cleanState, setCleanState] = useState(false);
  const [cleanPending, setCleanPending] = useState(false);
  const [serviceFeedback, setServiceFeedback] = useState<string>();
  const [cameraTopic, setCameraTopic] = useState('');
  const [cameraCache, setCameraCache] = useState(Date.now());
  const joystickRef = useRef<HTMLDivElement>(null);
  const tiltPublisher = useRef<ROSLIB.Topic>();
  const vacuumPublisher = useRef<ROSLIB.Topic>();
  const brushPublisher = useRef<ROSLIB.Topic>();
  const trackerPublisher = useRef<ROSLIB.Topic>();
  const wanderPublisher = useRef<ROSLIB.Topic>();
  const cleanPublisher = useRef<ROSLIB.Topic>();
  const robotStatus = useRobotStatus();
  const yawSamplesRef = useRef<{ t: number; yaw: number }[]>([]);
  const distanceSamplesRef = useRef<{ t: number; dist: number }[]>([]);
  const batterySamplesRef = useRef<{ t: number; vbat: number }[]>([]);
  const [yawAvg, setYawAvg] = useState<number>();
  const [distanceAvg, setDistanceAvg] = useState<number>();
  const [batteryAvg, setBatteryAvg] = useState<number>();
  const imageTopics = useMemo(
    () => topics.filter((name) => name.toLowerCase().includes('image')),
    [topics],
  );
  const cameraUrl = useMemo(() => {
    const separator = baseVideoUrl.includes('?') ? '&' : '?';
    const base = cameraTopic ? `${baseVideoUrl}${separator}topic=${encodeURIComponent(cameraTopic)}` : baseVideoUrl;
    return `${base}${base.includes('?') ? '&' : '?'}cb=${cameraCache}`;
  }, [cameraTopic, cameraCache]);

  const vacuumFeedback = useTopicSubscription<{ data: boolean }>(VACUUM_STATE_TOPIC);
  const brushFeedback = useTopicSubscription<{ data: boolean }>(BRUSH_STATE_TOPIC);
  const trackerFeedback = useTopicSubscription<{ data: boolean }>(TRACKER_STATE_TOPIC);
  const wanderFeedback = useTopicSubscription<{ data: boolean }>(WANDER_STATE_TOPIC);
  const cleanFeedback = useTopicSubscription<{ data: boolean }>(CLEAN_STATE_TOPIC);
  const batteryVoltage =
    typeof robotStatus.robotStatus?.vbat === 'number' ? robotStatus.robotStatus.vbat : undefined;
  const batteryCurrent =
    typeof robotStatus.robotStatus?.ibat === 'number' ? robotStatus.robotStatus.ibat : undefined;
  const distanceCm =
    typeof robotStatus.robotStatus?.dist === 'number' ? robotStatus.robotStatus.dist : undefined;
  const bumperLeftActive =
    robotStatus.robotStatus?.bL !== undefined ? Number(robotStatus.robotStatus.bL) > 0 : undefined;
  const bumperRightActive =
    robotStatus.robotStatus?.bR !== undefined ? Number(robotStatus.robotStatus.bR) > 0 : undefined;
  const yawDeg =
    typeof robotStatus.robotStatus?.yaw === 'number' ? robotStatus.robotStatus.yaw : undefined;

  useEffect(() => {
    if (yawDeg === undefined) {
      return;
    }
    yawSamplesRef.current = [...yawSamplesRef.current, { t: performance.now(), yaw: yawDeg }].slice(-200);
  }, [yawDeg]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      const now = performance.now();
      const yawSamples = yawSamplesRef.current.filter((sample) => now - sample.t <= 2000);
      yawSamplesRef.current = yawSamples;
      if (!yawSamples.length) {
        setYawAvg(undefined);
      } else {
        const sum = yawSamples.reduce((acc, sample) => acc + sample.yaw, 0);
        setYawAvg(sum / yawSamples.length);
      }
      const distanceSamples = distanceSamplesRef.current.filter((sample) => now - sample.t <= 2000);
      distanceSamplesRef.current = distanceSamples;
      if (!distanceSamples.length) {
        setDistanceAvg(undefined);
      } else {
        const sum = distanceSamples.reduce((acc, sample) => acc + sample.dist, 0);
        setDistanceAvg(sum / distanceSamples.length);
      }
      const batterySamples = batterySamplesRef.current.filter((sample) => now - sample.t <= 2000);
      batterySamplesRef.current = batterySamples;
      if (!batterySamples.length) {
        setBatteryAvg(undefined);
      } else {
        const sum = batterySamples.reduce((acc, sample) => acc + sample.vbat, 0);
        setBatteryAvg(sum / batterySamples.length);
      }
    }, 2000);
    return () => window.clearInterval(timer);
  }, []);

  useEffect(() => {
    if (distanceCm === undefined) {
      return;
    }
    distanceSamplesRef.current = [...distanceSamplesRef.current, { t: performance.now(), dist: distanceCm }].slice(-200);
  }, [distanceCm]);

  useEffect(() => {
    if (batteryVoltage === undefined) {
      return;
    }
    batterySamplesRef.current = [...batterySamplesRef.current, { t: performance.now(), vbat: batteryVoltage }].slice(-200);
  }, [batteryVoltage]);

  const combinedAxis = useMemo(() => {
    return {
      x: clamp(joystickAxis.x + keyboardAxis.x, -1, 1),
      y: clamp(joystickAxis.y + keyboardAxis.y, -1, 1),
    };
  }, [joystickAxis, keyboardAxis]);

  useEffect(() => {
    if (!ros) {
      return;
    }
    const topic = new ROSLIB.Topic({
      ros,
      name: CMD_TOPIC,
      messageType: 'geometry_msgs/msg/Twist',
    });
    topic.advertise();
    const timer = window.setInterval(() => {
      const blockMultiplier = blocked ? 0 : 1;
      const message = new ROSLIB.Message({
        linear: { x: combinedAxis.y * maxLinear * blockMultiplier, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -combinedAxis.x * maxAngular * blockMultiplier },
      });
      topic.publish(message);
    }, 80);
    return () => {
      window.clearInterval(timer);
      topic.unadvertise();
    };
  }, [ros, combinedAxis, maxLinear, maxAngular, blocked]);

  useEffect(() => {
    const pressed = new Set<string>();
    const updateAxes = () => {
      const forward = (pressed.has('KeyW') ? 1 : 0) + (pressed.has('ArrowUp') ? 1 : 0);
      const backward = (pressed.has('KeyS') ? 1 : 0) + (pressed.has('ArrowDown') ? 1 : 0);
      const turnRight = (pressed.has('KeyD') ? 1 : 0) + (pressed.has('ArrowRight') ? 1 : 0);
      const turnLeft = (pressed.has('KeyA') ? 1 : 0) + (pressed.has('ArrowLeft') ? 1 : 0);
      const y = forward - backward;
      const x = turnRight - turnLeft;
      setKeyboardAxis({ x: clamp(x, -1, 1), y: clamp(y, -1, 1) });
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (!['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(event.code)) {
        return;
      }
      event.preventDefault();
      if (event.code === 'Space') {
        activarParada();
        return;
      }
      pressed.add(event.code);
      updateAxes();
    };
    const onKeyUp = (event: KeyboardEvent) => {
      if (pressed.has(event.code)) {
        pressed.delete(event.code);
        updateAxes();
      }
    };
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
    };
  }, []);

  useEffect(() => {
    if (!ros) {
      tiltPublisher.current?.unadvertise();
      vacuumPublisher.current?.unadvertise();
      brushPublisher.current?.unadvertise();
      trackerPublisher.current?.unadvertise();
      wanderPublisher.current?.unadvertise();
      cleanPublisher.current?.unadvertise();
      tiltPublisher.current = undefined;
      vacuumPublisher.current = undefined;
      brushPublisher.current = undefined;
      trackerPublisher.current = undefined;
      wanderPublisher.current = undefined;
      cleanPublisher.current = undefined;
      return;
    }
    const tiltTopic = new ROSLIB.Topic({
      ros,
      name: TILT_CMD_TOPIC,
      messageType: 'std_msgs/msg/Float32',
    });
    const vacuumTopic = new ROSLIB.Topic({
      ros,
      name: VACUUM_CMD_TOPIC,
      messageType: 'std_msgs/msg/Bool',
    });
    const brushTopic = new ROSLIB.Topic({
      ros,
      name: BRUSH_CMD_TOPIC,
      messageType: 'std_msgs/msg/Bool',
    });
    const trackerTopic = new ROSLIB.Topic({
      ros,
      name: TRACKER_CMD_TOPIC,
      messageType: 'std_msgs/msg/String',
    });
    const wanderTopic = new ROSLIB.Topic({
      ros,
      name: WANDER_CMD_TOPIC,
      messageType: 'std_msgs/msg/Bool',
    });
    const cleanTopic = new ROSLIB.Topic({
      ros,
      name: CLEAN_CMD_TOPIC,
      messageType: 'std_msgs/msg/Bool',
    });
    tiltTopic.advertise();
    vacuumTopic.advertise();
    brushTopic.advertise();
    trackerTopic.advertise();
    wanderTopic.advertise();
    cleanTopic.advertise();
    tiltPublisher.current = tiltTopic;
    vacuumPublisher.current = vacuumTopic;
    brushPublisher.current = brushTopic;
    trackerPublisher.current = trackerTopic;
    wanderPublisher.current = wanderTopic;
    cleanPublisher.current = cleanTopic;
    return () => {
      tiltTopic.unadvertise();
      vacuumTopic.unadvertise();
      brushTopic.unadvertise();
      trackerTopic.unadvertise();
      wanderTopic.unadvertise();
      cleanTopic.unadvertise();
      if (tiltPublisher.current === tiltTopic) {
        tiltPublisher.current = undefined;
      }
      if (vacuumPublisher.current === vacuumTopic) {
        vacuumPublisher.current = undefined;
      }
      if (brushPublisher.current === brushTopic) {
        brushPublisher.current = undefined;
      }
      if (trackerPublisher.current === trackerTopic) {
        trackerPublisher.current = undefined;
      }
      if (wanderPublisher.current === wanderTopic) {
        wanderPublisher.current = undefined;
      }
      if (cleanPublisher.current === cleanTopic) {
        cleanPublisher.current = undefined;
      }
    };
  }, [ros]);

  useEffect(() => {
    if (typeof vacuumFeedback.data?.data === 'boolean') {
      setVacuumState(vacuumFeedback.data.data);
    }
  }, [vacuumFeedback.data]);

  useEffect(() => {
    if (typeof brushFeedback.data?.data === 'boolean') {
      setBrushState(brushFeedback.data.data);
    }
  }, [brushFeedback.data]);

  useEffect(() => {
    if (typeof trackerFeedback.data?.data === 'boolean') {
      setTrackerState(trackerFeedback.data.data);
    }
  }, [trackerFeedback.data]);

  useEffect(() => {
    if (typeof wanderFeedback.data?.data === 'boolean') {
      setWanderState(wanderFeedback.data.data);
    }
  }, [wanderFeedback.data]);

  useEffect(() => {
    if (typeof cleanFeedback.data?.data === 'boolean') {
      setCleanState(cleanFeedback.data.data);
    }
  }, [cleanFeedback.data]);

  const actualizarJoystick = (clientX: number, clientY: number) => {
    if (!joystickRef.current) {
      return;
    }
    const rect = joystickRef.current.getBoundingClientRect();
    const x = (clientX - rect.left) / rect.width;
    const y = (clientY - rect.top) / rect.height;
    const axisX = clamp(x * 2 - 1, -1, 1);
    const axisY = clamp((1 - y) * 2 - 1, -1, 1);
    setJoystickAxis({ x: axisX, y: axisY });
  };

  const handlePointerDown = (event: PointerEvent<HTMLDivElement>) => {
    setDragging(true);
    event.currentTarget.setPointerCapture(event.pointerId);
    actualizarJoystick(event.clientX, event.clientY);
  };

  const handlePointerMove = (event: PointerEvent<HTMLDivElement>) => {
    if (!dragging) {
      return;
    }
    actualizarJoystick(event.clientX, event.clientY);
  };

  const handlePointerUp = (event: PointerEvent<HTMLDivElement>) => {
    setDragging(false);
    event.currentTarget.releasePointerCapture?.(event.pointerId);
    setJoystickAxis({ x: 0, y: 0 });
  };

  const activarParada = () => {
    setJoystickAxis({ x: 0, y: 0 });
    setKeyboardAxis({ x: 0, y: 0 });
    setBlocked(true);
  };

  const reanudar = () => {
    setBlocked(false);
  };

  const publicarTilt = (valor: number) => {
    if (!tiltPublisher.current) {
      return;
    }
    tiltPublisher.current.publish(new ROSLIB.Message({ data: valor }));
  };

  const handleTiltChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(event.target.value);
    setTiltValue(value);
    setTiltPending(true);
    publicarTilt(value);
    window.setTimeout(() => setTiltPending(false), 300);
  };

  const centrarTilt = () => {
    const neutral = 0;
    setTiltValue(neutral);
    publicarTilt(neutral);
  };

  const sendVacuum = (desired: boolean) => {
    if (!vacuumPublisher.current) {
      setServiceFeedback('ROS no disponible para aspiradora.');
      return;
    }
    setVacuumPending(true);
    setVacuumState(desired);
    vacuumPublisher.current.publish(new ROSLIB.Message({ data: desired }));
    setServiceFeedback(`Comando enviado: aspiradora ${desired ? 'encendida' : 'apagada'}.`);
    window.setTimeout(() => setVacuumPending(false), 400);
  };

  const sendBrush = (desired: boolean) => {
    if (!brushPublisher.current) {
      setServiceFeedback('ROS no disponible para escobillas.');
      return;
    }
    setBrushPending(true);
    setBrushState(desired);
    brushPublisher.current.publish(new ROSLIB.Message({ data: desired }));
    setServiceFeedback(`Comando enviado: escobillas ${desired ? 'encendidas' : 'apagadas'}.`);
    window.setTimeout(() => setBrushPending(false), 400);
  };

  const sendTracker = (desired: boolean) => {
    if (!trackerPublisher.current) {
      setServiceFeedback('ROS no disponible para seguimiento de personas.');
      return;
    }
    setTrackerPending(true);
    setTrackerState(desired);
    const command = desired ? 'seguir' : 'dejar de seguir';
    trackerPublisher.current.publish(new ROSLIB.Message({ data: command }));
    setServiceFeedback(`Comando enviado: ${desired ? 'activar' : 'desactivar'} seguimiento.`);
    window.setTimeout(() => setTrackerPending(false), 500);
  };

  const sendWander = (desired: boolean) => {
    if (!wanderPublisher.current) {
      setServiceFeedback('ROS no disponible para navegación autónoma.');
      return;
    }
    setWanderPending(true);
    setWanderState(desired);
    wanderPublisher.current.publish(new ROSLIB.Message({ data: desired }));
    setServiceFeedback(`Comando enviado: ${desired ? 'activar' : 'desactivar'} wander_avoid.`);
    window.setTimeout(() => setWanderPending(false), 500);
  };

  const sendClean = (desired: boolean) => {
    if (!cleanPublisher.current) {
      setServiceFeedback('ROS no disponible para modo limpieza.');
      return;
    }
    setCleanPending(true);
    setCleanState(desired);
    cleanPublisher.current.publish(new ROSLIB.Message({ data: desired }));
    setServiceFeedback(`Comando enviado: ${desired ? 'activar' : 'desactivar'} limpieza rápida.`);
    window.setTimeout(() => setCleanPending(false), 500);
  };

  const handleMaxLinear = (event: React.ChangeEvent<HTMLInputElement>) => {
    setMaxLinear(Number(event.target.value));
  };

  const handleMaxAngular = (event: React.ChangeEvent<HTMLInputElement>) => {
    setMaxAngular(Number(event.target.value));
  };

  return (
    <div className="teleop-panel">
      <section className="card">
        <h3>Teleoperación</h3>
        <p>Conexión: <strong>{status}</strong></p>
        <div
          className="joystick"
          ref={joystickRef}
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerLeave={handlePointerUp}
        >
          <div
            className="joystick-handle"
            style={{
              left: `${50 + combinedAxis.x * 40}%`,
              top: `${50 - combinedAxis.y * 40}%`,
            }}
          />
        </div>
        <p>WASD o flechas también controlan el robot.</p>
        <div style={{ display: 'flex', gap: '0.5rem', marginTop: '0.5rem' }}>
          <button className="e_stop" onClick={activarParada} type="button">
            Parada de emergencia
          </button>
          {blocked && (
            <button className="secondary" onClick={reanudar} type="button">
              Reanudar
            </button>
          )}
        </div>
      </section>
      <section className="card sensor-panel">
        <h3>Sensores inmediatos</h3>
        <p>
          Batería:{' '}
          {batteryAvg !== undefined ? `${batteryAvg.toFixed(2)} V` : 'Sin datos (status_raw)'}
          {batteryCurrent !== undefined ? ` · ${batteryCurrent.toFixed(1)} A` : ''}
        </p>
        <p>
          Distancia frontal:{' '}
          {distanceAvg !== undefined ? `${distanceAvg.toFixed(1)} cm` : 'Sin datos (status_raw)'}
        </p>
        <p>
          Yaw actual:{' '}
          {yawAvg !== undefined ? `${yawAvg.toFixed(1)}°` : 'Sin datos (status_raw)'}
        </p>
        <div className="bumper-pills">
          <span className={`bumper-pill ${bumperLeftActive ? 'active' : ''}`}>
            Bumper izquierdo:{' '}
            {bumperLeftActive === undefined ? 'N/D' : bumperLeftActive ? 'Presionado' : 'Libre'}
          </span>
          <span className={`bumper-pill ${bumperRightActive ? 'active' : ''}`}>
            Bumper derecho:{' '}
            {bumperRightActive === undefined ? 'N/D' : bumperRightActive ? 'Presionado' : 'Libre'}
          </span>
        </div>
        <small>
          Datos de <code>arturito/status_raw</code>{' '}
          {robotStatus.hz ? `(${robotStatus.hz} Hz)` : '(sin tasa visible)'}
        </small>
        {robotStatus.parseError && !robotStatus.robotStatus && (
          <small style={{ color: '#f86f6f' }}>Error parseando status_raw: {robotStatus.parseError}</small>
        )}
      </section>
      <section className="card control-inputs">
        <h3>Ajustes</h3>
        <label>
          Velocidad lineal máx (m/s)
          <span>{maxLinear.toFixed(2)}</span>
        </label>
        <input
          type="range"
          min="0.05"
          max="0.8"
          step="0.05"
          value={maxLinear}
          onChange={handleMaxLinear}
        />
        <label>
          Velocidad angular máx (rad/s)
          <span>{maxAngular.toFixed(2)}</span>
        </label>
        <input
          type="range"
          min="0.2"
          max="2.5"
          step="0.1"
          value={maxAngular}
          onChange={handleMaxAngular}
        />
        <p>Tópico objetivo: <code>{CMD_TOPIC}</code></p>
        <p>Publicación continua: 12 Hz mientras haya comandos.</p>
      </section>
      <section className="card actuator-controls">
        <h3>Actuadores auxiliares</h3>
        <label>
          Tilt de cámara/cabeza
          <span>{tiltValue.toFixed(1)}°</span>
        </label>
        <input
          type="range"
          min={TILT_MIN}
          max={TILT_MAX}
          step="1"
          value={tiltValue}
          onChange={handleTiltChange}
          disabled={!ros}
        />
        <small>Publicando en <code>{TILT_CMD_TOPIC}</code>{tiltPending ? ' (enviando...)' : ''}</small>
        <button className="secondary" onClick={centrarTilt} type="button" disabled={!ros}>
          Centrar tilt
        </button>
        <hr style={{ borderColor: '#1c2538' }} />
        <div className="actuator-buttons">
          <div className="actuator-row">
            <span>Seguimiento de personas</span>
            <button
              type="button"
              className={`secondary toggle ${trackerState ? 'active' : 'danger'}`}
              onClick={() => sendTracker(!trackerState)}
              disabled={!ros || trackerPending}
            >
              {trackerState ? 'Desactivar' : 'Activar'}
            </button>
          </div>
          <div className="actuator-row">
            <span>Navegación wander_avoid</span>
            <button
              type="button"
              className={`secondary toggle ${wanderState ? 'active' : 'danger'}`}
              onClick={() => sendWander(!wanderState)}
              disabled={!ros || wanderPending}
            >
              {wanderState ? 'Desactivar' : 'Activar'}
            </button>
          </div>
          <div className="actuator-row">
            <span>Limpieza rápida</span>
            <button
              type="button"
              className={`secondary toggle ${cleanState ? 'active' : 'danger'}`}
              onClick={() => sendClean(!cleanState)}
              disabled={!ros || cleanPending}
            >
              {cleanState ? 'Desactivar' : 'Activar'}
            </button>
          </div>
          <div className="actuator-row">
            <span>Aspiradora</span>
            <button
              type="button"
              className={`secondary toggle ${vacuumState ? 'active' : 'danger'}`}
              onClick={() => sendVacuum(!vacuumState)}
              disabled={!ros || vacuumPending}
            >
              {vacuumState ? 'Apagar' : 'Encender'}
            </button>
          </div>
          <div className="actuator-row">
            <span>Escobillas</span>
            <button
              type="button"
              className={`secondary toggle ${brushState ? 'active' : 'danger'}`}
              onClick={() => sendBrush(!brushState)}
              disabled={!ros || brushPending}
            >
              {brushState ? 'Apagar' : 'Encender'}
            </button>
          </div>
        </div>
        <small>
          Estado aspiradora: <strong>{vacuumState ? 'Encendida' : 'Apagada'}</strong> — Escobillas:{' '}
          <strong>{brushState ? 'Encendidas' : 'Apagadas'}</strong>
        </small>
        {serviceFeedback && <small style={{ color: '#9ba7c1' }}>{serviceFeedback}</small>}
      </section>
      <section className="card camera-embed">
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: '1rem' }}>
          <h3 style={{ margin: 0 }}>Vista rápida de cámara</h3>
          <button className="secondary" type="button" onClick={() => setCameraCache(Date.now())}>
            Recargar
          </button>
        </div>
        <label htmlFor="camera-topic-control" style={{ marginTop: '0.5rem' }}>
          Tópico
          <input
            list="camera-topics-control"
            id="camera-topic-control"
            type="text"
            value={cameraTopic}
            onChange={(event) => setCameraTopic(event.target.value)}
            placeholder="/arturito/camera/faces/image"
          />
          <datalist id="camera-topics-control">
            {imageTopics.map((name) => (
              <option key={name} value={name} />
            ))}
          </datalist>
        </label>
        <div className="camera-preview">
          <img src={cameraUrl} alt="vista rápida del robot" />
        </div>
      </section>
    </div>
  );
}
