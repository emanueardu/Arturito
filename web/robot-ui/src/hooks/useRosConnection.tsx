import ROSLIB from 'roslib';
import {
  createContext,
  ReactNode,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';

type RosStatus = 'desconectado' | 'conectando' | 'reintentando' | 'conectado' | 'error';

interface RosGraphSnapshot {
  nodes: string[];
  topics: string[];
  services: string[];
}

interface RosContextData extends RosGraphSnapshot {
  status: RosStatus;
  lastError?: string;
  latencyMs?: number;
  ros?: ROSLIB.Ros;
  refreshGraph: () => void;
}

const defaultState: RosContextData = {
  status: 'desconectado',
  nodes: [],
  topics: [],
  services: [],
  refreshGraph: () => undefined,
};

const RosContext = createContext<RosContextData>(defaultState);

const ROS_URL = import.meta.env.VITE_ROSBRIDGE_WS_URL ?? 'ws://localhost:9090';

export function RosProvider({ children }: { children: ReactNode }) {
  const [status, setStatus] = useState<RosStatus>('conectando');
  const [lastError, setLastError] = useState<string>();
  const [latencyMs, setLatency] = useState<number>();
  const [graph, setGraph] = useState<RosGraphSnapshot>({
    nodes: [],
    topics: [],
    services: [],
  });
  const rosRef = useRef<ROSLIB.Ros>();
  const reconnectRef = useRef<number>();

  const fetchGraph = useCallback(() => {
    const rosInstance = rosRef.current;
    if (!rosInstance) {
      return;
    }
    rosInstance.getNodes(
      (nodes) => {
        setGraph((prev) => ({ ...prev, nodes: nodes.sort() }));
      },
      (err) => setLastError(String(err)),
    );
    rosInstance.getTopics(
      (resp) => {
        setGraph((prev) => ({
          ...prev,
          topics: resp.topics.sort(),
        }));
      },
      (err) => setLastError(String(err)),
    );
    rosInstance.getServices(
      (services) => {
        setGraph((prev) => ({ ...prev, services: services.sort() }));
      },
      (err) => setLastError(String(err)),
    );
  }, []);

  useEffect(() => {
    let active = true;
    const connect = () => {
      if (!active) {
        return;
      }
      setStatus('conectando');
      const ros = new ROSLIB.Ros({ url: ROS_URL });
      rosRef.current = ros;
      const cleanup = () => {
        ros.close();
        if (rosRef.current === ros) {
          rosRef.current = undefined;
        }
      };
      ros.on('connection', () => {
        if (!active) {
          cleanup();
          return;
        }
        setStatus('conectado');
        setLastError(undefined);
        fetchGraph();
      });
      ros.on('error', () => {
        setStatus('error');
        setLastError('No se pudo conectar con rosbridge.');
      });
      ros.on('close', () => {
        if (!active) {
          return;
        }
        setStatus('reintentando');
        reconnectRef.current = window.setTimeout(connect, 2500);
      });
      return cleanup;
    };
    const cleanup = connect();
    return () => {
      active = false;
      if (reconnectRef.current) {
        window.clearTimeout(reconnectRef.current);
      }
      cleanup?.();
    };
  }, [fetchGraph]);

  useEffect(() => {
    if (status !== 'conectado' || !rosRef.current) {
      return;
    }
    let cancelled = false;
    const runPing = () => {
      if (!rosRef.current || cancelled) {
        return;
      }
      const before = performance.now();
      rosRef.current.getNodes(
        () => {
          if (!cancelled) {
            setLatency(performance.now() - before);
          }
        },
        () => undefined,
      );
    };
    const timer = window.setInterval(runPing, 5000);
    runPing();
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, [status]);

  useEffect(() => {
    if (status !== 'conectado') {
      return;
    }
    const interval = window.setInterval(fetchGraph, 6000);
    return () => window.clearInterval(interval);
  }, [status, fetchGraph]);

  const value = useMemo<RosContextData>(
    () => ({
      status,
      lastError,
      latencyMs,
      ros: rosRef.current,
      ...graph,
      refreshGraph: fetchGraph,
    }),
    [status, lastError, latencyMs, graph, fetchGraph],
  );

  return <RosContext.Provider value={value}>{children}</RosContext.Provider>;
}

export function useRosConnection(): RosContextData {
  return useContext(RosContext);
}
