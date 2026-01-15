import { useEffect, useMemo, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRosConnection } from './useRosConnection';

export interface RobotStatus {
  vbat?: number;
  at?: number;
  ibat?: number;
  dist?: number;
  yaw?: number;
  bL?: number;
  bR?: number;
  cliff?: number;
  [key: string]: unknown;
}

function normalizeJson(raw: string): string {
  let text = raw.trim();
  if (!text.startsWith('{')) {
    text = `{${text}`;
  }
  if (!text.endsWith('}')) {
    text = `${text}}`;
  }
  // Corrige claves sin comillas o con comillas incompletas (ej: at":7.98 o vbat:8.2)
  text = text.replace(/([,{]\s*)([a-zA-Z_][a-zA-Z0-9_]*)\s*:/g, '$1"$2":');
  text = text.replace(/([,{]\s*)([a-zA-Z_][a-zA-Z0-9_]*)\"\s*:/g, '$1"$2":');
  return text;
}

function regexExtract(payload: string): RobotStatus | undefined {
  const assign = (target: RobotStatus, key: keyof RobotStatus, match: RegExpMatchArray | null) => {
    if (!match) {
      return;
    }
    const value = Number(match[1]);
    if (!Number.isNaN(value)) {
      target[key] = value;
    }
  };
  const status: RobotStatus = {};
  assign(status, 'at', payload.match(/at"?\s*:\s*([-+]?\d*\.?\d+)/i));
  assign(status, 'vbat', payload.match(/vbat"?\s*:\s*([-+]?\d*\.?\d+)/i));
  assign(status, 'ibat', payload.match(/ibat"?\s*:\s*([-+]?\d*\.?\d+)/i));
  assign(status, 'dist', payload.match(/dist"?\s*:\s*([-+]?\d*\.?\d+)/i));
  assign(status, 'yaw', payload.match(/yaw"?\s*:\s*([-+]?\d*\.?\d+)/i));
  assign(status, 'bL', payload.match(/bL"?\s*:\s*([-+]?\d*\.?\d+)/i));
  assign(status, 'bR', payload.match(/bR"?\s*:\s*([-+]?\d*\.?\d+)/i));
  assign(status, 'cliff', payload.match(/cliff"?\s*:\s*([-+]?\d*\.?\d+)/i));
  if (Object.keys(status).length === 0) {
    return undefined;
  }
  return status;
}

export function useRobotStatus(topic = '/arturito/status_raw') {
  const { ros, topics } = useRosConnection();
  const [lastMessage, setLastMessage] = useState<string>();
  const [hz, setHz] = useState<number>();
  const lastStampRef = useRef<number>();
  const resolvedTopic = useMemo(() => {
    if (topics?.includes(topic)) {
      return topic;
    }
    if (topic.startsWith('/') && topics?.includes(topic.slice(1))) {
      return topic.slice(1);
    }
    if (!topic.startsWith('/') && topics?.includes(`/${topic}`)) {
      return `/${topic}`;
    }
    return topic;
  }, [topics, topic]);

  useEffect(() => {
    if (!ros) {
      setLastMessage(undefined);
      setHz(undefined);
      lastStampRef.current = undefined;
      return;
    }
    const rosTopic = new ROSLIB.Topic<{ data: string }>({
      ros,
      name: resolvedTopic,
      messageType: 'std_msgs/msg/String',
    });
    rosTopic.subscribe((message: { data: string }) => {
      const now = performance.now();
      if (lastStampRef.current) {
        const delta = (now - lastStampRef.current) / 1000;
        setHz(delta > 0 ? Number((1 / delta).toFixed(2)) : undefined);
      }
      lastStampRef.current = now;
      setLastMessage(message.data);
    });
    return () => {
      rosTopic.unsubscribe();
    };
  }, [ros, resolvedTopic]);

  const { status, error, raw } = useMemo(() => {
    const payload = lastMessage;
    if (!payload) {
      return { status: undefined, error: undefined, raw: undefined };
    }
    try {
      const normalized = normalizeJson(payload);
      const parsed = JSON.parse(normalized) as RobotStatus;
      if (parsed.vbat === undefined && typeof parsed.at === 'number') {
        parsed.vbat = parsed.at;
      }
      return { status: parsed, error: undefined, raw: payload };
    } catch (err) {
      const fallback = regexExtract(payload);
      const message = err instanceof Error ? err.message : String(err);
      if (fallback) {
        if (fallback.vbat === undefined && typeof fallback.at === 'number') {
          fallback.vbat = fallback.at;
        }
        return { status: fallback, error: undefined, raw: payload };
      }
      return { status: undefined, error: message, raw: payload };
    }
  }, [lastMessage]);

  return {
    hz,
    robotStatus: status,
    parseError: error,
    raw,
  };
}
