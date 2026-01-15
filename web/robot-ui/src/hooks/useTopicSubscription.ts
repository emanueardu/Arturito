import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useRosConnection } from './useRosConnection';

interface TopicState<T> {
  data?: T;
  hz?: number;
  messageType?: string;
  error?: string;
}

export function useTopicSubscription<T = unknown>(topicName?: string | null): TopicState<T> {
  const { ros } = useRosConnection();
  const [state, setState] = useState<TopicState<T>>({});

  useEffect(() => {
    if (!topicName || !ros) {
      setState({});
      return;
    }
    let active = true;
    let topic: ROSLIB.Topic<T> | undefined;
    let lastStamp: number | undefined;
    const subscribe = (type: string) => {
      topic = new ROSLIB.Topic<T>({ ros, name: topicName, messageType: type });
      topic.subscribe((message: T) => {
        const now = performance.now();
        let hz: number | undefined;
        if (lastStamp) {
          const delta = (now - lastStamp) / 1000;
          hz = delta > 0 ? Number((1 / delta).toFixed(2)) : undefined;
        }
        lastStamp = now;
        setState({ data: message, hz, messageType: type });
      });
    };
    ros.getTopicType(
      topicName,
      (type) => {
        if (!active) {
          return;
        }
        subscribe(type);
      },
      (err) => {
        setState({ error: String(err) });
      },
    );
    return () => {
      active = false;
      if (topic) {
        topic.unsubscribe();
      }
    };
  }, [topicName, ros]);

  return state;
}
