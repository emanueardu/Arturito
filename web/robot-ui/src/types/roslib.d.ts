declare module 'roslib' {
  export interface RosOptions {
    url: string;
  }

  export type RosEvent = 'connection' | 'close' | 'error';

  export class Ros {
    constructor(options: RosOptions);
    on(event: RosEvent, callback: () => void): void;
    close(): void;
    getNodes(callback: (nodes: string[]) => void, errorCallback?: (err: unknown) => void): void;
    getTopics(callback: (topics: { topics: string[]; types: string[] }) => void, errorCallback?: (err: unknown) => void): void;
    getServices(callback: (services: string[]) => void, errorCallback?: (err: unknown) => void): void;
    getTopicType(name: string, callback: (type: string) => void, errorCallback?: (err: unknown) => void): void;
  }

  export interface TopicOptions {
    ros: Ros;
    name: string;
    messageType: string;
  }

  export class Topic<T = unknown> {
    constructor(options: TopicOptions);
    subscribe(callback: (message: T) => void): void;
    unsubscribe(): void;
    publish(message: T): void;
    advertise(): void;
    unadvertise(): void;
  }

  export class Message<T = unknown> {
    constructor(values: T);
  }

  export interface ServiceOptions {
    ros: Ros;
    name: string;
    serviceType: string;
  }

  export class Service<Request = unknown, Response = unknown> {
    constructor(options: ServiceOptions);
    callService(
      request: ServiceRequest<Request>,
      callback?: (response: Response) => void,
      errorCallback?: (error: unknown) => void
    ): void;
  }

  export class ServiceRequest<T = unknown> {
    constructor(values: T);
  }
}
