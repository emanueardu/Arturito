import { useEffect, useMemo, useRef, useState, type ChangeEvent } from 'react';
import type { MouseEventHandler } from 'react';
import ROSLIB from 'roslib';
import { useRosConnection } from '../hooks/useRosConnection';
import { useTopicSubscription } from '../hooks/useTopicSubscription';

const MAP_IMAGE_URL = import.meta.env.VITE_MAP_IMAGE_URL ?? '/maps/apartamento_map.png';
const ZONE_CATALOG_TOPIC = import.meta.env.VITE_ZONE_CATALOG_TOPIC ?? '/robot_web/zone_catalog';
const ZONE_STATE_TOPIC = import.meta.env.VITE_ZONE_STATE_TOPIC ?? '/robot_web/zone_state';
const ZONE_GOAL_TOPIC = import.meta.env.VITE_ZONE_GOAL_TOPIC ?? '/zone_goal';
const ROBOT_POSE_TOPIC = import.meta.env.VITE_ROBOT_POSE_TOPIC ?? '/amcl_pose';
const FALLBACK_ZONES_URL = import.meta.env.VITE_FALLBACK_ZONES_URL ?? '/maps/apartamento_zones.json';
const WIFI_ANCHORS_TOPIC = import.meta.env.VITE_WIFI_ANCHORS_TOPIC ?? '/robot_web/wifi_anchors';
const WIFI_POSE_TOPIC = import.meta.env.VITE_WIFI_POSE_TOPIC ?? '/robot_web/wifi_pose';
const WIFI_STATUS_TOPIC = import.meta.env.VITE_WIFI_STATUS_TOPIC ?? '/robot_web/wifi_status';
const WIFI_CALIBRATE_TOPIC =
  import.meta.env.VITE_WIFI_CALIBRATE_TOPIC ?? '/robot_web/wifi_calibrate';
const DOCKING_INFO_TOPIC =
  import.meta.env.VITE_DOCKING_INFO_TOPIC ?? '/robot_web/docking_info';
const DOCKING_STATUS_TOPIC =
  import.meta.env.VITE_DOCKING_STATUS_TOPIC ?? '/robot_web/docking_status';
const DOCKING_CALIBRATE_TOPIC =
  import.meta.env.VITE_DOCKING_CALIBRATE_TOPIC ?? '/robot_web/docking_calibrate';
const WIFI_START_TOPIC =
  import.meta.env.VITE_WIFI_START_TOPIC ?? '/robot_web/start_wifi_localization';
const COVERAGE_SPACING_ENV = Number(import.meta.env.VITE_COVERAGE_SPACING_M ?? 0.35);
const COVERAGE_SPACING_M = Number.isFinite(COVERAGE_SPACING_ENV) ? COVERAGE_SPACING_ENV : 0.35;

interface ZoneEntry {
  id: string;
  centroid?: [number, number];
  area_m2?: number;
  polygon?: [number, number][];
}

interface CatalogPayload {
  zones: ZoneEntry[];
  metadata?: {
    resolution?: number;
    image_width_px?: number;
    image_height_px?: number;
    origin?: [number, number, number?];
  };
  dock_pose?: { x: number; y: number; theta?: number };
}

interface ZoneStatePayload {
  estado: string;
  zona?: string | null;
  timestamp?: number;
}

interface MapMetadata {
  resolution: number;
  image_width_px: number;
  image_height_px: number;
  origin: [number, number, number?];
}

interface WifiAnchor {
  id: string;
  x: number;
  y: number;
  label?: string | null;
  latencies_ms?: number[];
  created_at?: number;
}

interface WifiAnchorsPayload {
  anchors: WifiAnchor[];
  targets?: string[];
  updated_at?: number;
}

interface WifiPosePayload {
  x: number;
  y: number;
  anchor_id?: string;
  confidence?: number;
  distance?: number;
  latencies_ms?: number[];
  timestamp?: number;
}

interface WifiStatusPayload {
  state?: string;
  message?: string;
  timestamp?: number;
}

interface DockingRecord {
  record_id: string;
  timestamp: number;
  distance_m?: number;
  tag_id?: string;
  confidence?: number;
  label?: string;
  tag_size_m?: number;
  bbox?: {
    center: { x: number; y: number; theta: number };
    size_x: number;
    size_y: number;
  };
  pose?: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

interface DockingInfoPayload {
  records: DockingRecord[];
  updated_at?: number;
}

interface DockingStatusPayload {
  distance_m?: number;
  tag_id?: string;
  confidence?: number;
  timestamp?: number;
}

interface PixelPoint {
  x: number;
  y: number;
}

const FALLBACK_METADATA: MapMetadata = {
  resolution: 0.0069453925,
  image_width_px: 1896,
  image_height_px: 2864,
  origin: [0, 0, 0],
};

const estadoLabels: Record<string, string> = {
  disponible: 'Disponible',
  en_cola: 'En cola',
  esperando_nav2: 'Esperando Nav2',
  navegando: 'Navegando',
  completado: 'Completado',
  fallo: 'Falló',
};

const ACTIVE_STATES = new Set(['en_cola', 'esperando_nav2', 'navegando']);

function formatZoneName(id: string): string {
  return id
    .split('_')
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(' ');
}

function quaternionToYaw(q: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1 - 2.0 * (q.y * q.y + q.z * q.z));
}

function quaternionFromYaw(yaw: number) {
  const half = 0.5 * yaw;
  return {
    x: 0,
    y: 0,
    z: Math.sin(half),
    w: Math.cos(half),
  };
}

function worldToPixel(
  x: number,
  y: number,
  metadata: MapMetadata,
): PixelPoint | undefined {
  const { resolution, origin, image_width_px, image_height_px } = metadata;
  if (!resolution) {
    return undefined;
  }
  const originX = origin?.[0] ?? 0;
  const originY = origin?.[1] ?? 0;
  const px = (x - originX) / resolution;
  const py = (y - originY) / resolution;
  return {
    x: px,
    y: image_height_px - py,
  };
}

function pixelToWorld(
  x: number,
  y: number,
  metadata: MapMetadata,
): { x: number; y: number } | undefined {
  const { resolution, origin, image_height_px } = metadata;
  if (!resolution) {
    return undefined;
  }
  const originX = origin?.[0] ?? 0;
  const originY = origin?.[1] ?? 0;
  const worldX = x * resolution + originX;
  const worldY = (image_height_px - y) * resolution + originY;
  return { x: worldX, y: worldY };
}

function hashColor(id: string): string {
  let hash = 0;
  for (let i = 0; i < id.length; i += 1) {
    hash = (hash * 31 + id.charCodeAt(i)) & 0xffffffff;
  }
  const hue = Math.abs(hash) % 360;
  return `hsl(${hue}, 65%, 55%)`;
}

function sanitizeId(value: string): string {
  return value.replace(/[^a-zA-Z0-9_-]/g, '-');
}

function buildCoveragePath(points: PixelPoint[], spacingPx: number): string | undefined {
  if (!points.length || !Number.isFinite(spacingPx) || spacingPx <= 0) {
    return undefined;
  }
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const width = maxX - minX;
  const height = maxY - minY;
  const horizontal = width >= height;
  const span = horizontal ? height : width;
  if (span <= 0) {
    return undefined;
  }
  const steps = Math.max(1, Math.round(span / spacingPx));
  const commands: string[] = [];
  for (let i = 0; i <= steps; i += 1) {
    if (horizontal) {
      const y = minY + (height / Math.max(steps, 1)) * i;
      const startX = i % 2 === 0 ? minX : maxX;
      const endX = i % 2 === 0 ? maxX : minX;
      commands.push(`M ${startX.toFixed(1)} ${y.toFixed(1)} L ${endX.toFixed(1)} ${y.toFixed(1)}`);
    } else {
      const x = minX + (width / Math.max(steps, 1)) * i;
      const startY = i % 2 === 0 ? minY : maxY;
      const endY = i % 2 === 0 ? maxY : minY;
      commands.push(`M ${x.toFixed(1)} ${startY.toFixed(1)} L ${x.toFixed(1)} ${endY.toFixed(1)}`);
    }
  }
  return commands.join(' ');
}

export default function MapTab() {
  const { ros, status } = useRosConnection();
  const catalogState = useTopicSubscription<{ data: string }>(ZONE_CATALOG_TOPIC);
  const zoneState = useTopicSubscription<{ data: string }>(ZONE_STATE_TOPIC);
  const poseState = useTopicSubscription<{
    pose: {
      pose: {
        position: { x: number; y: number };
        orientation: { x: number; y: number; z: number; w: number };
      };
    };
  }>(ROBOT_POSE_TOPIC);
  const wifiAnchorsState = useTopicSubscription<{ data: string }>(WIFI_ANCHORS_TOPIC);
  const wifiPoseState = useTopicSubscription<{ data: string }>(WIFI_POSE_TOPIC);
  const wifiStatusState = useTopicSubscription<{ data: string }>(WIFI_STATUS_TOPIC);
  const dockingInfoState = useTopicSubscription<{ data: string }>(DOCKING_INFO_TOPIC);
  const dockingStatusState = useTopicSubscription<{ data: string }>(DOCKING_STATUS_TOPIC);

  const [hoveredZone, setHoveredZone] = useState<string>();
  const [selectedZone, setSelectedZone] = useState<string>();
  const [feedback, setFeedback] = useState<string>();
  const [wifiFeedback, setWifiFeedback] = useState<string>();
  const [dockingFeedback, setDockingFeedback] = useState<string>();
  const [anchorLabel, setAnchorLabel] = useState<string>('');
  const [dockLabel, setDockLabel] = useState<string>('');
  const [selectedPoint, setSelectedPoint] = useState<{
    pixel: PixelPoint;
    world: { x: number; y: number };
  } | null>(null);
  const [manualGoal, setManualGoal] = useState({ x: '', y: '', yaw: '0' });
  const [navFeedback, setNavFeedback] = useState<string>();
  const [fallbackCatalog, setFallbackCatalog] = useState<CatalogPayload | null>(null);
  const publisherRef = useRef<ROSLIB.Topic>();
  const wifiPublisherRef = useRef<ROSLIB.Topic>();
  const wifiStartRef = useRef<ROSLIB.Topic>();
  const dockingPublisherRef = useRef<ROSLIB.Topic>();
  const pendingCalibrateRef = useRef<{ x: number; y: number; label?: string } | null>(null);
  const pendingCalibrateTimerRef = useRef<number | null>(null);
  const navActionRef = useRef<any>();
  const navGoalRef = useRef<any>();

  useEffect(() => {
    let cancelled = false;
    fetch(FALLBACK_ZONES_URL)
      .then((resp) => resp.json())
      .then((data: CatalogPayload) => {
        if (!cancelled) {
          setFallbackCatalog(data);
        }
      })
      .catch((err) => console.warn('No se pudo cargar el fallback de zonas', err));
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    return () => {
      if (pendingCalibrateTimerRef.current) {
        window.clearTimeout(pendingCalibrateTimerRef.current);
      }
    };
  }, []);

  useEffect(() => {
    if (!ros) {
      navGoalRef.current?.cancel();
      navGoalRef.current = undefined;
      navActionRef.current = undefined;
      return;
    }
    const client = new (ROSLIB as any).ActionClient({
      ros,
      serverName: '/navigate_to_pose',
      actionName: 'nav2_msgs/action/NavigateToPose',
    });
    navActionRef.current = client;
    return () => {
      navGoalRef.current?.cancel();
      navGoalRef.current = undefined;
      if (navActionRef.current === client) {
        navActionRef.current = undefined;
      }
    };
  }, [ros]);

  const rosCatalog = useMemo<CatalogPayload | null>(() => {
    if (!catalogState.data?.data) {
      return null;
    }
    try {
      return JSON.parse(catalogState.data.data) as CatalogPayload;
    } catch (err) {
      console.warn('No se pudo parsear zone_catalog', err);
      return null;
    }
  }, [catalogState.data]);

  const catalog = rosCatalog ?? fallbackCatalog;

  const metadata: MapMetadata = useMemo(() => {
    const info = catalog?.metadata ?? fallbackCatalog?.metadata;
    return {
      resolution: info?.resolution ?? FALLBACK_METADATA.resolution,
      image_width_px: info?.image_width_px ?? FALLBACK_METADATA.image_width_px,
      image_height_px: info?.image_height_px ?? FALLBACK_METADATA.image_height_px,
      origin: (info?.origin as [number, number, number?]) ?? FALLBACK_METADATA.origin,
    };
  }, [catalog, fallbackCatalog]);

  const coverageSpacingPx = useMemo(() => {
    const px = COVERAGE_SPACING_M / (metadata.resolution || FALLBACK_METADATA.resolution);
    return Number.isFinite(px) && px > 4 ? px : 60;
  }, [metadata]);

  const formatDistance = (value?: number) =>
    typeof value === 'number' ? `${(value * 100).toFixed(1)} cm` : 'N/D';
  const formatTimestamp = (value?: number) =>
    value ? new Date(value * 1000).toLocaleTimeString() : 'Sin registro';

  const zones = useMemo<ZoneEntry[]>(() => {
    if (!catalog?.zones) {
      return [];
    }
    return [...catalog.zones].sort((a, b) => a.id.localeCompare(b.id));
  }, [catalog]);

  const selectedZoneEntry = useMemo(
    () => zones.find((zone) => zone.id === selectedZone),
    [zones, selectedZone],
  );

  const zoneGoal = useMemo(() => {
    if (!selectedZoneEntry?.centroid) {
      return null;
    }
    return {
      x: selectedZoneEntry.centroid[0],
      y: selectedZoneEntry.centroid[1],
      yaw: 0,
    };
  }, [selectedZoneEntry]);

  useEffect(() => {
    if (!selectedZone && zones.length > 0) {
      setSelectedZone(zones[0].id);
    } else if (selectedZone && !zones.some((zone) => zone.id === selectedZone)) {
      setSelectedZone(zones[0]?.id);
    }
  }, [zones, selectedZone]);

  const polygonPixels = useMemo(() => {
    const map = new Map<string, PixelPoint[]>();
    zones.forEach((zone) => {
      if (!zone.polygon) {
        return;
      }
      const points = zone.polygon
        .map((point) => worldToPixel(point[0], point[1], metadata))
        .filter((p): p is PixelPoint => Boolean(p));
      if (points.length) {
        map.set(zone.id, points);
      }
    });
    return map;
  }, [zones, metadata]);

  const parsedZoneState = useMemo<ZoneStatePayload | null>(() => {
    if (!zoneState.data?.data) {
      return null;
    }
    try {
      return JSON.parse(zoneState.data.data) as ZoneStatePayload;
    } catch (err) {
      console.warn('Estado de zona inválido', err);
      return null;
    }
  }, [zoneState.data]);

  const wifiAnchors = useMemo<WifiAnchorsPayload | null>(() => {
    if (!wifiAnchorsState.data?.data) {
      return null;
    }
    try {
      return JSON.parse(wifiAnchorsState.data.data) as WifiAnchorsPayload;
    } catch (err) {
      console.warn('Anchors WiFi invalidos', err);
      return null;
    }
  }, [wifiAnchorsState.data]);

  const wifiPose = useMemo<WifiPosePayload | null>(() => {
    if (!wifiPoseState.data?.data) {
      return null;
    }
    try {
      return JSON.parse(wifiPoseState.data.data) as WifiPosePayload;
    } catch (err) {
      console.warn('WiFi pose invalido', err);
      return null;
    }
  }, [wifiPoseState.data]);

  const wifiStatus = useMemo<WifiStatusPayload | null>(() => {
    if (!wifiStatusState.data?.data) {
      return null;
    }
    try {
      return JSON.parse(wifiStatusState.data.data) as WifiStatusPayload;
    } catch (err) {
      console.warn('WiFi status invalido', err);
      return null;
    }
  }, [wifiStatusState.data]);

  const dockingInfo = useMemo<DockingInfoPayload | null>(() => {
    if (!dockingInfoState.data?.data) {
      return null;
    }
    try {
      return JSON.parse(dockingInfoState.data.data) as DockingInfoPayload;
    } catch (err) {
      console.warn('Docking info inválido', err);
      return null;
    }
  }, [dockingInfoState.data]);

  const dockingRecords = dockingInfo?.records ?? [];

  const dockingStatus = useMemo<DockingStatusPayload | null>(() => {
    if (!dockingStatusState.data?.data) {
      return null;
    }
    try {
      return JSON.parse(dockingStatusState.data.data) as DockingStatusPayload;
    } catch (err) {
      console.warn('Docking status inválido', err);
      return null;
    }
  }, [dockingStatusState.data]);

  const robotMarker = useMemo(() => {
    const pose = poseState.data?.pose?.pose;
    if (!pose) {
      return null;
    }
    const coords = worldToPixel(pose.position.x, pose.position.y, metadata);
    if (!coords) {
      return null;
    }
    const yaw = quaternionToYaw(pose.orientation);
    const headingDeg = (-yaw * 180) / Math.PI;
    return { ...coords, headingDeg };
  }, [poseState.data, metadata]);

  const wifiMarker = useMemo(() => {
    if (!wifiPose) {
      return null;
    }
    if (!Number.isFinite(wifiPose.x) || !Number.isFinite(wifiPose.y)) {
      return null;
    }
    const coords = worldToPixel(wifiPose.x, wifiPose.y, metadata);
    if (!coords) {
      return null;
    }
    return { ...coords, confidence: wifiPose.confidence ?? 0 };
  }, [wifiPose, metadata]);

  const wifiAnchorsPixels = useMemo(() => {
    if (!wifiAnchors?.anchors?.length) {
      return [];
    }
    return wifiAnchors.anchors
      .map((anchor) => {
        if (!Number.isFinite(anchor.x) || !Number.isFinite(anchor.y)) {
          return null;
        }
        const coords = worldToPixel(anchor.x, anchor.y, metadata);
        if (!coords) {
          return null;
        }
        return { ...coords, anchor };
      })
      .filter((item): item is { anchor: WifiAnchor; x: number; y: number } => Boolean(item));
  }, [wifiAnchors, metadata]);

  const dockMarker = useMemo(() => {
    const dockSource = catalog?.dock_pose ?? fallbackCatalog?.dock_pose;
    if (!dockSource) {
      return null;
    }
    const coords = worldToPixel(dockSource.x, dockSource.y, metadata);
    if (!coords) {
      return null;
    }
    const headingDeg =
      typeof dockSource.theta === 'number'
        ? (-dockSource.theta * 180) / Math.PI
        : -90;
    return { ...coords, headingDeg };
  }, [catalog, fallbackCatalog, metadata]);

  useEffect(() => {
    if (!ros) {
      publisherRef.current?.unadvertise();
      publisherRef.current = undefined;
      return;
    }
    const topic = new ROSLIB.Topic({
      ros,
      name: ZONE_GOAL_TOPIC,
      messageType: 'std_msgs/msg/String',
    });
    topic.advertise();
    publisherRef.current = topic;
    return () => {
      topic.unadvertise();
      if (publisherRef.current === topic) {
        publisherRef.current = undefined;
      }
    };
  }, [ros]);

  useEffect(() => {
    if (!ros) {
      wifiStartRef.current?.unadvertise();
      wifiStartRef.current = undefined;
      return;
    }
    const topic = new ROSLIB.Topic({
      ros,
      name: WIFI_START_TOPIC,
      messageType: 'std_msgs/msg/Bool',
    });
    topic.advertise();
    wifiStartRef.current = topic;
    return () => {
      topic.unadvertise();
      if (wifiStartRef.current === topic) {
        wifiStartRef.current = undefined;
      }
    };
  }, [ros]);

  useEffect(() => {
    if (!ros) {
      wifiPublisherRef.current?.unadvertise();
      wifiPublisherRef.current = undefined;
      return;
    }
    const topic = new ROSLIB.Topic({
      ros,
      name: WIFI_CALIBRATE_TOPIC,
      messageType: 'std_msgs/msg/String',
    });
    topic.advertise();
    wifiPublisherRef.current = topic;
    return () => {
      topic.unadvertise();
      if (wifiPublisherRef.current === topic) {
        wifiPublisherRef.current = undefined;
      }
    };
  }, [ros]);

  useEffect(() => {
    if (!ros) {
      dockingPublisherRef.current?.unadvertise();
      dockingPublisherRef.current = undefined;
      return;
    }
    const topic = new ROSLIB.Topic({
      ros,
      name: DOCKING_CALIBRATE_TOPIC,
      messageType: 'std_msgs/msg/String',
    });
    topic.advertise();
    dockingPublisherRef.current = topic;
    return () => {
      topic.unadvertise();
      if (dockingPublisherRef.current === topic) {
        dockingPublisherRef.current = undefined;
      }
    };
  }, [ros]);

  const sendZoneGoal = (zoneId: string) => {
    if (!publisherRef.current) {
      setFeedback('ROS no disponible para enviar zonas.');
      return;
    }
    publisherRef.current.publish(new ROSLIB.Message({ data: zoneId }));
    setFeedback(`Comando enviado: limpiar ${formatZoneName(zoneId)}.`);
  };

  const sendNavGoalMessage = (target: { x: number; y: number; yaw: number }) => {
    if (!ros) {
      setNavFeedback('ROS no disponible para enviar goals.');
      return;
    }
    const actionClient = navActionRef.current;
    if (!actionClient) {
      setNavFeedback('Action server de Nav2 no disponible.');
      return;
    }
    const orientation = quaternionFromYaw(target.yaw || 0);
    const goalMessage = {
      pose: {
        header: {
          frame_id: 'map',
          stamp: { secs: 0, nanosec: 0 },
        },
        pose: {
          position: { x: target.x, y: target.y, z: 0 },
          orientation,
        },
      },
      behavior_tree: '',
    };
    navGoalRef.current?.cancel();
    const goal = new (ROSLIB as any).Goal({
      actionClient,
      goalMessage,
    });
    goal.on('feedback', (feedback: any) => {
      if (typeof feedback?.distance_remaining === 'number') {
        setNavFeedback(`Distancia restante: ${feedback.distance_remaining.toFixed(2)} m`);
      } else {
        setNavFeedback('Nav2 publicó feedback.');
      }
    });
    goal.on('result', (result: any) => {
      navGoalRef.current = undefined;
      const errorCode = result?.result?.error_code;
      const errorMsg = result?.result?.error_msg;
      if (typeof errorCode === 'number' && errorCode !== 0) {
        setNavFeedback(`Nav2 finalizó con error ${errorCode}: ${errorMsg ?? 'sin mensaje'}`);
      } else {
        setNavFeedback('Nav2 alcanzó el objetivo.');
      }
    });
    navGoalRef.current = goal;
    goal.send();
    setNavFeedback(`Goal enviado: (${target.x.toFixed(2)}, ${target.y.toFixed(2)})`);
  };

  const parseManualTarget = () => {
    const x = Number(manualGoal.x);
    const y = Number(manualGoal.y);
    const yaw = Number(manualGoal.yaw);
    if (!Number.isFinite(x)) {
      setNavFeedback('Coordenada X inválida.');
      return null;
    }
    if (!Number.isFinite(y)) {
      setNavFeedback('Coordenada Y inválida.');
      return null;
    }
    if (!Number.isFinite(yaw)) {
      setNavFeedback('Yaw inválido.');
      return null;
    }
    return { x, y, yaw };
  };

  const handleZoneNavigation = () => {
    if (!zoneGoal) {
      setNavFeedback('Seleccioná una zona con centroid para navegar.');
      return;
    }
    sendNavGoalMessage(zoneGoal);
  };

  const handleManualNavigation = () => {
    const target = parseManualTarget();
    if (!target) {
      return;
    }
    sendNavGoalMessage(target);
  };

  const copyZoneToManual = () => {
    if (!zoneGoal) {
      setNavFeedback('No hay zona con centroid seleccionada.');
      return;
    }
    setManualGoal({
      x: zoneGoal.x.toFixed(2),
      y: zoneGoal.y.toFixed(2),
      yaw: zoneGoal.yaw.toFixed(2),
    });
    setNavFeedback('Coordenadas copiadas desde la zona seleccionada.');
  };

  const handleManualInput =
    (field: 'x' | 'y' | 'yaw') => (event: ChangeEvent<HTMLInputElement>) => {
      setManualGoal((prev) => ({ ...prev, [field]: event.target.value }));
    };

  const publishWifiCalibrateMessage = (payload: { x: number; y: number; label?: string }) => {
    if (!wifiPublisherRef.current) {
      setWifiFeedback('ROS no disponible para calibrar WiFi.');
      return;
    }
    wifiPublisherRef.current.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
    setWifiFeedback('Calibración enviada.');
  };

  const schedulePendingCalibrate = (payload: { x: number; y: number; label?: string }) => {
    pendingCalibrateRef.current = payload;
    if (pendingCalibrateTimerRef.current) {
      window.clearTimeout(pendingCalibrateTimerRef.current);
    }
    pendingCalibrateTimerRef.current = window.setTimeout(() => {
      pendingCalibrateTimerRef.current = null;
      if (pendingCalibrateRef.current) {
        publishWifiCalibrateMessage(pendingCalibrateRef.current);
        pendingCalibrateRef.current = null;
      }
    }, 2500);
  };

  const requestWifiStart = () => {
    if (!wifiStartRef.current) {
      setWifiFeedback('ROS no disponible para iniciar localización WiFi.');
      return false;
    }
    wifiStartRef.current.publish(new ROSLIB.Message({ data: true }));
    setWifiFeedback('Activando nodo de localización WiFi...');
    return true;
  };

  useEffect(() => {
    if (!pendingCalibrateRef.current) {
      return;
    }
    if (!wifiStatus || wifiStatus.state === 'error') {
      return;
    }
    if (pendingCalibrateTimerRef.current) {
      window.clearTimeout(pendingCalibrateTimerRef.current);
      pendingCalibrateTimerRef.current = null;
    }
    const payload = pendingCalibrateRef.current;
    pendingCalibrateRef.current = null;
    publishWifiCalibrateMessage(payload);
  }, [wifiStatus]);

  const handleMapClick: MouseEventHandler<HTMLDivElement> = (event) => {
    const rect = event.currentTarget.getBoundingClientRect();
    const x = ((event.clientX - rect.left) / rect.width) * metadata.image_width_px;
    const y = ((event.clientY - rect.top) / rect.height) * metadata.image_height_px;
    const world = pixelToWorld(x, y, metadata);
    if (!world) {
      setWifiFeedback('No se pudo calcular la posicion en el mapa.');
      return;
    }
    setSelectedPoint({ pixel: { x, y }, world });
    setWifiFeedback(`Punto seleccionado: (${world.x.toFixed(2)}, ${world.y.toFixed(2)}).`);
  };

  const sendWifiCalibration = () => {
    if (!selectedPoint) {
      setWifiFeedback('Selecciona un punto en el mapa antes de calibrar.');
      return;
    }
    const label = anchorLabel.trim();
    const payload = {
      x: selectedPoint.world.x,
      y: selectedPoint.world.y,
      label: label || undefined,
    };
    const needsActivation =
      !wifiStatus || wifiStatus.state === undefined || wifiStatus.state === 'error';
    if (needsActivation) {
      const started = requestWifiStart();
      if (started) {
        schedulePendingCalibrate(payload);
        return;
      }
      setWifiFeedback('No se pudo activar el nodo WiFi.');
      return;
    }
    publishWifiCalibrateMessage(payload);
  };

  const clearSelectedPoint = () => {
    setSelectedPoint(null);
    setWifiFeedback('Seleccion de punto limpiada.');
  };

  const sendDockingCalibration = () => {
    if (!dockingPublisherRef.current) {
      setDockingFeedback('ROS no disponible para guardar info de docking.');
      return;
    }
    const payload = {
      label: dockLabel.trim() || undefined,
    };
    dockingPublisherRef.current.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
    setDockLabel('');
    setDockingFeedback('Información del dock registrada.');
  };

  useEffect(() => {
    if (!feedback) {
      return;
    }
    const timer = window.setTimeout(() => setFeedback(undefined), 4000);
    return () => window.clearTimeout(timer);
  }, [feedback]);

  useEffect(() => {
    if (!wifiFeedback) {
      return;
    }
    const timer = window.setTimeout(() => setWifiFeedback(undefined), 5000);
    return () => window.clearTimeout(timer);
  }, [wifiFeedback]);

  useEffect(() => {
    if (!dockingFeedback) {
      return;
    }
    const timer = window.setTimeout(() => setDockingFeedback(undefined), 5000);
    return () => window.clearTimeout(timer);
  }, [dockingFeedback]);

  const currentZone = parsedZoneState?.zona ?? undefined;
  const currentState = parsedZoneState?.estado ?? 'disponible';
  const humanState = estadoLabels[currentState] ?? currentState;
  const isBusy = Boolean(currentZone && ACTIVE_STATES.has(currentState));
  const catalogSource = rosCatalog ? 'ROS (zone_waypoint_node)' : 'archivo estático';
  const wifiAnchorCount = wifiAnchors?.anchors?.length ?? 0;
  const wifiConfidence =
    wifiPose?.confidence !== undefined
      ? `${Math.round(wifiPose.confidence * 100)}%`
      : 'N/D';
  const wifiStatusText = wifiStatus?.message ?? 'Sin estado';
  const latestDock = dockingRecords[dockingRecords.length - 1];
  const dockingRecordsSorted = [...dockingRecords].reverse();
  const currentDistance = dockingStatus?.distance_m ?? latestDock?.distance_m;
  const currentTag = dockingStatus?.tag_id ?? latestDock?.tag_id;
  const currentConfidence =
    dockingStatus?.confidence ?? latestDock?.confidence;

  return (
    <div className="map-layout">
      <section className="card map-view">
        <div className="map-header">
          <div>
            <h3>Mapa del hogar</h3>
            <p style={{ margin: 0, color: '#9ba7c1' }}>
              Basado en el mapa de Nav2, zonas de limpieza y trayectoria desde la base ({catalogSource}).
            </p>
          </div>
          <span className={`status-badge ${status === 'conectado' ? 'ok' : 'warn'}`}>
            {status === 'conectado' ? 'ROS conectado' : 'ROS no disponible'}
          </span>
        </div>
        <div
          className={`map-frame ${selectedPoint ? 'has-selection' : ''}`}
          style={{
            backgroundImage: `url(${MAP_IMAGE_URL})`,
            aspectRatio: `${metadata.image_width_px} / ${metadata.image_height_px}`,
          }}
          onClick={handleMapClick}
        >
          <svg viewBox={`0 0 ${metadata.image_width_px} ${metadata.image_height_px}`} className="map-overlay">
            <defs>
              {zones.map((zone) => {
                const poly = polygonPixels.get(zone.id);
                if (!poly?.length) {
                  return null;
                }
                const clipId = `clip-${sanitizeId(zone.id)}`;
                const points = poly.map((p) => `${p.x},${p.y}`).join(' ');
                return (
                  <clipPath key={clipId} id={clipId}>
                    <polygon points={points} />
                  </clipPath>
                );
              })}
            </defs>
            {zones.map((zone) => {
              const polygon = polygonPixels.get(zone.id);
              if (!polygon?.length) {
                return null;
              }
              const clipId = `clip-${sanitizeId(zone.id)}`;
              const points = polygon.map((p) => `${p.x},${p.y}`).join(' ');
              const active = zone.id === currentZone;
              const hovered = zone.id === hoveredZone;
              const color = hashColor(zone.id);
              const coveragePath = buildCoveragePath(polygon, coverageSpacingPx);
              const centroid =
                zone.centroid && worldToPixel(zone.centroid[0], zone.centroid[1], metadata);
              const highlight = active || hovered || selectedZone === zone.id;
              return (
                <g key={zone.id}>
                  <polygon
                    points={points}
                    className={[
                      'zone-polygon',
                      active ? 'active' : '',
                      hovered ? 'hovered' : '',
                    ]
                      .filter(Boolean)
                      .join(' ')}
                    style={{ stroke: color, fill: `${color}22` }}
                  />
                  {coveragePath && (
                    <path
                      d={coveragePath}
                      clipPath={`url(#${clipId})`}
                      className={`coverage-path ${highlight ? 'active' : ''}`}
                      style={{ stroke: color }}
                    />
                  )}
                  {dockMarker && centroid && (
                    <line
                      x1={dockMarker.x}
                      y1={dockMarker.y}
                      x2={centroid.x}
                      y2={centroid.y}
                      className={`route-line ${highlight ? 'active' : ''}`}
                      style={{ stroke: color }}
                    />
                  )}
                  {centroid && (
                    <circle
                      cx={centroid.x}
                      cy={centroid.y}
                      r={highlight ? 10 : 7}
                      className="zone-centroid"
                      style={{ stroke: color }}
                    />
                  )}
                </g>
              );
            })}
            {dockMarker && (
              <g
                className="dock-marker"
                transform={`translate(${dockMarker.x} ${dockMarker.y}) rotate(${dockMarker.headingDeg})`}
              >
                <rect x="-10" y="-6" width="20" height="12" rx="3" />
                <polygon points="0,-28 12,-6 -12,-6" />
                <text x="0" y="25" textAnchor="middle">
                  Dock
                </text>
              </g>
            )}
            {wifiAnchorsPixels.map((item) => (
              <g key={item.anchor.id} className="anchor-marker" transform={`translate(${item.x} ${item.y})`}>
                <circle cx="0" cy="0" r="7" />
                <text x="12" y="4">
                  {item.anchor.label ?? item.anchor.id}
                </text>
              </g>
            ))}
            {wifiMarker && (
              <g className="wifi-marker" transform={`translate(${wifiMarker.x} ${wifiMarker.y})`}>
                <circle cx="0" cy="0" r="18" />
                <circle cx="0" cy="0" r="6" />
                <text x="0" y="28" textAnchor="middle">
                  WiFi {Math.round(wifiMarker.confidence * 100)}%
                </text>
              </g>
            )}
            {robotMarker && (
              <g
                className="robot-indicator"
                transform={`translate(${robotMarker.x} ${robotMarker.y}) rotate(${robotMarker.headingDeg})`}
              >
                <polygon points="0,-26 13,16 -13,16" />
                <circle cx="0" cy="0" r="8" />
              </g>
            )}
            {selectedPoint && (
              <g
                className="calibration-point"
                transform={`translate(${selectedPoint.pixel.x} ${selectedPoint.pixel.y})`}
              >
                <line x1="-12" y1="0" x2="12" y2="0" />
                <line x1="0" y1="-12" x2="0" y2="12" />
                <circle cx="0" cy="0" r="6" />
              </g>
            )}
          </svg>
        </div>
        <small style={{ color: '#70809f' }}>
          Resolución {metadata.resolution.toFixed(4)} m/px — {metadata.image_width_px}×
          {metadata.image_height_px}px — Espaciado de barrido {COVERAGE_SPACING_M.toFixed(2)} m.
        </small>
      </section>
      <section className="card zone-panel">
        <h3>Limpieza por ambientes</h3>
        <p>
          Estado actual:{' '}
          <strong>
            {humanState}
            {currentZone ? ` (${formatZoneName(currentZone)})` : ''}
          </strong>
        </p>
        {parsedZoneState?.timestamp && (
          <small style={{ color: '#70809f' }}>
            Última actualización: {new Date(parsedZoneState.timestamp * 1000).toLocaleTimeString()}
          </small>
        )}
        <div className="zone-list-header">
          <strong>Zonas detectadas ({zones.length})</strong>
          <small style={{ color: '#70809f' }}>
            Fuente: {catalogSource}. Tocá un ambiente para previsualizar el recorrido; el botón envía la limpieza.
          </small>
        </div>
        <ul className="zone-list">
          {zones.map((zone) => {
            const label = formatZoneName(zone.id);
            const active = currentZone === zone.id && isBusy;
            const areaText =
              typeof zone.area_m2 === 'number' ? `${zone.area_m2.toFixed(1)} m²` : 'Área desconocida';
            const color = hashColor(zone.id);
            return (
              <li
                key={zone.id}
                className={`zone-item ${active || selectedZone === zone.id ? 'active' : ''}`}
                onMouseEnter={() => setHoveredZone(zone.id)}
                onMouseLeave={() => setHoveredZone(undefined)}
                onClick={() => setSelectedZone(zone.id)}
              >
                <div className="zone-item-info">
                  <span className="zone-color-dot" style={{ background: color }} />
                  <div>
                    <strong>{label}</strong>
                    <div style={{ fontSize: '0.85rem', color: '#9ba7c1' }}>{areaText}</div>
                  </div>
                </div>
                <button
                  type="button"
                  className={`secondary ${active ? 'toggle active' : ''}`}
                  onClick={(event) => {
                    event.stopPropagation();
                    sendZoneGoal(zone.id);
                  }}
                  disabled={!ros || active}
                >
                  {active ? 'En progreso' : 'Limpiar'}
                </button>
              </li>
            );
          })}
          {zones.length === 0 && (
            <li className="zone-item">
              <span>No se recibió el catálogo de zonas. Verificá que el nodo zone_waypoint esté activo.</span>
            </li>
          )}
        </ul>
        <div className="nav-panel">
          <h4>Navegación Nav2</h4>
          <p style={{ marginBottom: '0.5rem', color: '#9ba7c1' }}>
            Envía objetivos de navegación al action server <code>/navigate_to_pose</code> usando la zona
            seleccionada o coordenadas manuales.
          </p>
          <div className="nav-zone-info">
            <span>
              Zona activa:&nbsp;
              <strong>{selectedZone ? formatZoneName(selectedZone) : 'Ninguna'}</strong>
              {zoneGoal && selectedZone && (
                <small style={{ marginLeft: '0.3rem', color: '#9ba7c1' }}>
                  ({zoneGoal.x.toFixed(2)}, {zoneGoal.y.toFixed(2)})
                </small>
              )}
            </span>
          </div>
          <div className="nav-buttons">
            <button
              type="button"
              className="secondary"
              onClick={handleZoneNavigation}
              disabled={!ros || !zoneGoal}
            >
              Navegar a la zona seleccionada
            </button>
            <button
              type="button"
              className="secondary"
              onClick={copyZoneToManual}
              disabled={!zoneGoal}
            >
              Copiar coordenadas al objetivo manual
            </button>
          </div>
          <div className="nav-inputs">
            <label>
              Objetivo X (m)
              <input
                type="text"
                placeholder="0.00"
                value={manualGoal.x}
                onChange={handleManualInput('x')}
              />
            </label>
            <label>
              Objetivo Y (m)
              <input
                type="text"
                placeholder="0.00"
                value={manualGoal.y}
                onChange={handleManualInput('y')}
              />
            </label>
            <label>
              Yaw (rad)
              <input
                type="text"
                placeholder="0"
                value={manualGoal.yaw}
                onChange={handleManualInput('yaw')}
              />
            </label>
          </div>
          <div className="nav-buttons">
            <button
              type="button"
              className="secondary"
              onClick={handleManualNavigation}
              disabled={!ros}
            >
              Enviar goal manual
            </button>
            <small style={{ color: '#9ba7c1' }}>
              Las coordenadas se interpretan en el frame <code>map</code>.
            </small>
          </div>
          {navFeedback && <small className="nav-feedback">{navFeedback}</small>}
        </div>
        <div className="wifi-panel">
          <h4>Ubicación por WiFi</h4>
          <p style={{ marginBottom: '0.5rem' }}>
            Estimación actual:{' '}
            <strong>
              {wifiPose ? `(${wifiPose.x.toFixed(2)}, ${wifiPose.y.toFixed(2)})` : 'Sin datos'}
            </strong>{' '}
            — Confianza: {wifiConfidence}
          </p>
          <small style={{ color: '#9ba7c1' }}>
            Selecciona un punto en el mapa y presiona "Calibrar punto" para guardarlo como ancla.
          </small>
          <div className="wifi-actions">
            <input
              className="wifi-input"
              type="text"
              placeholder="Etiqueta (opcional)"
              value={anchorLabel}
              onChange={(event) => setAnchorLabel(event.target.value)}
            />
            <div className="wifi-buttons">
              <button
                type="button"
                className="secondary"
                onClick={sendWifiCalibration}
                disabled={!ros || !selectedPoint}
              >
                Calibrar punto
              </button>
              <button type="button" className="secondary danger" onClick={clearSelectedPoint}>
                Limpiar selección
              </button>
            </div>
          </div>
          <div className="wifi-anchor-list">
            <strong>Anchors guardados ({wifiAnchorCount})</strong>
            <ul>
              {(wifiAnchors?.anchors ?? []).map((anchor) => (
                <li key={anchor.id}>
                  <span>{anchor.label ?? anchor.id}</span>
                  <small>
                    ({typeof anchor.x === 'number' ? anchor.x.toFixed(2) : '?'},
                    {typeof anchor.y === 'number' ? anchor.y.toFixed(2) : '?'})
                    {anchor.latencies_ms && anchor.latencies_ms.length > 0 && (
                      <>
                        {' — Intensidades: '}
                        {anchor.latencies_ms.map((value) => `${value.toFixed(1)}ms`).join(', ')}
                      </>
                    )}
                  </small>
                </li>
              ))}
              {wifiAnchorCount === 0 && <li>No hay anchors aún.</li>}
            </ul>
          </div>
          <div className="dock-panel">
            <h4>Dock y carga</h4>
            <p style={{ margin: '0 0 0.25rem' }}>
              Distancia actual: {formatDistance(currentDistance)} — Tag detectado: {currentTag ?? 'No detectado'}
              {currentConfidence !== undefined && ` (${Math.round(currentConfidence * 100)}%)`}
            </p>
            <div className="dock-actions">
              <input
                className="wifi-input"
                type="text"
                placeholder="Etiqueta para el dock"
                value={dockLabel}
                onChange={(event) => setDockLabel(event.target.value)}
              />
              <button
                type="button"
                className="secondary"
                onClick={sendDockingCalibration}
                disabled={!ros}
              >
                Guardar info del dock
              </button>
            </div>
            <ul className="dock-record-list">
              {dockingRecordsSorted.length > 0 ? (
                dockingRecordsSorted.map((record) => (
                  <li key={record.record_id}>
                    <div className="dock-record-header">
                      <strong>{record.label ?? 'Dock'}</strong>
                      <span>
                        {formatDistance(record.distance_m)} · {formatTimestamp(record.timestamp)}
                      </span>
                    </div>
                    <div className="dock-record-body">
                      <span>Tag: {record.tag_id ?? 'sin detección'}</span>
                      {record.confidence !== undefined && (
                        <small>Confianza: {Math.round(record.confidence * 100)}%</small>
                      )}
                      {record.tag_size_m && (
                        <small>Size: {(record.tag_size_m * 100).toFixed(1)} cm</small>
                      )}
                    </div>
                  </li>
                ))
              ) : (
                <li>No hay calibraciones registradas.</li>
              )}
            </ul>
            <small style={{ color: '#9ba7c1' }}>{dockingFeedback ?? 'Estado: listo para grabar.'}</small>
          </div>
          <small style={{ color: '#9ba7c1' }}>Estado WiFi: {wifiStatusText}</small>
          {wifiFeedback && <small style={{ color: '#9ba7c1' }}>{wifiFeedback}</small>}
        </div>
        {feedback && <small style={{ color: '#9ba7c1' }}>{feedback}</small>}
      </section>
    </div>
  );
}
