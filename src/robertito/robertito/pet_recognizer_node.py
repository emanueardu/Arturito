"""Reconoce mascotas en el stream de cámara y emite saludos por TTS.

Pipeline:
  /arturito/camera/faces/image  ──subscribe──> PetRecognizerNode
                                                │
                                                ├── MobileNet-SSD VOC (cat/dog)
                                                ├── classify_cat() por % naranja
                                                │   (Loli calicó vs Pipi tabby)
                                                ├── dog → Joky (única opción)
                                                │
                                                ▼ /assistant/say
                                          "[pet_recognizer] ¡Hola Loli!"

El recognizer solo corre cuando NO hay otra actividad (idle), para no pisarse
con conversaciones. Cooldown por mascota evita saludar 100 veces seguidas.
"""
import os
import random
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


# Clases del modelo MobileNet-SSD VOC (orden del prototxt original).
_VOC_CLASSES = (
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle',
    'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog',
    'horse', 'motorbike', 'person', 'pottedplant', 'sheep',
    'sofa', 'train', 'tvmonitor',
)
_CAT_IDX = _VOC_CLASSES.index('cat')
_DOG_IDX = _VOC_CLASSES.index('dog')
_PERSON_IDX = _VOC_CLASSES.index('person')

_GREETINGS = {
    'loli': [
        '¡Hola Loli!',
        '¡Loli, ahí estás!',
        '¡Mi Loli linda!',
        '¡Hola gatita!',
        '¡Ey, Loli!',
    ],
    'pipi': [
        '¡Hola Pipi!',
        '¡Pipi, viniste!',
        '¡Hola Pipi, gatito!',
        '¡Ahí está Pipi!',
    ],
    'joky': [
        '¡Hola Joky!',
        '¡Joky! ¿Cómo estás?',
        '¡Hola perrito!',
        '¡Joky lindo!',
    ],
}


class PetRecognizerNode(Node):
    """Detecta y saluda a Loli, Pipi y Joky cuando aparecen en cámara."""

    def __init__(self) -> None:
        super().__init__('pet_recognizer_node')

        # ── Parametros ──────────────────────────────────────────────
        self._caffemodel = str(
            self.declare_parameter('mobilenet_caffemodel', '').value
        )
        self._prototxt = str(
            self.declare_parameter('mobilenet_prototxt', '').value
        )
        self._conf_threshold = float(
            self.declare_parameter('conf_threshold', 0.5).value
        )
        self._check_interval_s = float(
            self.declare_parameter('check_interval_s', 3.0).value
        )
        self._cooldown_s = float(
            self.declare_parameter('greet_cooldown_s', 60.0).value
        )
        # Minimo % de area del bbox respecto al frame para considerar la
        # detección (evita gatos lejanos / falsos positivos chicos).
        self._min_bbox_area_pct = float(
            self.declare_parameter('min_bbox_area_pct', 0.02).value
        )
        # Thresholds del clasificador de gato por color naranja.
        self._orange_loli_min = float(
            self.declare_parameter('orange_loli_min', 0.05).value
        )
        self._orange_pipi_max = float(
            self.declare_parameter('orange_pipi_max', 0.02).value
        )
        self._image_topic = str(
            self.declare_parameter('image_topic', '/arturito/camera/faces/image').value
        )
        self._tts_topic = str(
            self.declare_parameter('tts_topic', '/assistant/say').value
        )
        # Si listening/speaking estan activos, no saludar para no pisarse.
        self._listening_topic = str(
            self.declare_parameter('listening_topic', '/behavior/listening_active').value
        )
        self._speaking_topic = str(
            self.declare_parameter('speaking_topic', '/behavior/speaking_active').value
        )
        # Flag de presencia humana: TRUE mientras MobileNet-SSD vea person en el
        # frame, FALSE cuando deja de verla por más de person_clear_after_s.
        # NO saluda — sólo emite flag (otros nodos como presence_orchestrator
        # pueden consumirlo). El wake word sigue siendo la vía de saludo.
        self._person_topic = str(
            self.declare_parameter('person_topic', '/arturito/person_detected').value
        )
        self._person_min_bbox_area_pct = float(
            self.declare_parameter('person_min_bbox_area_pct', 0.03).value
        )
        self._person_clear_after_s = float(
            self.declare_parameter('person_clear_after_s', 8.0).value
        )

        # ── Cargar modelo ──────────────────────────────────────────
        if not (self._caffemodel and self._prototxt
                and os.path.isfile(self._caffemodel)
                and os.path.isfile(self._prototxt)):
            self.get_logger().error(
                f'MobileNet-SSD no encontrado (caffemodel={self._caffemodel}, '
                f'prototxt={self._prototxt}); el nodo no detectará nada.'
            )
            self._net = None
        else:
            self._net = cv2.dnn.readNetFromCaffe(self._prototxt, self._caffemodel)
            self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().info(
                f'MobileNet-SSD VOC cargado desde {os.path.basename(self._caffemodel)}'
            )

        # ── ROS ─────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._tts_pub = self.create_publisher(String, self._tts_topic, 10)
        self._person_pub = self.create_publisher(Bool, self._person_topic, 10)

        self.create_subscription(Image, self._image_topic, self._on_image, 10)
        self.create_subscription(Bool, self._listening_topic, self._on_listening, 10)
        self.create_subscription(Bool, self._speaking_topic, self._on_speaking, 10)

        self._listening_active = False
        self._speaking_active = False
        self._last_greet_per_pet: Dict[str, float] = {}
        self._person_flag = False
        self._last_person_seen_at = 0.0

        if self._net is not None:
            self._timer = self.create_timer(self._check_interval_s, self._tick)
            self.get_logger().info(
                f'pet_recognizer listo: cada {self._check_interval_s:.1f}s, '
                f'cooldown {self._cooldown_s:.0f}s/mascota'
            )

    # ─────────────────────────────────────────── ROS subs
    def _on_image(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        with self._frame_lock:
            self._latest_frame = img

    def _on_listening(self, msg: Bool) -> None:
        self._listening_active = bool(msg.data)

    def _on_speaking(self, msg: Bool) -> None:
        self._speaking_active = bool(msg.data)

    # ─────────────────────────────────────────── Tick
    def _tick(self) -> None:
        if self._listening_active or self._speaking_active:
            return
        with self._frame_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
        if frame is None:
            return

        detections = self._detect(frame)

        # Person detection: actualiza flag de presencia (no saluda).
        # El flag se setea TRUE en cuanto se ve persona y se baja a FALSE
        # cuando pasaron person_clear_after_s sin verla.
        now = time.monotonic()
        person_seen = any(d[0] == 'person' for d in detections)
        if person_seen:
            self._last_person_seen_at = now
            if not self._person_flag:
                self._person_flag = True
                self._person_pub.publish(Bool(data=True))
                self.get_logger().info('person_detected → TRUE')
        elif (
            self._person_flag
            and (now - self._last_person_seen_at) > self._person_clear_after_s
        ):
            self._person_flag = False
            self._person_pub.publish(Bool(data=False))
            self.get_logger().info('person_detected → FALSE')

        pet_detections = [d for d in detections if d[0] in ('cat', 'dog')]
        self.get_logger().info(
            f'tick: frame {frame.shape[1]}x{frame.shape[0]} '
            f'person={person_seen} pets={len(pet_detections)}'
        )
        if not pet_detections:
            return

        # Mascotas: saludo con cooldown por individuo.
        pet_detections.sort(key=lambda d: d[1], reverse=True)
        for cls, conf, x1, y1, x2, y2 in pet_detections:
            crop = frame[max(0, y1):y2, max(0, x1):x2]
            if crop.size == 0:
                continue
            pet_name = (
                self._classify_cat(crop) if cls == 'cat'
                else 'joky' if cls == 'dog'
                else None
            )
            if pet_name is None:
                continue
            last_t = self._last_greet_per_pet.get(pet_name, 0.0)
            if now - last_t < self._cooldown_s:
                continue
            self._last_greet_per_pet[pet_name] = now
            phrase = random.choice(_GREETINGS.get(pet_name, [f'¡Hola {pet_name}!']))
            self.get_logger().info(
                f'Saludo: "{phrase}" (cls={cls} conf={conf:.2f} bbox={x2-x1}x{y2-y1})'
            )
            self._tts_pub.publish(String(data=f'[pet_recognizer] {phrase}'))
            return  # un saludo por tick

    # ─────────────────────────────────────────── Detección
    def _detect(
        self, frame: np.ndarray
    ) -> List[Tuple[str, float, int, int, int, int]]:
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, 0.007843, (300, 300),
            (127.5, 127.5, 127.5), False, False,
        )
        try:
            self._net.setInput(blob)
            out = self._net.forward()
        except Exception as exc:
            self.get_logger().warn(
                f'MobileNet forward fallo: {exc}', throttle_duration_sec=10.0,
            )
            return []
        results = []
        for d in out[0, 0]:
            cls_id = int(d[1])
            conf = float(d[2])
            if conf < self._conf_threshold:
                continue
            if cls_id == _CAT_IDX:
                cls, min_area = 'cat', self._min_bbox_area_pct
            elif cls_id == _DOG_IDX:
                cls, min_area = 'dog', self._min_bbox_area_pct
            elif cls_id == _PERSON_IDX:
                cls, min_area = 'person', self._person_min_bbox_area_pct
            else:
                continue
            x1 = max(0, int(d[3] * w))
            y1 = max(0, int(d[4] * h))
            x2 = min(w - 1, int(d[5] * w))
            y2 = min(h - 1, int(d[6] * h))
            if x2 <= x1 or y2 <= y1:
                continue
            area_pct = ((x2 - x1) * (y2 - y1)) / float(w * h)
            if area_pct < min_area:
                continue
            results.append((cls, conf, x1, y1, x2, y2))
        return results

    # ─────────────────────────────────────────── Clasificador gato
    def _classify_cat(self, crop_bgr: np.ndarray) -> Optional[str]:
        """Distingue Loli (calicó con naranja) de Pipi (tabby gris)
        usando el % de pixels naranjas en el bbox. None si es ambiguo
        (puede ser otro gato distinto)."""
        try:
            hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
        except cv2.error:
            return None
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        total = crop_bgr.shape[0] * crop_bgr.shape[1]
        if total == 0:
            return None
        orange_mask = ((H >= 5) & (H <= 25)) & (S > 80) & (V > 80)
        orange_pct = float(orange_mask.sum()) / float(total)
        if orange_pct >= self._orange_loli_min:
            return 'loli'
        if orange_pct <= self._orange_pipi_max:
            return 'pipi'
        return None


def main() -> None:
    rclpy.init()
    node = PetRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
