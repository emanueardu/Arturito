import os
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class ArturitoFaceDetector(Node):
    """Capture frames from a USB camera and publish face detections."""

    def __init__(self) -> None:
        super().__init__('arturito_face_detector')

        qos = QoSProfile(depth=10)
        self._camera_device = int(self.declare_parameter('camera_device', 0).value)
        self._frame_width = int(self.declare_parameter('frame_width', 640).value)
        self._frame_height = int(self.declare_parameter('frame_height', 480).value)
        self._fps = float(self.declare_parameter('frame_rate', 15.0).value)
        self._frame_id = str(self.declare_parameter('frame_id', 'arturito_camera').value)
        self._cascade_path = str(
            self.declare_parameter('cascade_path', '').value
        )
        if not self._cascade_path:
            self._cascade_path = self._find_default_cascade(
                'haarcascade_frontalface_default.xml'
            )
        self._scale_factor = float(self.declare_parameter('scale_factor', 1.1).value)
        self._min_neighbors = int(self.declare_parameter('min_neighbors', 5).value)
        self._min_size = int(self.declare_parameter('min_size_px', 30).value)

        # Pre-procesado para baja iluminación.
        # CLAHE (Contrast Limited Adaptive Histogram Equalization) ecualiza
        # contraste por tiles, levanta caras en zonas oscuras sin saturar
        # las claras. Es gratis (~0.5ms en 640x480 en Pi 5) y mejora mucho
        # la detección con luz pobre.
        self._enable_clahe = bool(self.declare_parameter('enable_clahe', True).value)
        self._clahe_clip_limit = float(
            self.declare_parameter('clahe_clip_limit', 2.5).value
        )
        self._clahe_tile_grid = int(
            self.declare_parameter('clahe_tile_grid', 8).value
        )
        if self._enable_clahe and self._clahe_tile_grid > 0:
            self._clahe = cv2.createCLAHE(
                clipLimit=self._clahe_clip_limit,
                tileGridSize=(self._clahe_tile_grid, self._clahe_tile_grid),
            )
        else:
            self._clahe = None
        # Brillo y umbral para boost de gamma en frames oscuros (refuerzo).
        self._dark_gamma_threshold = float(
            self.declare_parameter('dark_gamma_threshold', 60.0).value
        )
        self._dark_gamma_value = float(
            self.declare_parameter('dark_gamma_value', 1.6).value
        )

        # V4L2 boost: los rangos varían por cámara. -1 = no tocar (preserva
        # el default del driver/usuario). Para FaceCam 1000X probá:
        #   auto_exposure=3, brightness=128, gain=200, contrast=128
        # Si se sobre-expone (lavado), bajá brillo y gain.
        self._auto_exposure_v4l = int(
            self.declare_parameter('cam_auto_exposure', 3).value
        )
        self._brightness_v4l = int(
            self.declare_parameter('cam_brightness', 128).value
        )
        self._gain_v4l = int(self.declare_parameter('cam_gain', 200).value)
        self._contrast_v4l = int(self.declare_parameter('cam_contrast', 128).value)
        self._publish_annotated = bool(
            self.declare_parameter('publish_annotated_image', True).value
        )
        self._annotated_topic = str(
            self.declare_parameter('annotated_topic', 'arturito/camera/faces/image').value
        )
        self._detection_topic = str(
            self.declare_parameter('detection_topic', 'arturito/camera/faces').value
        )
        self._publish_brightness = bool(
            self.declare_parameter('publish_brightness', True).value
        )
        self._brightness_topic = str(
            self.declare_parameter('brightness_topic', 'arturito/camera/brightness').value
        )

        self._wake_topic = str(
            self.declare_parameter('wake_topic', '/wake_word/detected').value
        )
        self._active = bool(self.declare_parameter('start_active', False).value)

        self._cap_lock = threading.Lock()
        self._cap: Optional[cv2.VideoCapture] = None

        self._cascade = cv2.CascadeClassifier(self._cascade_path)
        if self._cascade.empty():
            raise RuntimeError(f'Failed to load cascade file: {self._cascade_path}')

        self._bridge = CvBridge()
        self._pub_detections = self.create_publisher(Detection2DArray, self._detection_topic, qos)
        self._pub_face_flag = self.create_publisher(Bool, 'arturito/face_detected', qos)
        self._pub_brightness = (
            self.create_publisher(Float32, self._brightness_topic, qos)
            if self._publish_brightness
            else None
        )
        self._pub_image = (
            self.create_publisher(Image, self._annotated_topic, qos)
            if self._publish_annotated
            else None
        )

        self.create_subscription(Bool, self._wake_topic, self._on_wake_signal, qos)

        timer_period = max(1.0 / max(self._fps, 1.0), 0.02)
        self._timer = self.create_timer(timer_period, self._process_frame)
        self.get_logger().info(
            f'Arturito face detector using camera {self._camera_device} at {self._fps:.1f} FPS'
        )
        if self._active:
            self._open_camera()
            self.get_logger().info('Detección de caras iniciada en modo activo.')
        else:
            self.get_logger().info('Detección de caras en modo inactivo hasta wake word.')

    def _open_camera(self) -> None:
        if not self._active:
            return
        with self._cap_lock:
            if self._cap:
                return
            cap = cv2.VideoCapture(self._camera_device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._frame_width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._frame_height))
            cap.set(cv2.CAP_PROP_FPS, float(self._fps))
            # Boost para baja iluminación: dejamos auto-exposure activo (3 en
            # V4L2) y subimos brillo/gain. Cada webcam acepta rangos distintos;
            # los SET pueden silenciosamente fallar en valores fuera de rango,
            # por eso ignoramos el resultado y solo logueamos.
            try:
                if self._auto_exposure_v4l > 0:
                    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, float(self._auto_exposure_v4l))
                if self._brightness_v4l >= 0:
                    cap.set(cv2.CAP_PROP_BRIGHTNESS, float(self._brightness_v4l))
                if self._gain_v4l >= 0:
                    cap.set(cv2.CAP_PROP_GAIN, float(self._gain_v4l))
                if self._contrast_v4l >= 0:
                    cap.set(cv2.CAP_PROP_CONTRAST, float(self._contrast_v4l))
                self.get_logger().info(
                    f'Camera tuned: auto_exp={self._auto_exposure_v4l} '
                    f'brightness={self._brightness_v4l} '
                    f'gain={self._gain_v4l} contrast={self._contrast_v4l}'
                )
            except Exception as exc:
                self.get_logger().warn(f'Camera tuning falló: {exc}')
            if not cap.isOpened():
                self.get_logger().error(
                    f'Unable to open camera device {self._camera_device}'
                )
            self._cap = cap

    def _find_default_cascade(self, filename: str) -> str:
        candidates = []
        data_attr = getattr(cv2, 'data', None)
        cascade_root = getattr(data_attr, 'haarcascades', None)
        if cascade_root:
            candidates.append(cascade_root)
        candidates.extend(
            [
                '/usr/share/opencv4/haarcascades/',
                '/usr/share/opencv/haarcascades/',
            ]
        )
        for base in candidates:
            path = os.path.join(base, filename)
            if os.path.exists(path):
                return path
        self.get_logger().warn(
            'Could not locate OpenCV cascade automatically; '
            'set cascade_path parameter if detection fails.'
        )
        return filename

    def _release_camera(self) -> None:
        with self._cap_lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None

    def _set_active(self, active: bool) -> None:
        if self._active == active:
            return
        self._active = active
        if active:
            self.get_logger().info('Activando captura de rostros.')
            self._open_camera()
        else:
            self.get_logger().info('Deteniendo captura de rostros.')
            self._release_camera()

    def _on_wake_signal(self, msg: Bool) -> None:
        if msg.data:
            self._set_active(True)

    def _process_frame(self) -> None:
        if not self._active:
            return
        with self._cap_lock:
            cap = self._cap
        if cap is None or not cap.isOpened():
            self._open_camera()
            return

        ret, frame = cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Camera frame grab failed', throttle_duration_sec=10.0)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._pub_brightness is not None:
            mean_val = float(cv2.mean(gray)[0])
            self._pub_brightness.publish(Float32(data=mean_val))

        # Pre-procesado para detección en baja iluminación:
        #  1. Si la imagen está oscura (mean < umbral), aplica gamma < 1
        #     para levantar tonos medios.
        #  2. CLAHE ecualiza el contraste local, ayuda con caras en sombra.
        gray_for_detect = gray
        try:
            mean_now = float(cv2.mean(gray)[0])
            if mean_now < self._dark_gamma_threshold and self._dark_gamma_value > 1.0:
                inv_gamma = 1.0 / self._dark_gamma_value
                table = (
                    ((np.arange(256) / 255.0) ** inv_gamma) * 255.0
                ).astype('uint8')
                gray_for_detect = cv2.LUT(gray_for_detect, table)
            if self._clahe is not None:
                gray_for_detect = self._clahe.apply(gray_for_detect)
        except Exception:
            gray_for_detect = gray

        detections = self._detect_faces(gray_for_detect)
        self._publish_results(detections, frame)

    def _detect_faces(self, gray) -> list[tuple[int, int, int, int]]:
        faces = self._cascade.detectMultiScale(
            gray,
            scaleFactor=self._scale_factor,
            minNeighbors=self._min_neighbors,
            minSize=(self._min_size, self._min_size),
        )
        return list(faces)

    def _publish_results(self, faces, frame) -> None:
        stamp = self.get_clock().now().to_msg()

        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        for (x, y, w, h) in faces:
            detection = Detection2D()
            detection.header.stamp = stamp
            detection.header.frame_id = self._frame_id
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)
            detection.bbox.center.position.x = float(x + w / 2.0)
            detection.bbox.center.position.y = float(y + h / 2.0)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'face'
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)
            msg.detections.append(detection)

        self._pub_detections.publish(msg)
        self._pub_face_flag.publish(Bool(data=bool(faces)))

        if self._pub_image:
            annotated = frame.copy()
            for (x, y, w, h) in faces:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            image_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self._frame_id
            self._pub_image.publish(image_msg)

    def destroy_node(self) -> bool:
        self.get_logger().info('Shutting down Arturito face detector')
        if hasattr(self, '_timer') and self._timer is not None:
            self._timer.cancel()
        self._release_camera()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArturitoFaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
