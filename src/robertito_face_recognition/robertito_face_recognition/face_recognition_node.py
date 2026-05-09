from __future__ import annotations

import random
import threading
import time
from typing import List, Optional, Tuple

import cv2
import face_recognition
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from robertito_face_recognition.face_db import FaceDatabase
from robertito_face_recognition.srv import EnrollPerson


GREETINGS = [
    "Hola {name}, ¿cómo estás?",
    "Buenas {name}, ¿todo bien?",
    "Hey {name}! ¿Cómo va?",
    "Encantado de verte, {name}.",
    "{name}, un gusto verte de nuevo.",
    "¿Qué tal, {name}? Espero que estés genial.",
    "Hola {name}, ¿qué andás haciendo?",
    "¡Qué alegría verte, {name}!",
    "Saludos, {name}, ¿cómo va ese día?",
    "{name}, ¡buenas! ¿Cómo te trata el día?",
    "¿Cómo pinta el día, {name}?",
    "{name}, siempre es un placer verte."
]


class FaceRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__("face_recognition_node")
        image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        db_path = self.declare_parameter(
            "db_path", "~/.robertito/face_db/face_db.json"
        ).value
        threshold = self.declare_parameter("threshold", 0.6).value
        greet_cooldown = self.declare_parameter("greet_cooldown_sec", 3600).value
        process_fps = self.declare_parameter("process_fps", 5.0).value
        detection_model = self.declare_parameter("detection_model", "hog").value

        self._recognition_threshold = float(threshold)
        self._greet_cooldown = float(greet_cooldown)
        self._process_period = 1.0 / max(0.1, float(process_fps))
        self._detection_model = detection_model
        self._db = FaceDatabase(db_path)
        self._bridge = CvBridge()
        self._latest_image: Optional[np.ndarray] = None
        self._image_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._running = True
        self._last_seen: dict[str, float] = {}
        self._enroll_lock = threading.Lock()

        self._recognized_pub = self.create_publisher(String, "/face_recognition/recognized", 10)
        self._confidence_pub = self.create_publisher(Float32, "/face_recognition/confidence", 10)
        self._greet_pub = self.create_publisher(String, "/face_recognition/greet", 10)

        self._subscription = self.create_subscription(
            Image, image_topic, self._image_callback, 1
        )
        self._subscription  # prevent unused variable

        self._service = self.create_service(
            EnrollPerson, "/face_recognition/enroll", self._handle_enroll
        )
        self._service  # prevent unused variable

        self._worker_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._worker_thread.start()

    def destroy_node(self) -> None:
        self._running = False
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        super().destroy_node()

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # CvBridgeError
            self.get_logger().warning("Could not convert frame: %s", exc)
            return
        with self._image_lock:
            self._latest_image = frame
        self._frame_event.set()

    def _processing_loop(self) -> None:
        while self._running:
            loop_start = time.monotonic()
            frame = self._get_latest_frame()
            if frame is not None:
                self._process_frame(frame)
            elapsed = time.monotonic() - loop_start
            sleep_time = self._process_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _get_latest_frame(self) -> Optional[np.ndarray]:
        with self._image_lock:
            if self._latest_image is None:
                return None
            return self._latest_image.copy()

    def _process_frame(self, frame: np.ndarray) -> None:
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(
            rgb_frame, model=self._detection_model
        )
        if not face_locations:
            self._publish_unknown()
            return
        face_location = self._select_primary_face(face_locations, frame.shape)
        if face_location is None:
            self._publish_unknown()
            return
        encodings = face_recognition.face_encodings(rgb_frame, [face_location])
        if not encodings:
            self._publish_unknown()
            return
        encoding = encodings[0]
        name, distance = self._db.find_best_match(encoding, self._recognition_threshold)
        if name and distance is not None:
            confidence = max(0.0, 1.0 - min(distance, 1.0))
            self._publish_recognition(name, confidence)
            self._maybe_greet(name)
        else:
            self._publish_unknown()

    def _select_primary_face(
        self, faces: List[Tuple[int, int, int, int]], frame_shape: Tuple[int, int, int]
    ) -> Optional[Tuple[int, int, int, int]]:
        if not faces:
            return None
        best_face = max(faces, key=lambda loc: self._face_area(loc))
        return best_face

    @staticmethod
    def _face_area(face: Tuple[int, int, int, int]) -> int:
        top, right, bottom, left = face
        return max(0, bottom - top) * max(0, right - left)

    def _publish_unknown(self) -> None:
        self._recognized_pub.publish(String(data="unknown"))
        self._confidence_pub.publish(Float32(data=0.0))

    def _publish_recognition(self, name: str, confidence: float) -> None:
        self._recognized_pub.publish(String(data=name))
        self._confidence_pub.publish(Float32(data=confidence))

    def _maybe_greet(self, name: str) -> None:
        now = time.time()
        last = self._last_seen.get(name, 0.0)
        if now - last >= self._greet_cooldown:
            message = random.choice(GREETINGS).format(name=name)
            self._greet_pub.publish(String(data=message))
            self._last_seen[name] = now

    def _handle_enroll(
        self, request: EnrollPerson.Request, response: EnrollPerson.Response
    ) -> EnrollPerson.Response:
        name = request.name.strip()
        samples = request.samples if request.samples > 0 else 10
        if not name:
            response.success = False
            response.message = "El nombre no puede estar vacío"
            return response
        if not self._enroll_lock.acquire(blocking=False):
            response.success = False
            response.message = "Ya hay otro enrolamiento en curso"
            return response
        try:
            encodings = self._collect_encodings(samples)
        finally:
            self._enroll_lock.release()
        if not encodings:
            response.success = False
            response.message = "No se detectaron rostros válidos"
            return response
        average_embedding = np.mean(encodings, axis=0)
        self._db.update_person(name, average_embedding)
        response.success = True
        response.message = f"Registrado {name} ({len(encodings)} muestras)"
        return response

    def _collect_encodings(self, target_samples: int) -> List[np.ndarray]:
        encodings: List[np.ndarray] = []
        timeout = 2.0
        deadline = time.monotonic() + target_samples * 5
        while len(encodings) < target_samples and time.monotonic() < deadline:
            frame = self._wait_for_frame(timeout)
            if frame is None:
                continue
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(
                rgb_frame, model=self._detection_model
            )
            if not face_locations:
                continue
            face_location = self._select_primary_face(face_locations, frame.shape)
            if face_location is None:
                continue
            face_encodings = face_recognition.face_encodings(rgb_frame, [face_location])
            if face_encodings:
                encodings.append(face_encodings[0])
        return encodings

    def _wait_for_frame(self, timeout: float) -> Optional[np.ndarray]:
        if not self._frame_event.wait(timeout):
            return None
        with self._image_lock:
            frame = self._latest_image.copy() if self._latest_image is not None else None
        self._frame_event.clear()
        return frame


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
