# Robertito Face Recognition

Este paquete expone un nodo ROS 2 que detecta rostros, los identifica por nombre usando embeddings preexistentes y lanza saludos cuando toca.

## Dependencias

1. ROS 2 e infraestructura habitual (`rclpy`, `cv_bridge`, `sensor_msgs`, `std_msgs`).
2. Sistema:
   ```sh
   sudo apt update && sudo apt install -y python3-opencv libopenblas-dev libatlas-base-dev
   ```
3. Paquetes Python adicionales (preferentemente dentro del entorno ROS 2):
   ```sh
   pip install face_recognition numpy opencv-python
   ```
4. Para entornos embebidos (Raspberry), se recomienda compilar `dlib` con soporte a NEON y usar `face_recognition` con `HOG`.

## Construcción

Desde la raíz del workspace:

```sh
colcon build --packages-select robertito_face_recognition
```

## Ejecución

```sh
ros2 launch robertito_face_recognition face_recognition.launch.py
```

Parámetros configurables (con sus valores por defecto):

- `image_topic` (`/camera/image_raw`): tópico de cámara.
- `db_path` (`~/.robertito/face_db/face_db.json`): archivo JSON de embeddings.
- `threshold` (`0.6`): distancia Euclídea máxima para considerar coincidencia.
- `greet_cooldown_sec` (`3600`): segundos mínimos entre saludos.
- `process_fps` (`5.0`): velocidad máxima de procesamiento.
- `detection_model` (`hog`): modelo de detección (`hog` o `cnn`).

También publica

- `/face_recognition/recognized`: nombre detectado o `unknown`.
- `/face_recognition/confidence`: score calculado como `max(0, 1.0 - distance)`.
- `/face_recognition/greet`: texto a pronunciar cuando el cooldown lo permite.

## Enrolamiento de personas

1. Publicar la imagen de la persona frente a la cámara.
2. Ejecutar el servicio:

```sh
ros2 service call /face_recognition/enroll robertito_face_recognition/srv/EnrollPerson "name: 'emanuel' samples: 10"
```

El servicio captura `samples` frames y promedia los embeddings válidos antes de persistirlos.

## Verificación y pruebas

- Mirar tópicos:
  ```sh
  ros2 topic echo /face_recognition/recognized
  ros2 topic echo /face_recognition/greet
  rqt_image_view /camera/image_raw
  ```
- Asegúrese de que `~/.robertito/face_db/face_db.json` contenga entradas. El nodo recarga la base al iniciar, por lo que los registros persisten.

## Tests mínimos

```sh
pytest src/robertito_face_recognition/tests/test_face_db.py
```

## Ajustes de rendimiento y umbral

1. `process_fps`: bajar si la CPU está cargada.
2. `threshold`: valores entre `0.4` y `0.7` son comunes; bajar para exigir mayor precisión.
3. `greet_cooldown_sec`: ajustar si quiere saludar más seguido.

El nodo sólo saluda si ha pasado más del cooldown desde el último saludo; el tópico `/face_recognition/greet` puede conectarse al nodo de voz existente del repo.
