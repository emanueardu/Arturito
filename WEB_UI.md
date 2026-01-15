# Interfaz Web en Español

Este documento describe cómo usar el paquete `robot_web_bridge` y la SPA React para monitorear y controlar el robot sin romper los nodos existentes.

## Panorama del workspace

- **ROS 2**: `jazzy`
- **Paquetes relevantes**:
  - `robot_bringup`: lanza `v4l2_camera` (`/camera/image_raw`) y motores mediante `/cmd_vel`.
  - `robertito`: nodos de ojos, TTS, detección de rostros (`arturito/camera/faces/image`), wake word, etc. Publica `arturito/cmd_vel` para seguir personas.
  - `esp32_serial_bridge`: consume `/cmd_vel` para llegar al ESP32.
- **Tópicos de imagen** detectados en el código:
  - `/camera/image_raw` (v4l2_camera)
  - `arturito/camera/faces/image` (dibuja detecciones)
- **Tópicos de `Twist`**:
  - `/cmd_vel` (control directo del robot vía ESP32)
  - `arturito/cmd_vel` (nodos de comportamiento con nombre interno)
- **Actuadores y servicios**:
  - `/head/tilt` (std_msgs/Float32) controla la inclinación del mástil/cámara.
  - `/robot_web/vacuum_enable` y `/robot_web/brush_enable` son tópicos Bool que el bridge traduce a los comandos UART (`VAC1/0`, `BRUSH1/0`) más un espejo en `/vacuum/enabled` y `/brush/enabled`.
  - `arturito/vacuum_state` y `arturito/brush_state` publican el estado real para que la UI muestre feedback incluso si alguien activa desde consola.
  - `zone_waypoint_node` publica `/robot_web/zone_catalog` (JSON con polígonos, centroides y metadatos del mapa NAV2) y `/robot_web/zone_state` (estado actual / zona atendida) y escucha en `/zone_goal` para comenzar la limpieza de un ambiente específico.
- **Telemetría cruda (`status_raw`)**:
  - `/arturito/status_raw` es un `std_msgs/String` con JSON tipo `{"at":8.26,"yaw":-93.4,"dist":6,"bL":0,"bR":0,...}` (en algunos firmwares aparece `vbat`; la UI soporta ambos). Desde ahí se alimentan la tarjeta de batería, la distancia frontal, el yaw y los bumpers en la UI.
- **Seguimiento de personas**:
  - `/tracker_control` acepta strings `seguir` / `dejar de seguir` para activar o detener el tracker.
  - `/tracker_status` (Bool) publica el estado real para reflejarlo en el botón de la UI.
- **Navegación autónoma (wander_avoid)**:
  - `/robot_web/wander_enable` (Bool) enciende o apaga el nodo `wander_avoid`.
  - `/robot_web/wander_status` (Bool) refleja el estado real en el botón.
- **Audio**: no existe tópico ROS; por eso el servidor de audio abre ALSA en modo escucha (`sounddevice`). Si otro nodo ya usa el micro, desactiva el audio con el parámetro `enable_audio_server:=false`.
- **Video de respaldo**: cuando no hay cámaras publicando, `robot_web_bridge` genera un stream de prueba en `/robot_web/placeholder/image` para que la pestaña “Cámara” no quede en negro.

> Nota: `ros2 topic list` y `ros2 node list` no devolvieron datos en este entorno porque no había un grafo activo. Los defaults anteriores se dedujeron de los paquetes y se pueden cambiar por parámetros o variables de entorno.

## 1. Bridge ROS ↔ Web

### Compilación

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Ejecución rápida

`scripts/start_arturito.sh` ahora llama a `robot_web_bridge/robot_auto.launch.py`, por lo que basta con ejecutar ese script (o habilitar `arturito.service`) para que se levanten el stack original y el bridge web. Si necesitás lanzar solo el bridge, podés hacerlo manualmente:

```bash
scripts/ejecutar_web_bridge.sh
```

> Nota: `scripts/ejecutar_web_bridge.sh` ahora respeta un lock compartido con `start_arturito.sh`. Si el stack completo está activo, el script no levanta un segundo rosbridge. Si el bridge está activo y arrancás `start_arturito.sh`, el stack completo toma prioridad y detiene el bridge para evitar duplicados.

Variables opcionales:

| Variable / parámetro | Default | Descripción |
| --- | --- | --- |
| `ROSBRIDGE_PORT` / `rosbridge_port` | `9090` | Puerto WebSocket de rosbridge. |
| `VIDEO_PORT` / `video_port` | `8080` | HTTP MJPEG. |
| `VIDEO_TOPIC` / `video_topic` | auto | Se detecta `/camera/image_raw` y luego `arturito/camera/faces/image`. |
| `AUDIO_PORT` / `audio_port` | `8081` | HTTP WAV PCM. |
| `AUDIO_TOPIC` / `audio_topic` | vacío | Si existe `audio_common_msgs/AudioData`, se usa; sino ALSA. |
| `AUDIO_DEVICE` / `audio_device` | `default` | Dispositivo ALSA (usa `arecord -L` para listar). |
| `enable_audio_server` | `true` | Activa el servidor HTTP. En `scripts/start_arturito.sh` está en `false` por defecto para no bloquear el wake word. |
| `enable_alsa_fallback` | `false` | Cuando es `true`, el servidor abre el micrófono local (ALSA) si no hay `audio_common_msgs/AudioData`. Activa solo si tu wake word no necesita el mismo micro. |
| `UART_START_ENABLED` / `uart_start_enabled` | `true` | Si es `true`, `arturito/cmd_vel` llega al puente UART sin esperar wake word. |

### Endpoints

- Rosbridge WS: `ws://<robot>:9090`
- Video MJPEG: `http://<robot>:8080/mjpeg` (`?topic=/camera/image_raw` o `?topic=/robot_web/placeholder/image`)
- Audio WAV: `http://<robot>:8081/audio`
- Health status: `.../status` en ambos puertos
- Control auxiliar: el nodo `robot_web_control_bridge` escucha en `/robot_web/tilt_deg`, `/robot_web/vacuum_enable` y `/robot_web/brush_enable` para traducirlos a `/movement_cmds` y a los servicios `arturito/set_*`.

### Parámetros ROS

Puedes cambiar los tópicos con `ros2 param set`:

```bash
ros2 param set /robot_web_video_bridge video_topic /arturito/camera/faces/image
ros2 param set /robot_web_audio_bridge enable_audio_server false
```

## 2. Aplicación web (React + Vite)

### Preparación

```bash
cd ~/ros2_ws/web/robot-ui
cp .env.example .env
# Editar la IP del robot en el archivo .env
```

Variables principales en `.env` (todas en español):

```
VITE_ROSBRIDGE_WS_URL=ws://robotito.local:9090
VITE_VIDEO_URL=http://robotito.local:8080/mjpeg
VITE_AUDIO_URL=http://robotito.local:8081/audio
VITE_CMD_VEL_TOPIC=/cmd_vel
VITE_TILT_CMD_TOPIC=/robot_web/tilt_deg
VITE_VACUUM_CMD_TOPIC=/robot_web/vacuum_enable
VITE_BRUSH_CMD_TOPIC=/robot_web/brush_enable
VITE_VACUUM_STATE_TOPIC=/arturito/vacuum_state
VITE_BRUSH_STATE_TOPIC=/arturito/brush_state
VITE_MAP_IMAGE_URL=/maps/apartamento_map.png
VITE_ROBOT_POSE_TOPIC=/amcl_pose
VITE_ZONE_CATALOG_TOPIC=/robot_web/zone_catalog
VITE_ZONE_STATE_TOPIC=/robot_web/zone_state
VITE_ZONE_GOAL_TOPIC=/zone_goal
```

Variables opcionales para ajustar límites y feedback:

| Variable | Default | Uso |
| --- | --- | --- |
| `VITE_TILT_MIN` / `VITE_TILT_MAX` | `-20` / `45` | Rango permitido para el slider de inclinación. |
| `VITE_VACUUM_STATE_TOPIC` | `/arturito/vacuum_state` | Topic Bool para reflejar el estado real. |
| `VITE_BRUSH_STATE_TOPIC` | `/arturito/brush_state` | Topic Bool para escobillas. |
| `VITE_MAP_IMAGE_URL` | `/maps/apartamento_map.png` | Imagen estática (PNG) del mapa Nav2 para renderizar la pestaña Mapa. |
| `VITE_ROBOT_POSE_TOPIC` | `/amcl_pose` | Fuente de `geometry_msgs/PoseWithCovarianceStamped` para ubicar el robot sobre el mapa. |
| `VITE_ZONE_CATALOG_TOPIC` | `/robot_web/zone_catalog` | JSON latcheado con polígonos, centroides y metadatos de cada zona. |
| `VITE_ZONE_STATE_TOPIC` | `/robot_web/zone_state` | Estado publicado por `zone_waypoint_node` (disponible, navegando, etc.). |
| `VITE_ZONE_GOAL_TOPIC` | `/zone_goal` | Tópico String al que la UI publica para solicitar limpieza por ambiente. |
| `VITE_COVERAGE_SPACING_M` | `0.35` | Distancia entre las “pasadas” coloreadas que se dibujan dentro de cada zona. Ajustá para representar tu ancho real de escobillas. |
| `VITE_FALLBACK_ZONES_URL` | `/maps/apartamento_zones.json` | Catálogo estático que la UI abre cuando ROS aún no publica `zone_catalog`, garantizando que el mapa siempre tenga polígonos y recorridos. |

### Desarrollo LAN

```bash
scripts/ejecutar_web_ui.sh
# Abre http://<robot>:5173 desde cualquier PC en la red
```

### Build de producción

```bash
cd ~/ros2_ws/web/robot-ui
npm install
npm run build
# El resultado queda en web/robot-ui/dist (sirve para Nginx, etc.)
```

## 3. Contenido de la UI

1. **Panel**: estado de conexión, latencia, IP y URLs de servicios.
2. **Control**: teleoperación en español con joystick táctil, teclado WASD, sliders de velocidad, botón “Parada de emergencia”, control continuo de tilt (publica en `/robot_web/tilt_deg` → `robot_web_control_bridge` lo traduce a comandos `tXX`), toggles para aspiradora y escobillas (publican en `/robot_web/vacuum_enable` y `/robot_web/brush_enable`, y el bridge reenvía el Bool a `/vacuum/enabled` y `/brush/enabled` para que la ESP32 lo procese), vista rápida de cámara y un panel “Sensores inmediatos” que muestra distancia frontal y estado de bumpers directamente desde `/arturito/status_raw`.
3. **Mapa**: muestra el mapa estático de Nav2 (PNG en `web/robot-ui/public/maps/apartamento_map.png` por defecto), superpone la posición del robot (desde `/amcl_pose`), el dock/base (con orientación exportada desde `apartamento_waypoints.yaml`) y los polígonos de `zone_waypoint_node`. Si ROS aún no publicó el catálogo, la SPA carga automáticamente `web/robot-ui/public/maps/apartamento_zones.json` para mostrar los ambientes. Cada ambiente se rellena con trazos serpentinas coloreados (espaciado configurable con `VITE_COVERAGE_SPACING_M`) que representan el recorrido previsto para cubrir el área, además de una línea de ida/vuelta desde el dock. Basta con tocar un ambiente en la lista lateral para resaltar su trayectoria y, si se desea, lanzar la limpieza pulsando el botón correspondiente (publica en `/zone_goal`).
4. **Cámara**: reproductor MJPEG a pantalla completa con selector de tópico via query `?topic=` para no tocar /dev/video0.
5. **Audio**: reproducción WAV en vivo con silencio + recarga. Talkback documentado como pendiente (evita abrir el micro en escritura).
6. **Diagnóstico**: la tarjeta de batería ahora lee `vbat/ibat` desde `/arturito/status_raw` (y usa `arturito/battery` solo como respaldo para el porcentaje), acompañado por los módulos ya existentes (`arturito/ultrasonic`, bumpers, etc.).
7. **Inspector**: se suscribe a cualquier tópico usando roslibjs y muestra JSON + Hz.

Todos los textos, labels y mensajes están en español; los nombres técnicos (`/cmd_vel`, `rosbridge_server`) se mantienen en inglés por compatibilidad.

## 4. Servicio systemd

`arturito.service` (incluido en `systemd/`) está pensado para systemd de usuario. Copiá el archivo a `~/.config/systemd/user/`, recargá y habilitá:

```bash
mkdir -p ~/.config/systemd/user
cp systemd/arturito.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now arturito.service
```

Con `loginctl enable-linger robot` (ya aplicado en esta máquina) queda activo tras cada boot. Si necesitás únicamente el bridge, instalá `systemd/robot-web.service` y controlalo con `systemctl --user start|stop robot-web.service`.
Ese servicio detecta si `robot_auto.launch.py` ya está corriendo (por `arturito.service`) y, en tal caso, queda en espera para evitar nodos duplicados; si el stack grande no está presente, lanza rosbridge/video/audio automáticamente.

## 5. Troubleshooting

| Problema | Posibles causas / solución |
| --- | --- |
| **No aparece video** | El tópico seleccionado no existe. Visita `http://<robot>:8080/status` para ver la lista. Si ningún nodo publica imágenes, lanza `v4l2_camera` o el detector de caras (`arturito_face_detector`). Mientras tanto podés usar `/robot_web/placeholder/image`. |
| **Audio se corta o no levanta** | Otro proceso usa el micro ALSA (wake word, reconocimiento). Por defecto `ENABLE_AUDIO_SERVER=false` y `enable_alsa_fallback=false` para evitar conflictos. Publica `audio_common_msgs/AudioData` o habilita manualmente el fallback sabiendo que bloqueará el wake word. |
| **Teleop no mueve el robot** | Confirmá que `/arturito_uart_bridge` esté habilitado (`ros2 param get /arturito_uart_bridge start_enabled`). Si responde `false`, seteá `UART_START_ENABLED=true` (o `ros2 param set ... true`). Cambiá `VITE_CMD_VEL_TOPIC` si tu stack usa `arturito/cmd_vel`. |
| **Aspiradora/escobillas no responden** | Asegurate de que `robot_web_control_bridge` esté activo y que los tópicos `/vacuum/enabled` y `/brush/enabled` existan (`ros2 topic list | grep enabled`). Si usás otros nombres, ajusta `VACUUM_PASSTHROUGH_TOPIC` / `BRUSH_PASSTHROUGH_TOPIC` en el launch y/o las variables `VITE_*` (tanto CMD como COMMAND). Los botones muestran el estado real leyendo `arturito/vacuum_state` y `arturito/brush_state`. |
| **rosbridge no arranca** | Instala `rosbridge_server` (`sudo apt install ros-jazzy-rosbridge-server`). Verifica que ningún otro proceso use el puerto 9090. |
| **Navegador muestra contenido mixto inseguro** | Si sirves la SPA por HTTPS, también necesitas rosbridge/video/audio con TLS. Caso contrario, usa HTTP o un proxy local. |

## 6. Próximos pasos

- Implementar talkback seguro (WebRTC o duplex HTTP) respetando prioridades del wake word.
- Añadir autenticación básica o token en rosbridge cuando se abra el robot a redes compartidas.
- Exportar métricas adicionales (carga de CPU, temperatura, etc.) a la pestaña Diagnóstico.
