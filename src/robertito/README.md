# Robertito Navigation Stack

Este paquete ahora incluye un stack básico de navegación con Nav2 Jazzy, multiplexado de velocidades y corrección global sin encoders.

## Arquitectura de movimiento

- **Multiplexor `/cmd_vel`:** `robertito_twist_mux` prioriza `/cmd_vel/safety_stop`, luego `/cmd_vel/nav2`, `/cmd_vel/person_track`, `/cmd_vel/wander` y `/cmd_vel/teleop`. Solo este nodo publica `/cmd_vel`, de modo que `arturito_uart_bridge` recibe siempre el comando final.
- **Nodos de comportamiento:** `teleop_node`, `wander_avoid_node` y `person_tracker_node` publican en `/cmd_vel/teleop`, `/cmd_vel/wander` y `/cmd_vel/person_track` respectivamente; no escriben en `/cmd_vel` directo.
- **Nav2 → `/cmd_vel/nav2`:** El bridge `robertito_cmd_vel_bridge` remapea la salida `cmd_vel_nav` de Nav2 a `/cmd_vel/nav2`, que el multiplexor usa como entrada de alta prioridad.

## Localización y odometría

- **Odometría open loop (`robertito_open_loop_odom`):** integra `cmd_vel` + MPU6050, limita aceleraciones y calibra el bias del giroscopio en los primeros 5 segundos (archivo `~/.ros/robertito_gyro_bias.json`). Publica `/odom` y el TF `odom → base_link`.
- **Corrección global (`robertito_map_odom_broadcaster`):** usa las estimaciones WiFi (`/robot_web/wifi_pose`) y los registros de docking (`/robot_web/docking_info`) para publicar `map → odom` con filtrado, detección de saltos y un topic `/localization/status` de diagnóstico.
- **Seguridad + obstáculos (`robertito_obstacles`):** frena el robot si el sensor ultrasónico detecta < 0.25 m o si algún bumper se dispara, publica `/cmd_vel/safety_stop` a 15 Hz y genera `/obstacle_points` (PointCloud2) con el sonar frontal y los bumpers laterales.

## Nav2

- El lanzamiento `ros2 launch robertito robertito_nav2.launch.py map:=<ruta_al_map.yaml>` levanta:
  1. `arturito_uart_bridge`, wifi/docking, twist mux, odometría, corrección de mapa y nodo de obstáculos.
  2. Transformadores estáticos `base_link → imu_link|camera_link|ultrasonic_link`.
  3. `nav2_bringup` con `config/robertito_nav2_params.yaml` (controlador Regulated Pure Pursuit, costmaps 3 m × 3 m a 0.05 m, robot radius 0.17, layer de obstáculos basado en `/obstacle_points`).
  4. En la interfaz web del robot hay ahora una tarjeta “Navegación Nav2” que permite elegir una zona detectada (o ingresar coordenadas manuales) y enviar un goal directo al action server `/navigate_to_pose`. Así podés manejar rutas desde la PC o el celular sin usar RViz.

## Hitos de verificación

1. **Ver estado de TF**
   - `ros2 run tf2_tools view_frames`
   - `ros2 run tf2_ros tf2_echo map base_link`
   - `ros2 run tf2_ros tf2_echo odom base_link`
2. **Comprobar odometría:** `ros2 topic echo /odom --once`
3. **Ver mux:** `ros2 topic echo /cmd_vel` (debería reflejar la fuente activa y congelarse en `0` si safety lo bloquea).
4. **Iniciar Nav2 conservador:** `ros2 launch robertito robertito_nav2.launch.py map:=<ruta_al_map.yaml>`

## Notas adicionales

- Los parámetros están en `src/robertito/config/` (twist_mux, open_loop_odom, map_odom, obstacles, Nav2, wifi, docking, uart).
- Para ajustar el esquema de prioridades o velocidades basta editar `config/twist_mux.yaml`.
- Si el MPU6050 está quieto al arrancar, el nodo de odometría guarda el bias en `~/.ros/robertito_gyro_bias.json`; borrarlo fuerza recalibración.

## Asistente de voz Robertito

- **Requisitos**: `sudo apt install libasound2-dev portaudio19-dev ffmpeg` y `pip install -U requests azure-cognitiveservices-speech webrtcvad sounddevice pyaudio numpy whisper` (agrega `faster-whisper` si querés fallback adicional).
- **Variables de entorno**: exporta `OPENAI_API_KEY`, `AZURE_SPEECH_KEY`, `AZURE_SPEECH_REGION=eastus`, `AZURE_TTS_VOICE=es-AR-TomasNeural`. Opcionales: `STT_MODE=azure|whisper`, `VOICE_MODE=jarvis|normal`, `AUDIO_BACKEND=sounddevice`, `AUDIO_DEVICE=default`, `BREAK_MS=200`, `JARVIS_RATE=98%`, `JARVIS_PITCH=-1st`, `NORMAL_RATE=90%`, `NORMAL_PITCH=-2st`. Si usas `~/.config/robertito.env` podés cargarlo con `source ~/.config/robertito.env` o agregarlo a `~/.bashrc`.
- **Corer el asistente**: `ros2 launch robertito robertito_voice.launch.py audio_backend:=sounddevice audio_device:=default voice_mode:=normal stt_mode:=azure vad_silence_ms:=800 vad_aggressiveness:=2 sensitivity:=0.6 startup_message:='Buenos días' tts_rate:=0.90 tts_pitch:='-2st'`. Ajustá parámetros si necesitás más sensibilidad o un backend distinto; `voice_mode:=jarvis` usa el tono más energético.
- **Probar barge-in**: decí “robertito” para activar la escucha, luego hablá mientras el asistente responde; debería cortar el TTS y volver a escuchar de inmediato mostrando logs `wake->feedback_ms` y `wake->first_audio_ms`.
- **Troubleshooting ALSA**: si el dispositivo reporta “busy”, detené nodos anteriores (`ros2 node list | xargs -n1 ros2 lifecycle set ...` no es necesario), o ejecutá `sudo fuser -v /dev/snd/*` y cerrá procesos en conflicto; también podés cambiar a `audio_backend:=pyaudio` o `audio_device:=plughw:0` para saltar bloqueos.
