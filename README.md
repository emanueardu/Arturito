# Robertito ROS 2 Workspace

Distributed ROS 2 bringup for the Robertito service robot, including the motion/vision stack, the ROS↔Web bridge, and the React web console.

## Key features
- Coordinated ROS 2 Jazzy launch for `robertito`, `robot_bringup`, `esp32_serial_bridge`, vision helpers, and the web bridge to expose video/audio/control topics.
- Dedicated voice mode with `voice_synth_node`, Azure TTS hooks (`AZURE_SPEECH_*`), and local Piper models for fallback speech output.
- `robot_web_bridge` that runs `rosbridge_suite`, video/audio HTTP proxies, and a translator node so the UI controls (`/robot_web/*`) map to firmware topics.
- Web UI built with React 18 + Vite 5 that exposes camera streaming, teleop, audio, diagnostics, motion presets, and NAV2 overlays via rosbridge.
- Helper scripts for start/stop (`scripts/start_arturito.sh`, `scripts/stop_arturito.sh`), singleton launch, web-only startup, and duplicate-node diagnostics.
- Templates for `systemd` user services along with `docs/duplicate_nodes.md` and `WEB_UI.md` for additional troubleshooting.

## Tech Stack
- ROS 2 Jazzy (Cyclone DDS) on Python 3.12 with `colcon` workspaces.
- Python packages: `robertito`, `robot_web_bridge`, `esp32_serial_bridge`, `robot_bringup`, `vision_msgs`, `rosbridge_suite`, plus standard ROS 2 utilities.
- React 18 + TypeScript + Vite 5 web UI (`web/robot-ui`) communicating over `roslib` to rosbridge endpoints.
- Node.js 20+ / npm (matches the existing package-lock) for the front-end build and dev server.
- Piper TTS assets under `piper/` for offline speech playback.

## Architecture / How it works
1. `robertito` and `robot_bringup` spin up ROS 2 nodes for locomotion, sensors, voice, and the ESP32 serial bridge so the base firmware receives `/movement_cmds`, `/vacuum/enabled`, and `/brush/enabled` commands.
2. `robot_web_bridge` exposes `rosbridge_suite` plus HTTP endpoints (`video_port=8080`, `audio_port=8081`) and translates `/robot_web/*` topics for the firmware.
3. The React web client (`web/robot-ui`) connects to rosbridge (`ws://<robot>:9090`) and streams MJPEG video plus audio via the exposed HTTP endpoints, while buttons and sliders publish to control topics.
4. The voice stack publishes to `/assistant/say`, consumes `/wake_word/detected`, and uses Azure or local Piper outputs depending on the environment variables defined in `~/.config/robertito.env`.

```
     +---------------------+     rosbridge     +--------------------+     http
     | ROS 2 nodes &       | <--------------> | robot_web_bridge    | <----------+
     | hardware interface  |                   | (rosbridge, audio,  |            |
     | robertito/vision/   |                   | video, control mux) |            |
     | esp32_serial_bridge)|                   +--------------------+            |
     +---------------------+                       ^        ^                    |
                                                   |        |                    |
                                                   |        |                    |
                                                   |        |                    |
                                                video     audio               |
                                                   |        |                    |
                                                   v        v                    v
                                           +-------------------------+     +-----------------+
                                           | React + Vite UI         |     | Piper TTS        |
                                           | web/robot-ui (roslib)   |     | (piper/ + Azure) |
                                           +-------------------------+     +-----------------+
```

## Project Structure
- `src/`: ROS 2 Python packages (`robertito`, `robot_web_bridge`, `esp32_serial_bridge`, `robot_bringup`, `vision_msgs`, `rosbridge_suite`, `audio_common`).
- `scripts/`: helper launches (`start_arturito.sh`, `stop_arturito.sh`, `ejecutar_web_bridge.sh`, `diagnose_ros_duplicates.sh`).
- `web/robot-ui`: React/Vite TypeScript UI that connects to rosbridge endpoints (see `WEB_UI.md`).
- `systemd/`: sample `systemd` user services (`arturito.service`, `robot-web.service`). Copy to `~/.config/systemd/user/` and enable with `systemctl --user enable --now arturito.service`.
- `docs/`: additional troubleshooting notes (e.g., `duplicate_nodes.md`).
- `piper/` & `models/`: offline speech assets that feed into `voice_synth_node` when Azure credentials are absent.

## Installation
1. **ROS 2 prerequisites** (Jazzy + toolchain)
   ```bash
   sudo apt update && sudo apt install -y build-essential python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip
   sudo rosdep init  # (skip if already initialized)
   rosdep update
   ```
2. **Workspace bootstrap**
   ```bash
   cd /home/robot/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```
3. **Frontend dependencies**
   ```bash
   cd web/robot-ui
   npm install
   ```
4. **Optional: register systemd service**
   ```bash
   mkdir -p ~/.config/systemd/user
   cp systemd/arturito.service ~/.config/systemd/user/
   systemctl --user daemon-reload
   systemctl --user enable --now arturito.service
   ```

## Usage / Run
### Full stack (robot + web bridge)
```bash
source /home/robot/ros2_ws/install/setup.bash
/home/robot/ros2_ws/scripts/start_arturito.sh
```
Use `./scripts/stop_arturito.sh` to gracefully stop everything.

### Web bridge only (debugging)
```bash
source /home/robot/ros2_ws/install/setup.bash
/home/robot/ros2_ws/scripts/ejecutar_web_bridge.sh
```

### Web UI (local dev server)
```bash
cd /home/robot/ros2_ws/web/robot-ui
npm run dev -- --host 0.0.0.0 --port 5173
```
Open `http://<robot-ip>:5173` on your LAN browser and use the control panel, diagnostics, map, camera, and audio tabs.

### Diagnostics
```bash
/home/robot/ros2_ws/scripts/diagnose_ros_duplicates.sh
```
Checks for duplicate ROS 2 nodes/processes on the same machine.

## Configuration
- **`~/.config/robertito.env`**: set runtime variables that `scripts/start_arturito.sh` sources.
  ```ini
  AZURE_SPEECH_KEY=
  AZURE_SPEECH_REGION=
  AZURE_TTS_VOICE=es-ES-AliciaNeural
  MICROPHONE_DEVICE_INDEX=0
  VIDEO_TOPIC=/robot_web/video
  AUDIO_TOPIC=/robot_web/audio
  ENABLE_AUDIO_SERVER=false
  ENABLE_ALSA_FALLBACK=false
  UART_PARAMS=/home/robot/ros2_ws/src/robertito/config/arturito_uart.yaml
  ```
  - `ENABLE_AUDIO_SERVER`/`ENABLE_ALSA_FALLBACK`: keep both `false` when using the voice wake word to avoid capturing the mic twice.
  - `RMW_IMPLEMENTATION` is forced to `rmw_cyclonedds_cpp` in scripts for determinism.
- **`web/robot-ui/.env`**: mirrors the live robot endpoints.
  ```ini
  VITE_ROSBRIDGE_WS_URL=ws://<robot-ip>:9090
  VITE_VIDEO_URL=http://<robot-ip>:8080/mjpeg
  VITE_AUDIO_URL=http://<robot-ip>:8081/audio
  VITE_CMD_VEL_TOPIC=/cmd_vel
  VITE_PANEL_ROBOT_IP=<robot-ip>
  VITE_TILT_CMD_TOPIC=/robot_web/tilt_deg
  VITE_VACUUM_CMD_TOPIC=/robot_web/vacuum_enable
  VITE_BRUSH_CMD_TOPIC=/robot_web/brush_enable
  VITE_TRACKER_CMD_TOPIC=/tracker_control
  ```
  Update the IPs/ports to match your deployment before running `npm run dev` or `npm run build`.
- **Parameter files**: `src/robertito/config/robertito_voice.yaml` and `arturito_uart.yaml` contain defaults for pitch/rate and serial bus values.
- **OpenAI key**: The wake-word assistance uses `OPENAI_API_KEY` by default. Copy `.env.example` to `.env` or add the key to `~/.config/robertito.env`, then `source` the file before launching the stack so the node can read the variable at runtime.

## Demo / Quick test
1. Build and source the workspace (`colcon build --symlink-install && source install/setup.bash`).
2. Populate `~/.config/robertito.env` with at least the Azure keys or let the Piper models run locally.
3. Start the full stack: `scripts/start_arturito.sh`.
4. Launch the frontend: `cd web/robot-ui && npm run dev -- --host 0.0.0.0 --port 5173`.
5. Navigate to `http://<robot-ip>:5173` and exercise the tabs (Panel, Control, Map, Camera, Audio).

## Troubleshooting
- **`colcon build` errors complaining about missing ROS dependencies**: rerun `rosdep install --from-paths src --ignore-src -r -y`; missing packages usually hint that the ROS distro is not fully installed.
- **Launch scripts fail with `ROS 2` variables not found**: ensure you ran `source install/setup.bash` from the workspace before invoking `scripts/*`.
- **Web UI cannot connect / video stream is blank**: verify the `.env` URLs match the running bridge IP/ports and that `rosbridge` is listening on `9090` (check `scripts/ejecutar_web_bridge.sh`).
- **Wake word/audio server conflict**: keep `ENABLE_AUDIO_SERVER=false` when the voice node is running, or accept that enabling audio will pause the wake word.
- **Duplicate ROS nodes keep crashing**: run `scripts/diagnose_ros_duplicates.sh` and stop any stray `ros2` processes or use `scripts/start_singleton.sh` to grab the flock.
- **Systemd service refuses to start**: copy the desired service into `~/.config/systemd/user/`, reload the daemon, and route `/home/robot/.config/robertito.env` to the right credentials.

## Roadmap
- Harden automated tests (unit + integration) for `robot_web_bridge` and the ESP32 serial handler.
- Add CI validation for `npm run build` and `colcon test` before releasing packages.
- Expand docs with deployment checklists for different robot models and additional env var combos.

## License
Licensed under the MIT License. See `LICENSE` for details.

## Contact / Author
Emanuel Suarez – [LinkedIn](https://www.linkedin.com/in/emanuel-suarez)
