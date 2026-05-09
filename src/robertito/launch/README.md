# Launches de Robertito

## Launch principal (corre con systemd)

`robertito_launch.py` — incluido por `robot_auto.launch.py` desde el
paquete robot_web_bridge. Levanta el stack mínimo headless:

- eyes_node, voice_synth_node, wake_word_listener
- api_chat_node, face_detector_node, person_tracker_node (idle)
- presence_orchestrator, behavior_state_node, clean_quick (idle)
- arturito_uart_bridge

## Launch opcional: tools

`tools.launch.py` — para sesiones de calibración manual. NO se levanta
en boot. Usar:

    ros2 launch robertito tools.launch.py enable_docking_calibration:=true
    ros2 launch robertito tools.launch.py enable_wifi_localization:=true

## Launch opcional: web UI

`web_ui.launch.py` — para sesiones con frontend web. NO se levanta en boot
(default desde mayo 2026 para ahorrar RAM en el Pi 5).

    ros2 launch robertito web_ui.launch.py

O bien, levantando todo desde el launch raíz:

    ros2 launch robot_web_bridge robot_auto.launch.py enable_web_ui:=true

## Cambios en este PR (PR1 — 2026-05)

- Eliminado entry_point zombie `wake_word_node` de `setup.py`.
- Movido `docking_calibration_node` y `wifi_localization_node` a
  `tools.launch.py` (opt-in).
- Movido el web stack a `web_ui.launch.py` y agregado flag
  `enable_web_ui` (default `false`) en `robot_auto.launch.py`.
- Por default el robot ahora arranca con ~10 nodos en vez de 17.
