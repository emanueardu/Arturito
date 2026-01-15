# Duplicidad de nodos ROS 2: causa raiz y fix

## Resumen

La duplicidad aparecia cuando se lanzaba el bridge web en paralelo al stack completo:

- `robot_web_bridge/web_bridge.launch.py` (via `scripts/ejecutar_web_bridge.sh` o `ros2 launch robot_web_bridge web_bridge.launch.py`)
- `robot_web_bridge/robot_auto.launch.py` (via `scripts/start_arturito.sh` o `systemd/arturito.service`)

Esto crea dos instancias con el mismo nombre para `rosbridge_websocket`, `rosapi` y los nodos `robot_web_*`.

## Rutas de arranque inventariadas

1. Launch files:
   - `src/robot_web_bridge/launch/robot_auto.launch.py`
   - `src/robot_web_bridge/launch/web_bridge.launch.py`
   - `src/robertito/launch/robertito_launch.py`
   - `src/robertito/launch/arturito.launch.py`
   - `src/robot_bringup/launch/bringup.launch.py`
2. Scripts:
   - `scripts/start_arturito.sh` -> `robot_auto.launch.py`
   - `scripts/ejecutar_web_bridge.sh` -> `web_bridge.launch.py`
3. systemd:
   - `systemd/arturito.service` -> `scripts/start_arturito.sh`
   - `systemd/robot-web.service` -> `scripts/ejecutar_web_bridge.sh` (con guard de proceso)

## Reproduccion controlada

1. Con `arturito.service` activo, se ejecuto `scripts/ejecutar_web_bridge.sh`.
2. Resultado:
   - `ros2 node list` mostro warning por nombres duplicados y nodos repetidos:
     - `/rosbridge_websocket` (duplicado)
     - `/rosapi` (duplicado)
     - `/robot_web_control_bridge` (duplicado)
   - `ps -ef` mostro dos procesos `ros2 launch` en paralelo:
     - `ros2 launch robot_web_bridge robot_auto.launch.py`
     - `ros2 launch robot_web_bridge web_bridge.launch.py`

## Causa raiz

El script `scripts/ejecutar_web_bridge.sh` no tenia un guardrail para evitar que el bridge web se inicie cuando el stack completo ya esta activo. Esto permite lanzar `web_bridge.launch.py` en paralelo a `robot_auto.launch.py`, duplicando nodos con el mismo nombre.

## Fix aplicado

1. Se agrega un lock compartido (`/tmp/robot_auto.launch.lock`) via `scripts/start_singleton.sh`.
2. `scripts/ejecutar_web_bridge.sh` ahora utiliza ese lock, por lo que:
   - Si el stack completo esta activo, no inicia un segundo web bridge.
3. `scripts/start_arturito.sh` toma prioridad:
   - Si detecta el bridge web activo sin stack completo, lo detiene y arranca el stack.

## Guardrails y diagnostico

- `scripts/diagnose_ros_duplicates.sh`: captura `ros2 node list`, `ros2 topic list` y procesos asociados.
- `scripts/start_singleton.sh`: helper general para garantizar un unico stack activo.

## Verificacion recomendada

1. Lanzar el stack habitual.
2. Confirmar unicidad:
   - `ros2 node list | sort | uniq -d` (sin salida = OK)
3. Si hay dudas:
   - `scripts/diagnose_ros_duplicates.sh`
