# Contrato de modos en Robertito

## Modo "limpieza rápida"

### Activación
Trigger: `Bool(True)` en `/assistant/mode/cleaning_quick`.

Source-of-truth: `clean_quick_node`.

### Comportamiento al entrar
1. `clean_quick` publica `/robot_web/clean_status=True`
2. `uart_node` abre el gate `_clean_active`
3. `clean_quick` anuncia "Empezando limpieza" por TTS
4. Tilt sube a `tilt_active_deg` (default 60°) para no chocar muebles bajos
5. Vacuum y brush se prenden (servicios async)
6. `presence_orchestrator` detecta `clean_mode_active=True` y MUTEA:
   - No publica más Twist a `/cmd_vel`
   - No dispara `motor_gestures` (cliff, bumper, proximity reactions)
   - Fija expresión en "atento" (ignora otras hasta que termine)
7. `clean_quick` toma control exclusivo del Twist vía `arturito/cmd_vel_clean`

### Durante limpieza
- **Bumpers:** solo respondidos por `clean_quick` (con su propia lógica de reverse + spin)
- **Ultrasónico:** solo respondido por `clean_quick` (obstacle avoidance)
- **Cliff:** SAFETY hardware (firmware ESP32) + `clean_quick` si lo expone
- **Voz wake word:** sigue funcionando (`api_chat` puede recibir órdenes)

### Comportamiento al salir
Trigger: `Bool(False)` en `/assistant/mode/cleaning_quick`.

1. `clean_quick` anuncia "Limpieza terminada" por TTS
2. Twist cero a `arturito/cmd_vel_clean` (motores parados)
3. Vacuum y brush se apagan (servicios async)
4. Tilt vuelve a `tilt_idle_deg` (default 0°)
5. `clean_quick` publica `/robot_web/clean_status=False`
6. `uart_node` cierra el gate `_clean_active`
7. `presence_orchestrator` detecta `clean_mode_active=False` y reanuda
   comportamiento normal (gestures + cmd_vel + expresiones libres)

## Reglas de oro

1. **SOLO `clean_quick` puede mover ruedas durante limpieza.**
2. **SOLO `clean_quick` puede tocar la lógica de bumpers/proximidad durante limpieza.**
3. **`presence_orchestrator` es source-of-truth de gestures y expresiones EN MODO NORMAL.**
4. **`uart_node` es la única vía al firmware ESP32. Nadie escribe directo al puerto.**
5. **El gate `_clean_active` es señal de bus, no de estado interno:** solo `clean_quick` lo toggle, todos los demás lo escuchan.
