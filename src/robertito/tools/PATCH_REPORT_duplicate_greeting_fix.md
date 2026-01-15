# Fix: Eliminar Duplicación de Saludo de Inicio

## 📋 Resumen

Este patch elimina la duplicación del saludo "buenos días" que ocurría al arranque del sistema. La causa raíz era que dos nodos publicaban independientemente mensajes de saludo al topic `/assistant/say`:
1. `voice_synth_node` con `startup_message`
2. `robertito_node_manager` con `startup_greeting`

**Solución:** Establecer `startup_greeting` vacío por defecto en `node_manager`, manteniendo `startup_message` como mecanismo principal de saludo.

---

## 🔍 Causa Raíz

### Flujo de Arranque Actual
```
arturito.service
  └─> start_arturito.sh
      └─> ros2 launch robot_web_bridge robot_auto.launch.py
          └─> robertito_launch.py
              ├─> voice_synth_node (publica startup_message)
              └─> robertito_node_manager (publicaba startup_greeting)
```

### Problema Identificado
- **voice_synth_node.py:179-180**: Encola `startup_message` si no está vacío
  ```python
  if self._startup_message:
      self._schedule_startup_message()
  ```

- **node_manager.py:27-28, 46-47** (ANTES del patch): Publicaba `startup_greeting` con valor por defecto
  ```python
  self._startup_greeting = self.declare_parameter(
      "startup_greeting", "Hola, buenos días. Soy Robertito."  # ← Valor hardcodeado
  ).get_parameter_value().string_value
  ...
  if self._startup_greeting:  # ← Sin .strip(), cadena vacía es falsy pero no detecta whitespace
      self._publish_tts(self._startup_greeting)
  ```

- **robertito_launch.py:232** (ANTES del patch): Parámetro hardcodeado
  ```python
  {"startup_greeting": "Hola, buenos días. Soy Robertito."},  # ← Duplicaba el saludo
  ```

**Resultado:** Dos saludos publicados al mismo topic → audio duplicado.

---

## ✅ Cambios Implementados

### 1. `src/robertito/robertito/node_manager.py`

**Líneas 27-29:** Cambiar default de `startup_greeting` a cadena vacía
```diff
  self._startup_greeting = self.declare_parameter(
-     "startup_greeting", "Hola, buenos días. Soy Robertito."
+     "startup_greeting", ""
  ).get_parameter_value().string_value
```

**Líneas 46-47:** Agregar `.strip()` para manejar correctamente cadenas vacías/whitespace
```diff
- if self._startup_greeting:
+ if self._startup_greeting.strip():
      self._publish_tts(self._startup_greeting)
```

**Justificación:**
- Default vacío evita publicación automática
- `.strip()` previene publicación de strings con solo espacios
- Mantiene compatibilidad: si se configura explícitamente, se publica

---

### 2. `src/robertito/launch/robertito_launch.py`

**Líneas 62-66:** Agregar `DeclareLaunchArgument` para `startup_greeting`
```diff
  voice_mode_arg = DeclareLaunchArgument(
      "voice_mode",
      default_value="jarvis",
      description="Modo de voz: normal o jarvis.",
  )
+ startup_greeting_arg = DeclareLaunchArgument(
+     "startup_greeting",
+     default_value="",
+     description="Saludo inicial del node manager. Vacío por defecto (usa startup_message de voice_synth).",
+ )
```

**Línea 79:** Declarar `LaunchConfiguration`
```diff
  voice_mode = LaunchConfiguration("voice_mode")
+ startup_greeting = LaunchConfiguration("startup_greeting")
```

**Línea 232:** Usar LaunchConfiguration en lugar de valor hardcodeado
```diff
  parameters=[
      {"wake_topic": wake_topic},
      {"eyes_expression_topic": "robertito/eyes_expression"},
      {"tts_topic": assistant_say_topic},
-     {"startup_greeting": "Hola, buenos días. Soy Robertito."},
+     {"startup_greeting": startup_greeting},
      {"wake_greeting": "Hola, ¿en qué puedo ayudarte?"},
  ],
```

**Línea 253:** Agregar argumento a LaunchDescription
```diff
  ld.add_action(voice_mode_arg)
+ ld.add_action(startup_greeting_arg)
  ld.add_action(eyes_node)
```

**Justificación:**
- Permite override vía launch argument si se necesita
- Default vacío = no publicar
- Documentación inline clara sobre el propósito

---

### 3. `src/robertito/tools/verify_single_greeting.sh` (NUEVO)

Script de verificación bash que monitorea `/assistant/say` por N segundos y cuenta mensajes.

**Características:**
- Timeout configurable (default 8s)
- Códigos de salida: 0=éxito (1 mensaje), 1=fallo (>1 mensaje), 2=no hay mensajes
- Output claro con símbolos ✓/✗/⚠

---

## 📁 Archivos Modificados

```
src/robertito/robertito/node_manager.py          (2 cambios: líneas 28, 46)
src/robertito/launch/robertito_launch.py         (4 cambios: líneas 62-66, 79, 232, 253)
src/robertito/tools/verify_single_greeting.sh    (NUEVO: 73 líneas)
```

---

## 🧪 Validación Post-Restart

### Pre-requisitos
Asegurate de rebuild el workspace después de aplicar el patch:
```bash
cd /home/robot/ros2_ws
colcon build --packages-select robertito
source install/setup.bash
```

### 1. Reiniciar el Sistema
```bash
sudo systemctl restart arturito.service
```

### 2. Verificar Logs del Servicio
```bash
# Ver logs recientes del servicio
journalctl -u arturito.service -n 100 --no-pager

# Buscar mensajes de node_manager (NO debe aparecer "Hola, buenos días")
journalctl -u arturito.service | grep -i "robertito_node_manager"

# Buscar mensajes de voice_synth (DEBE aparecer "Startup message queued")
journalctl -u arturito.service | grep -i "startup message"
```

**Esperado:**
```
[voice_synth_node]: Startup message queued for speech.
[robertito_node_manager]: Node manager escuchando activaciones en /wake_word/detected
```

### 3. Monitorear Topic en Tiempo Real
```bash
# En una terminal, monitorear el topic por 10 segundos después del restart
timeout 10s ros2 topic echo /assistant/say
```

**Esperado:** Ver UNO solo mensaje con el texto configurado en `startup_message`.

### 4. Ejecutar Script de Verificación Automática
```bash
cd /home/robot/ros2_ws/src/robertito/tools
./verify_single_greeting.sh 8
```

**Output esperado:**
```
==========================================
Verificación de saludo único al arranque
==========================================
Topic monitoreado: /assistant/say
Duración: 8s

Monitoreando topic por 8 segundos...

==========================================
RESULTADOS
==========================================
Mensajes detectados en /assistant/say: 1

✓ ÉXITO: Se detectó exactamente 1 saludo (comportamiento esperado).

Contenido del mensaje:
data: Hola, soy Robertito
```

### 5. Verificar Parámetros del Node Manager
```bash
# Confirmar que startup_greeting está vacío
ros2 param get /robertito_node_manager startup_greeting
```

**Esperado:**
```
String value is: ''
```

### 6. Verificar Info del Topic
```bash
ros2 topic info /assistant/say -v
```

**Esperado:** Ver `voice_synth_node` como suscriptor y `robertito_node_manager` como publisher potencial (pero sin actividad al inicio).

---

## 🔄 Compatibilidad y Rollback

### Forzar startup_greeting (si alguien lo necesita)

**Opción A:** Via launch argument
```bash
ros2 launch robertito robertito_launch.py startup_greeting:="Mi mensaje personalizado"
```

**Opción B:** Modificar robot_auto.launch.py para pasar el argumento
```python
# En robot_web_bridge/launch/robot_auto.launch.py
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('robertito'), 'launch', 'robertito_launch.py')
    ]),
    launch_arguments={
        'startup_greeting': 'Hola desde robot_auto',
        # ... otros argumentos
    }.items()
)
```

**Opción C:** Via parámetro YAML (agregar en config)
```yaml
# robertito/config/custom_params.yaml
robertito_node_manager:
  ros__parameters:
    startup_greeting: "Mi saludo custom"
```

### Rollback Rápido

Si necesitás revertir el patch:

```bash
cd /home/robot/ros2_ws/src/robertito

# Revertir node_manager.py
git checkout robertito/node_manager.py

# Revertir robertito_launch.py
git checkout launch/robertito_launch.py

# Rebuild
cd /home/robot/ros2_ws
colcon build --packages-select robertito
source install/setup.bash

# Restart
sudo systemctl restart arturito.service
```

---

## ⚠️ Riesgos y Consideraciones

### Riesgos Bajos
1. **Backward compatibility:** ✅ Ningún código depende de `startup_greeting` teniendo valor por defecto. Es un parámetro local al nodo.
2. **Launch files externos:** ⚠️ Si hay otros launch files que incluyen `robertito_launch.py` y esperan que el node manager salude, ahora deben pasar explícitamente `startup_greeting`.
3. **Tests:** ℹ️ No se identificaron tests unitarios para `node_manager.py` que validen el saludo de inicio.

### Validaciones Adicionales Recomendadas
- Verificar que `robot_web_bridge/robot_auto.launch.py` no dependa del saludo duplicado
- Confirmar que no hay configuraciones YAML externas que overrideen `startup_greeting`
- Testear un ciclo completo: boot → wake word → saludo de activación (debe seguir funcionando)

### Impacto en Otros Componentes
- ✅ `voice_synth_node`: Sin cambios, sigue funcionando igual
- ✅ `wake_word_listener_node`: Sin cambios
- ✅ `api_chat_node`: Sin cambios
- ✅ `arturito.service`: Sin cambios en el flujo de arranque
- ✅ Otros nodos: Sin dependencias cruzadas

---

## 📊 Checklist de Validación

Después de aplicar el patch, confirmar:

- [ ] `colcon build --packages-select robertito` completa sin errores
- [ ] `sudo systemctl restart arturito.service` exitoso
- [ ] Logs muestran solo 1 "Startup message queued"
- [ ] `ros2 topic echo /assistant/say` captura 1 solo mensaje en 10s
- [ ] Script `verify_single_greeting.sh` retorna exit code 0
- [ ] `ros2 param get /robertito_node_manager startup_greeting` retorna cadena vacía
- [ ] Wake word sigue funcionando (decir "robertito" → escuchar "Hola, ¿en qué puedo ayudarte?")
- [ ] No hay errores en `journalctl -u arturito.service -f`

---

## 🎯 Conclusión

Este patch implementa un **fix mínimo, limpio y testeable** que:
- ✅ Elimina la duplicación del saludo sin romper compatibilidad
- ✅ Mantiene `startup_message` como mecanismo principal (limpio, centralizado)
- ✅ Permite override explícito de `startup_greeting` vía launch args si se necesita
- ✅ Incluye herramienta de verificación automatizada
- ✅ No requiere cambios en `arturito.service` ni flujo de arranque
- ✅ Documentado con diffs, comandos exactos y estrategia de rollback

**Próximos pasos opcionales:**
1. Agregar test unitario en `test_node_manager.py` para validar comportamiento condicional
2. Documentar en `README.md` que el saludo de boot se configura vía `startup_message`
3. Actualizar `duplicate_nodes.md` (si existe) explicando la resolución del issue

---

**Autor:** Mantenedor Senior ROS2 Jazzy
**Fecha:** 2026-01-15
**Versión ROS2:** Jazzy
**Target:** Raspberry Pi 5 / Ubuntu
**Estado:** ✅ Listo para aplicar
