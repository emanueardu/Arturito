# Quick Start: Fix Duplicación de Saludo

## 🚀 Aplicar el Patch (Los cambios ya están hechos)

```bash
# 1. Navegar al workspace
cd /home/robot/ros2_ws

# 2. Rebuild el paquete robertito
colcon build --packages-select robertito

# 3. Source el workspace
source install/setup.bash

# 4. Reiniciar el servicio
sudo systemctl restart arturito.service
```

## ✅ Verificación Rápida (5 minutos)

### Opción A: Script Automatizado
```bash
cd /home/robot/ros2_ws/src/robertito/tools
./verify_single_greeting.sh 8
```

**Esperado:** `✓ ÉXITO: Se detectó exactamente 1 saludo`

### Opción B: Verificación Manual
```bash
# Ver logs del arranque (buscar UNA sola instancia de startup message)
journalctl -u arturito.service -n 50 | grep -i "startup\|greeting"

# Verificar parámetro vacío
ros2 param get /robertito_node_manager startup_greeting
# Debe retornar: String value is: ''

# Monitorear topic (debe haber 1 solo mensaje al arrancar)
timeout 10s ros2 topic echo /assistant/say
```

## 📋 Resumen de Cambios

### Archivos Modificados
```
✏️  src/robertito/robertito/node_manager.py          (2 líneas)
✏️  src/robertito/launch/robertito_launch.py         (4 secciones)
➕  src/robertito/tools/verify_single_greeting.sh    (nuevo)
```

### Cambio Principal
**ANTES:** `node_manager` publicaba saludo hardcodeado + `voice_synth` publicaba `startup_message` = 2 saludos
**AHORA:** Solo `voice_synth` publica `startup_message` = 1 saludo

### Default Values
- `startup_greeting` en node_manager: `""` (vacío)
- `startup_message` en voice_synth: `"Hola, soy Robertito"` (sin cambios)

## 🔧 Configuración Avanzada

### Forzar startup_greeting del node_manager (opcional)
```bash
ros2 launch robertito robertito_launch.py startup_greeting:="Buenos días desde node manager"
```

### Cambiar startup_message del voice_synth
```bash
ros2 launch robertito robertito_launch.py startup_message:="Hola, sistema iniciado correctamente"
```

## 🆘 Troubleshooting

### Problema: Siguen apareciendo 2 saludos
```bash
# Verificar que el build fue exitoso
ls -lh install/robertito/lib/python3.12/site-packages/robertito/node_manager.py

# Ver fecha de modificación (debe ser reciente)
stat src/robertito/robertito/node_manager.py

# Forzar rebuild limpio
rm -rf build/robertito install/robertito
colcon build --packages-select robertito
source install/setup.bash
sudo systemctl restart arturito.service
```

### Problema: No hay ningún saludo
```bash
# Verificar que voice_synth tiene startup_message configurado
ros2 param get /voice_synth_node startup_message

# Debe retornar algo como: String value is: 'Hola, soy Robertito'
# Si está vacío, reiniciar con valor explícito:
ros2 launch robertito robertito_launch.py startup_message:="Hola, soy Robertito"
```

### Problema: Servicio no arranca
```bash
# Ver errores del servicio
journalctl -u arturito.service -xe

# Verificar sintaxis Python (no debe haber errores)
python3 -m py_compile src/robertito/robertito/node_manager.py
python3 -m py_compile src/robertito/launch/robertito_launch.py
```

## 📖 Documentación Completa

Para entender la causa raíz, diffs detallados y estrategia de rollback, ver:
```bash
cat /home/robot/ros2_ws/src/robertito/tools/PATCH_REPORT_duplicate_greeting_fix.md
```

## 🔄 Rollback (si es necesario)

```bash
cd /home/robot/ros2_ws/src/robertito

# Opción 1: Git revert (si está en git)
git checkout HEAD -- robertito/node_manager.py launch/robertito_launch.py

# Opción 2: Aplicar patch inverso
cd /home/robot/ros2_ws
patch -p1 -R < src/robertito/tools/duplicate_greeting_fix.patch

# Rebuild y restart
colcon build --packages-select robertito
source install/setup.bash
sudo systemctl restart arturito.service
```

---

**Status:** ✅ Cambios aplicados, listo para rebuild + test
**Tiempo estimado:** 5 minutos (build) + 2 minutos (verificación)
**Riesgo:** 🟢 Bajo (cambio aislado, backward compatible)
