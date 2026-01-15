#!/bin/bash
# apply_and_verify_fix.sh
# Script completo para reiniciar servicio y validar el fix de duplicación de saludo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="/home/robot/ros2_ws"

echo "=========================================="
echo "Fix Duplicación de Saludo - Restart & Verify"
echo "=========================================="
echo ""

# Check if running with appropriate permissions
if ! sudo -n true 2>/dev/null; then
    echo "⚠️  Este script requiere permisos sudo para reiniciar arturito.service"
    echo ""
    echo "Por favor ejecuta:"
    echo "  sudo $0"
    echo ""
    echo "O configura NOPASSWD en sudoers para el servicio arturito."
    exit 1
fi

echo "✓ Permisos verificados"
echo ""

# Step 1: Verificar build
echo "1. Verificando archivos instalados..."
INSTALLED_NODE_MANAGER="$WORKSPACE_DIR/install/robertito/lib/python3.12/site-packages/robertito/node_manager.py"
INSTALLED_LAUNCH="$WORKSPACE_DIR/install/robertito/share/robertito/launch/robertito_launch.py"

if [ ! -f "$INSTALLED_NODE_MANAGER" ]; then
    echo "❌ ERROR: No se encuentra node_manager.py instalado"
    echo "   Ejecuta: cd $WORKSPACE_DIR && colcon build --packages-select robertito"
    exit 1
fi

if grep -q 'startup_greeting", ""' "$INSTALLED_NODE_MANAGER" && \
   grep -q '.strip():' "$INSTALLED_NODE_MANAGER"; then
    echo "   ✓ node_manager.py: Cambios presentes"
else
    echo "   ⚠️  node_manager.py: Cambios NO detectados"
    echo "   Ejecuta rebuild: cd $WORKSPACE_DIR && colcon build --packages-select robertito"
    exit 1
fi

if grep -q 'startup_greeting_arg = DeclareLaunchArgument' "$INSTALLED_LAUNCH"; then
    echo "   ✓ robertito_launch.py: Cambios presentes"
else
    echo "   ⚠️  robertito_launch.py: Cambios NO detectados"
    exit 1
fi

echo ""

# Step 2: Restart service
echo "2. Reiniciando servicio arturito..."
sudo systemctl restart arturito.service
echo "   ✓ Servicio reiniciado"
echo ""

# Step 3: Wait for nodes to initialize
echo "3. Esperando que los nodos inicialicen (8 segundos)..."
sleep 8
echo "   ✓ Nodos listos"
echo ""

# Step 4: Verify service status
echo "4. Verificando estado del servicio..."
if systemctl is-active --quiet arturito.service; then
    echo "   ✓ Servicio activo"
else
    echo "   ❌ ERROR: Servicio no está activo"
    echo ""
    echo "Ver logs con:"
    echo "  journalctl -u arturito.service -n 50"
    exit 1
fi
echo ""

# Step 5: Check logs
echo "5. Analizando logs del arranque..."
STARTUP_MESSAGES=$(journalctl -u arturito.service --since "1 minute ago" | grep -c "Startup message queued" || echo "0")
NODE_MANAGER_LOGS=$(journalctl -u arturito.service --since "1 minute ago" | grep "robertito_node_manager" | tail -3)

echo "   Mensajes 'Startup message queued': $STARTUP_MESSAGES"

if [ "$STARTUP_MESSAGES" -eq 1 ]; then
    echo "   ✓ Solo 1 startup message detectado (correcto)"
elif [ "$STARTUP_MESSAGES" -eq 0 ]; then
    echo "   ⚠️  No se detectó startup message (posible issue)"
else
    echo "   ❌ Se detectaron $STARTUP_MESSAGES startup messages (duplicación persiste)"
fi

echo ""
echo "   Logs del node_manager:"
echo "$NODE_MANAGER_LOGS" | sed 's/^/     /'
echo ""

# Step 6: Verify parameter
echo "6. Verificando parámetro startup_greeting..."
cd "$WORKSPACE_DIR"
source install/setup.bash

# Wait for node to be available
sleep 2

PARAM_VALUE=$(ros2 param get /robertito_node_manager startup_greeting 2>/dev/null || echo "ERROR")

if [ "$PARAM_VALUE" = "String value is: ''" ] || [ "$PARAM_VALUE" = "String value is: " ]; then
    echo "   ✓ startup_greeting está vacío (correcto)"
elif echo "$PARAM_VALUE" | grep -q "ERROR"; then
    echo "   ⚠️  No se pudo leer el parámetro (nodo aún inicializando?)"
else
    echo "   ⚠️  startup_greeting tiene valor: $PARAM_VALUE"
fi
echo ""

# Step 7: Run automated verification
echo "7. Ejecutando verificación automática..."
if [ -x "$SCRIPT_DIR/verify_single_greeting.sh" ]; then
    if "$SCRIPT_DIR/verify_single_greeting.sh" 6; then
        echo ""
        echo "=========================================="
        echo "✅ ÉXITO: Fix aplicado correctamente"
        echo "=========================================="
        echo ""
        echo "El sistema ahora publica un único saludo al arrancar."
        echo ""
        exit 0
    else
        EXIT_CODE=$?
        echo ""
        echo "=========================================="
        if [ $EXIT_CODE -eq 2 ]; then
            echo "⚠️  ADVERTENCIA: No se detectaron mensajes"
            echo "=========================================="
            echo ""
            echo "Posibles causas:"
            echo "  - El sistema ya arrancó hace más de 6 segundos"
            echo "  - Los nodos están aún inicializando"
            echo ""
            echo "Ejecuta manualmente:"
            echo "  sudo systemctl restart arturito.service"
            echo "  sleep 2"
            echo "  $SCRIPT_DIR/verify_single_greeting.sh 8"
        else
            echo "❌ FALLO: Verificación detectó múltiples saludos"
            echo "=========================================="
            echo ""
            echo "Troubleshooting:"
            echo "  1. Ver logs: journalctl -u arturito.service -n 100"
            echo "  2. Verificar parámetros: ros2 param list /robertito_node_manager"
            echo "  3. Revisar archivos fuente en: $WORKSPACE_DIR/src/robertito"
        fi
        exit 1
    fi
else
    echo "   ⚠️  Script verify_single_greeting.sh no encontrado o no ejecutable"
    echo ""
    echo "Verificación manual requerida:"
    echo "  timeout 10s ros2 topic echo /assistant/say"
    exit 1
fi
