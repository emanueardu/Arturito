#!/bin/bash
# verify_single_greeting.sh
# Script de verificación para confirmar que solo se publica un saludo de inicio
# en el topic /assistant/say durante el arranque del sistema.
#
# Uso:
#   ./verify_single_greeting.sh [duracion_en_segundos]
#
# Ejemplo:
#   ./verify_single_greeting.sh 10

set -e

DURATION=${1:-8}
TOPIC="/assistant/say"
OUTPUT_FILE="/tmp/greeting_check_$$.txt"

echo "=========================================="
echo "Verificación de saludo único al arranque"
echo "=========================================="
echo "Topic monitoreado: $TOPIC"
echo "Duración: ${DURATION}s"
echo ""

# Verificar que ROS2 esté disponible
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 no está disponible en el PATH"
    exit 1
fi

echo "Monitoreando topic por ${DURATION} segundos..."
echo "Presiona Ctrl+C para detener antes si es necesario."
echo ""

# Ejecutar ros2 topic echo en background con timeout
timeout ${DURATION}s ros2 topic echo $TOPIC > "$OUTPUT_FILE" 2>&1 || true

# Contar cuántos mensajes se recibieron
# Los mensajes en ros2 topic echo aparecen como "data: <texto>"
MESSAGE_COUNT=$(grep -c "^data:" "$OUTPUT_FILE" || echo "0")

echo ""
echo "=========================================="
echo "RESULTADOS"
echo "=========================================="
echo "Mensajes detectados en $TOPIC: $MESSAGE_COUNT"
echo ""

if [ "$MESSAGE_COUNT" -eq 0 ]; then
    echo "⚠ ADVERTENCIA: No se detectaron mensajes."
    echo "  Posible causa: el sistema aún no ha arrancado o el topic no está activo."
    echo ""
    cat "$OUTPUT_FILE"
    rm -f "$OUTPUT_FILE"
    exit 2
elif [ "$MESSAGE_COUNT" -eq 1 ]; then
    echo "✓ ÉXITO: Se detectó exactamente 1 saludo (comportamiento esperado)."
    echo ""
    echo "Contenido del mensaje:"
    grep "^data:" "$OUTPUT_FILE" | head -1
    echo ""
    rm -f "$OUTPUT_FILE"
    exit 0
else
    echo "✗ FALLO: Se detectaron $MESSAGE_COUNT saludos (se esperaba 1)."
    echo ""
    echo "Mensajes capturados:"
    grep "^data:" "$OUTPUT_FILE"
    echo ""
    echo "Archivo completo guardado en: $OUTPUT_FILE"
    exit 1
fi
