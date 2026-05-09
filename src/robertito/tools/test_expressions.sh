#!/usr/bin/env bash
# Test E2E: dispara las 13 expresiones secuencialmente, 3 segundos cada una.
# Asume que el nodo robertito_eyes ya está corriendo (vos lo lanzas con
# ros2 launch antes de correr esto).

set -e

EXPRS=(calma atento pensativo feliz triste sorprendido dormido
       corazones risa_fuerte picaron saludo dormitando enojado)

TOPIC="robertito/eyes_expression"

echo "Test E2E de expresiones de Robertito"
echo "Topic: $TOPIC"
echo "Expresiones: ${#EXPRS[@]}"
echo ""

for expr in "${EXPRS[@]}"; do
    echo ">>> $expr"
    ros2 topic pub --once "$TOPIC" std_msgs/msg/String "{data: '$expr'}"
    sleep 3
done

echo ""
echo ">>> Volviendo a calma"
ros2 topic pub --once "$TOPIC" std_msgs/msg/String "{data: 'calma'}"
echo "Test completo."
