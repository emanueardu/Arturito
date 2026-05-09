#!/usr/bin/env bash
# Smoke-test E2E del presence_orchestrator_node.
#
# Pre-requisitos:
#   - El robot apagado (no systemd, no robot real moviéndose).
#   - El orchestrator corriendo en otra terminal (vos lo lanzás antes):
#       ros2 launch robertito robertito_launch.py
#
# El script publica eventos de sensores y deja al humano observar la
# respuesta visual + auditiva + de motor. NO hace asserts automáticos.

set -e

echo "=== TEST 1: Bumper LEFT ==="
ros2 topic pub --once /arturito/bumper_left std_msgs/msg/Bool "{data: true}"
sleep 4
echo "Esperado: expr=sorprendido, frase de bump, reverse + spin a la derecha"
echo ""

echo "=== TEST 2: Bumper RIGHT ==="
ros2 topic pub --once /arturito/bumper_right std_msgs/msg/Bool "{data: true}"
sleep 4
echo "Esperado: expr=sorprendido, frase de bump, reverse + spin a la izquierda"
echo ""

echo "=== TEST 3: Cliff ==="
ros2 topic pub --once /arturito/cliff std_msgs/msg/Bool "{data: true}"
sleep 4
echo "Esperado: expr=sorprendido, frase de cliff, reverse 10cm"
ros2 topic pub --once /arturito/cliff std_msgs/msg/Bool "{data: false}"
sleep 1
echo ""

echo "=== TEST 4: Startled (US ~15cm) ==="
ros2 topic pub --once /arturito/ultrasonic sensor_msgs/msg/Range \
  "{header: {frame_id: 'us'}, radiation_type: 0, field_of_view: 0.5, min_range: 0.02, max_range: 2.0, range: 0.15}"
sleep 5
echo "Esperado: expr=sorprendido, frase de startled, retreat 10cm"
echo ""

echo "=== TEST 5: Wake word ==="
ros2 topic pub --once /wake_word/detected std_msgs/msg/Bool "{data: true}"
sleep 3
echo "Esperado: expr=atento, frase de wake_ack"
echo ""

echo "=== TEST 6: Cara nueva (greeting según ausencia) ==="
echo "(skip — requiere construcción de Detection2DArray con 1 detección)"
echo "Para probar manual: arrimate al robot con la cara visible y observá la transición."
echo ""

echo "=== TEST 7: Estado actual del orchestrator ==="
ros2 service call /orchestrator/get_state std_srvs/srv/Trigger
echo ""

echo "Tests E2E completos. Verificá visualmente y por logs."
