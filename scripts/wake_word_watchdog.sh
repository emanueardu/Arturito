#!/usr/bin/env bash
# wake_word_watchdog.sh
#
# Watchdog externo para wake_word_listener_node: si el proceso sigue vivo
# pero NO escribe nada al journal de arturito.service durante 5 minutos,
# asume zombi y reinicia el servicio.
#
# Detecta el caso "PipeWire abrió el stream raro al boot y la calibración o
# pyaudio.read() quedaron colgados pidiendo samples que nunca llegan". El
# proceso parece vivo (`ps`, fuser ven el mic abierto) pero no progresa.
#
# Lógica de cada ciclo (60s):
#   - Cuento líneas de "wake_word_listener" en el journal últimos 5 min.
#   - Si conteo == 0 Y el proceso existe   → zombi → restart arturito + 90s pausa
#   - Si conteo == 0 Y no hay proceso      → no zombi, no hago nada
#   - Cualquier otro caso                  → todo bien, sigo vigilando

set -u

readonly TAG="wake_word_watchdog"
readonly SERVICE="arturito.service"
readonly POLL_INTERVAL_SEC=60
readonly RESTART_COOLDOWN_SEC=90
readonly JOURNAL_WINDOW="5 minutes ago"
readonly LISTENER_PATTERN="wake_word_listener"
readonly PROC_PATTERN="wake_word_listener_node"

log() {
    # Timestamp + log al syslog (visible con journalctl -t wake_word_watchdog).
    logger -t "${TAG}" -- "$*"
}

cleanup() {
    log "Recibida señal de terminación, saliendo."
    exit 0
}

# Manejo limpio de SIGTERM/SIGINT para que systemd pueda detener el watchdog
# sin necesidad de SIGKILL.
trap cleanup TERM INT

log "Watchdog iniciado (poll=${POLL_INTERVAL_SEC}s, ventana=${JOURNAL_WINDOW})."

while true; do
    # Conteo de líneas del listener en la ventana reciente.
    line_count="$(
        journalctl --user -u "${SERVICE}" \
            --since "${JOURNAL_WINDOW}" --no-pager 2>/dev/null \
            | grep -c "${LISTENER_PATTERN}"
    )"

    # Existencia del proceso del listener.
    if pgrep -f "${PROC_PATTERN}" >/dev/null 2>&1; then
        proc_alive=1
    else
        proc_alive=0
    fi

    if [[ "${line_count}" -eq 0 && "${proc_alive}" -eq 1 ]]; then
        log "ZOMBI detectado: 0 líneas de '${LISTENER_PATTERN}' en ${JOURNAL_WINDOW} pero el proceso sigue vivo. Reiniciando ${SERVICE}."
        if systemctl --user restart "${SERVICE}"; then
            log "Restart de ${SERVICE} OK. Pausa ${RESTART_COOLDOWN_SEC}s para estabilizar."
        else
            log "ERROR: 'systemctl --user restart ${SERVICE}' falló (rc=$?)."
        fi
        sleep "${RESTART_COOLDOWN_SEC}"
        continue
    fi

    if [[ "${line_count}" -eq 0 && "${proc_alive}" -eq 0 ]]; then
        # Sin líneas y sin proceso = el servicio está reiniciándose o down,
        # no nos metemos.
        log "Sin actividad pero proceso ausente; no intervengo."
    fi

    sleep "${POLL_INTERVAL_SEC}"
done
