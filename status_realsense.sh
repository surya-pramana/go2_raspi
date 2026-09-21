#!/usr/bin/env bash
#
# status_realsense.sh
#   It changes NOTHING — it only looks. Safe to run any time.
#
set -uo pipefail

# ─── Config (shared with start_realsense.sh via camera_env.sh) ───────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=camera_env.sh
source "${SCRIPT_DIR}/camera_env.sh"

GO2_IP="$GO2_CAMERA_HOST"
GO2_USER="$GO2_CAMERA_USER"
PORT="$GO2_CAMERA_PORT"
LOG_PATH="${LOG_PATH:-/tmp/realsense_server.log}"
# ─────────────────────────────────────────────────────────────────────────────

SSH="ssh -o ConnectTimeout=8 ${GO2_USER}@${GO2_IP}"

echo "[STATUS] Checking ${GO2_USER}@${GO2_IP}:${PORT} ..."

# Run all checks in ONE remote shell (one SSH round-trip). The remote script
# prints a status and exits 0 (up+listening) or 1 (not fully up).
# shellcheck disable=SC2086
$SSH bash -s -- "$PORT" "$LOG_PATH" "$GO2_STATUS_PORT" "$GO2_CAMERA_IO" <<'REMOTE'
set -u
PORT="$1"
LOG_PATH="$2"
STATUS_PORT="$3"
ENABLE_IO="$4"

# PID(s) of the actual python server (ignore editors/greps with the name in them).
PIDS="$(pgrep -f 'python.*realsense_server.py' 2>/dev/null || true)"

port_listening() {
    (ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) \
        | grep -qE "[:.]${PORT}[[:space:]]"
}

if [ -n "$PIDS" ]; then
    echo "[GO2] Server RUNNING  (PID: $(echo "$PIDS" | tr '\n' ' '))"
else
    echo "[GO2] Server NOT running."
fi

if port_listening; then
    echo "[GO2] Port ${PORT} LISTENING (ready for a client)."
    LISTEN=1
else
    echo "[GO2] Port ${PORT} not listening."
    LISTEN=0
fi

if [ -f "$LOG_PATH" ]; then
    echo "[GO2] Last log lines (${LOG_PATH}):"
    tail -n 8 "$LOG_PATH" 2>/dev/null | sed 's/^/[GO2][log] /'
else
    echo "[GO2] No log file at ${LOG_PATH} yet."
fi

if [ "$ENABLE_IO" = "true" ]; then
    if (ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) \
        | grep -qE "[:.]${STATUS_PORT}[[:space:]]"; then
        echo "[GO2] IO WebSocket port ${STATUS_PORT} LISTENING (check payload for GPIO health)."
    else
        echo "[GO2] IO WebSocket port ${STATUS_PORT} not listening."
        exit 1
    fi
fi

# "Up" means both a live process AND the port open.
[ -n "$PIDS" ] && [ "$LISTEN" = "1" ] && exit 0
exit 1
REMOTE
rc=$?

if [ "$rc" -eq 0 ]; then
    echo "[STATUS] Result: UP — server running and accepting connections."
else
    echo "[STATUS] Result: DOWN — server not fully up (see lines above)."
fi
exit "$rc"
