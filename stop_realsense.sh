#!/usr/bin/env bash
#
# stop_realsense.sh
# =================
#
#   This script does the stopping the realsense server that is running on the Go2 robot by the previous start_realsense.sh script. It does the following:
#   Use this when you're done, or when the stream is stuck and you want a clean
#   restart (stop, then run start_realsense.sh again).

set -uo pipefail

# ─── Config (shared with start_realsense.sh via camera_env.sh) ───────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=camera_env.sh
source "${SCRIPT_DIR}/camera_env.sh"

GO2_IP="$GO2_CAMERA_HOST"
GO2_USER="$GO2_CAMERA_USER"
PORT="$GO2_CAMERA_PORT"
# ─────────────────────────────────────────────────────────────────────────────

SSH="ssh -o ConnectTimeout=8 ${GO2_USER}@${GO2_IP}"

echo "[STOP] Stopping server on ${GO2_USER}@${GO2_IP}:${PORT} ..."

# One remote shell: stop the server gracefully, then force-kill if it lingers,
# and wait for the port to be released. Exit 0 if stopped, 1 if it wouldn't die.
# shellcheck disable=SC2086
$SSH bash -s -- "$PORT" <<'REMOTE'
set -u
PORT="$1"

server_running() { pgrep -f 'python.*realsense_server.py' >/dev/null 2>&1; }
port_listening() {
    (ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) \
        | grep -qE "[:.]${PORT}[[:space:]]"
}

if ! server_running; then
    echo "[GO2] No server process found — nothing to stop."
    # Still report the port so you know it's free.
    port_listening && echo "[GO2] (note: something else is on port ${PORT})"
    exit 0
fi

echo "[GO2] Found server (PID: $(pgrep -f 'python.*realsense_server.py' | tr '\n' ' '))"

# 1. Ask it to quit politely (SIGTERM) so it can release the camera cleanly.
echo "[GO2] Sending TERM ..."
pkill -TERM -f 'python.*realsense_server.py' 2>/dev/null || true
for _ in $(seq 1 10); do          # up to ~5s
    server_running || break
    sleep 0.5
done

# 2. If still alive, force it (SIGKILL).
if server_running; then
    echo "[GO2] Still alive — sending KILL ..."
    pkill -KILL -f 'python.*realsense_server.py' 2>/dev/null || true
    for _ in $(seq 1 6); do        # up to ~3s
        server_running || break
        sleep 0.5
    done
fi

if server_running; then
    echo "[GO2][ERROR] Server process would not die." >&2
    exit 1
fi

# 3. Wait for the port to be released so a restart won't hit a busy device.
for _ in $(seq 1 10); do
    port_listening || break
    sleep 0.5
done
if port_listening; then
    echo "[GO2] Server stopped, but port ${PORT} still shows listening (may clear shortly)."
else
    echo "[GO2] Server stopped and port ${PORT} released."
fi
exit 0
REMOTE
rc=$?

if [ "$rc" -eq 0 ]; then
    echo "[STOP] Done — server is stopped."
else
    echo "[STOP][ERROR] Could not stop the server (see lines above)." >&2
fi
exit "$rc"
