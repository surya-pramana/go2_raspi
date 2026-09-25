#!/usr/bin/env bash
# start_realsense.sh
# ==================
#
#   This ONE script does all of these
#     1. Copies the latest camera program (realsense_server.py) onto the Go2, so
#        the robot always runs the newest version of the code.
#     2. Logs into the Go2 over SSH and starts that program in the background
#        (it keeps running even after the SSH login closes).
#     3. Waits until the camera program is ready and accepting connections.
#     4. Starts the viewer (realsense_client.py) on your PC so you see the video.

# ── HOW TO RUN ───────────────────────────────────────────────────────────────
#   ./start_realsense.sh
#
#   password connection to go2 already satisfied by using the ssh key method
#   From now on ./start_realsense.sh runs without any password.

set -euo pipefail # used for better error handling in bash scripts

# ─── Config ──────────────────────────────────────────────────────────────
# Host/user/port/path all come from camera_env.sh so this script, stop_ and
# status_ can never disagree about which machine the camera is on.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=camera_env.sh
source "${SCRIPT_DIR}/camera_env.sh"

GO2_IP="$GO2_CAMERA_HOST"
GO2_USER="$GO2_CAMERA_USER"
PORT="$GO2_CAMERA_PORT"
SERVER_PATH="$GO2_CAMERA_SERVER_PATH"

# Push the local (hardened) server to the camera host before starting. Set to
# "false" if you intentionally maintain a different server there.
SYNC_SERVER="${SYNC_SERVER:-true}"

# Run the local viewer (realsense_client.py) after the server is up. Set to
# "false" to ONLY (re)start the server on the Go2 and exit — use this before
# launching the AprilTag detector (apriltag_checkpoint_tcp.launch.py), since the
# server serves one client at a time and the detector IS that client:
#   RUN_CLIENT=false ./start_realsense.sh
RUN_CLIENT="${RUN_CLIENT:-false}" #false as default


WAIT_TIMEOUT="${WAIT_TIMEOUT:-25}"   # seconds to wait for the port to open
# ─────────────────────────────────────────────────────────────────────────

# Password auth, only when asked for.
#
# The header above says an SSH key makes this passwordless, and that is still
# the right setup — run `ssh-copy-id ${GO2_CAMERA_USER}@${GO2_CAMERA_HOST}`
# once and none of this applies again. Until that is done the camera host
# answers with "Permission denied (publickey,password)", and the GUI has no
# terminal to type a password into.
#
# So: if SSHPASS is set in the environment, ssh/scp go through sshpass -e.
# Via the ENVIRONMENT deliberately — `sshpass -p <secret>` puts the password in
# the process list where any user on the box can read it with ps.
if [ -n "${SSHPASS:-}" ]; then
    if ! command -v sshpass >/dev/null 2>&1; then
        echo "[LAUNCH][ERROR] SSHPASS is set but sshpass is not installed" >&2
        exit 1
    fi
    SSHPASS_CMD="sshpass -e"
    SSH_OPTS="-o ConnectTimeout=8 -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no"
else
    SSHPASS_CMD=""
    SSH_OPTS="-o ConnectTimeout=8"
fi

# shellcheck disable=SC2086
SSH="${SSHPASS_CMD} ssh ${SSH_OPTS} ${GO2_USER}@${GO2_IP}"
LOCAL_SERVER="${SCRIPT_DIR}/realsense_server.py"

echo "[LAUNCH] Target: ${GO2_USER}@${GO2_IP}:${PORT}"

# 0. Sync the hardened server to the Go2 so the latest code actually runs.
#    (Without this, an old crashy copy on the Go2 keeps getting launched.)
FORCE_RESTART=0
if [ "$SYNC_SERVER" = "true" ]; then
    if [ ! -f "$LOCAL_SERVER" ]; then
        echo "[LAUNCH][ERROR] Local server not found: $LOCAL_SERVER" >&2
        exit 1
    fi
    echo "[LAUNCH] Syncing server to Go2 (${SERVER_PATH}) ..."
    # shellcheck disable=SC2086
    ${SSHPASS_CMD} scp ${SSH_OPTS} "$LOCAL_SERVER" \
        "${GO2_USER}@${GO2_IP}:${SERVER_PATH}"
    # The camera server imports the local GPIO/WebSocket module.
    ${SSHPASS_CMD} scp ${SSH_OPTS} "${SCRIPT_DIR}/realsense_io.py" \
        "${GO2_USER}@${GO2_IP}:${SERVER_PATH%/*}/realsense_io.py"
    ${SSHPASS_CMD} scp ${SSH_OPTS} "${SCRIPT_DIR}/research_metrics.py" \
        "${GO2_USER}@${GO2_IP}:${SERVER_PATH%/*}/research_metrics.py"
    FORCE_RESTART=1   # code may have changed → restart even if already running
fi

# 1. (Re)start the server on the Go2.
echo "[LAUNCH] Checking / starting server on Go2 ..."
# shellcheck disable=SC2086
$SSH bash -s -- "$SERVER_PATH" "$PORT" "$FORCE_RESTART" \
    "$GO2_STATUS_PORT" "$GO2_CAMERA_IO" "$GO2_CAMERA_PYTHON" "$GO2_CAMERA_SUDO" <<'REMOTE'
set -e
SERVER_PATH="$1"
PORT="$2"
FORCE_RESTART="$3"
STATUS_PORT="$4"
ENABLE_IO="$5"
CAMERA_PYTHON="$6"
PRIVILEGE=()
if [ "$7" = "true" ]; then
    PRIVILEGE=(sudo -n)
    sudo -n true || { echo "[ERROR] Noninteractive sudo is unavailable." >&2; exit 1; }
fi

port_listening() {
    # True only if something is actually bound+LISTENing on $PORT.
    (ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) \
        | grep -qE "[:.]${PORT}[[:space:]]"
}

# Only count an *actual python server*, not an editor (nano/vim) or grep that
# happens to have the filename on its command line — that was masking a dead
# server and skipping the (re)launch.
server_running() {
    pgrep -f "python.*realsense_server.py" >/dev/null 2>&1
}

# If we didn't just push new code and a healthy server is up, reuse it.
if [ "$FORCE_RESTART" != "1" ] && server_running && port_listening; then
    echo "[GO2] Server already running and listening on ${PORT}."
    exit 0
fi

# Otherwise stop any existing server (stale, not listening, or about to be
# replaced by freshly-synced code) and give the camera time to release.
if server_running; then
    echo "[GO2] Stopping existing server process ..."
    "${PRIVILEGE[@]}" pkill -TERM -f "python.*realsense_server.py" 2>/dev/null || true
    # Wait for the USB camera device to be fully released.
    for _ in $(seq 1 30); do
        server_running || break
        sleep 0.5
    done
    if server_running; then
        echo "[ERROR] Old server is still alive; refusing to open a second camera instance." >&2
        exit 1
    fi
    sleep 2   # extra settle so pipeline.start() doesn't hit a busy device
fi

# Auto-detect the script path if not provided.
if [ -z "$SERVER_PATH" ]; then
    SERVER_PATH="$(find "$HOME" -name realsense_server.py 2>/dev/null | head -n1)"
fi

# Expand a leading ~ if the caller passed one.
SERVER_PATH="${SERVER_PATH/#\~/$HOME}"
if [ -z "$SERVER_PATH" ] || [ ! -f "$SERVER_PATH" ]; then
    echo "[GO2][ERROR] realsense_server.py not found. Set SERVER_PATH in start_realsense.sh." >&2
    exit 1
fi

echo "[GO2] Starting server: $SERVER_PATH"
IO_ARGS=(--status-port "$STATUS_PORT")
if [ "$ENABLE_IO" != "true" ]; then IO_ARGS+=(--no-io); fi
nohup "${PRIVILEGE[@]}" "$CAMERA_PYTHON" -u "$SERVER_PATH" \
    --port "$PORT" "${IO_ARGS[@]}" >/tmp/realsense_server.log 2>&1 &
disown || true

# Confirm it actually binds the port; surface the log if it doesn't.
# (create_pipeline retries + a 30-frame settle can take a few seconds.)
for _ in $(seq 1 40); do
    port_listening && { echo "[GO2] Server listening on ${PORT}."; exit 0; }
    # If the process already died, stop waiting and show the log immediately.
    server_running || break
    sleep 0.5
done
echo "[GO2][ERROR] Server did not bind ${PORT}. Last log lines:" >&2
tail -n 20 /tmp/realsense_server.log >&2 2>/dev/null || true
exit 1
REMOTE

# 2. Wait for the port to accept connections (avoid racing the server start).
echo "[LAUNCH] Waiting for ${GO2_IP}:${PORT} ..."
deadline=$(( $(date +%s) + WAIT_TIMEOUT ))
until bash -c "exec 3<>/dev/tcp/${GO2_IP}/${PORT}" 2>/dev/null; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
        echo "[LAUNCH][ERROR] Server did not open ${PORT} within ${WAIT_TIMEOUT}s." >&2
        echo "                Check /tmp/realsense_server.log on the Go2." >&2
        exit 1
    fi
    sleep 0.5
done
exec 3>&- 2>/dev/null || true
echo "[LAUNCH] Server is up."

# 3. Launch the client locally (unless RUN_CLIENT=false — server-only mode).
if [ "$RUN_CLIENT" != "true" ]; then
    echo "[LAUNCH] RUN_CLIENT=false — server is up on ${GO2_IP}:${PORT}; not"
    echo "         starting the viewer. Now launch the detector, e.g.:"
    echo "         ros2 launch go2_perception apriltag_checkpoint_tcp.launch.py host:=${GO2_IP}"
    exit 0
fi

echo "[LAUNCH] Starting client ..."
CLIENT_IO_ARGS=(--status-port "$GO2_STATUS_PORT")
if [ "$GO2_CAMERA_IO" != "true" ]; then CLIENT_IO_ARGS+=(--no-io); fi
exec python3 "${SCRIPT_DIR}/realsense_client.py" --host "${GO2_IP}" \
    --port "${PORT}" "${CLIENT_IO_ARGS[@]}"
