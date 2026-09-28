#!/usr/bin/env bash
# Run the existing diagnostic with a bounded lifetime on Linux/Raspberry Pi.
# Invoke this wrapper with the same privileges/environment as the diagnostic.
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_PYTHON="${CAMERA_PYTHON:-${SCRIPT_DIR}/.venv/bin/python3}"
TEST_TIMEOUT_SECONDS="${TEST_TIMEOUT_SECONDS:-120}"
STOP_GRACE_SECONDS="${STOP_GRACE_SECONDS:-30}"

for duration in "$TEST_TIMEOUT_SECONDS" "$STOP_GRACE_SECONDS"; do
    if [[ ! "$duration" =~ ^[1-9][0-9]*$ ]]; then
        echo '[ERROR] Timeout and grace must be positive integer seconds.' >&2
        exit 2
    fi
done
if [[ ! -x "$CAMERA_PYTHON" ]]; then
    echo "[ERROR] Python is not executable: $CAMERA_PYTHON" >&2
    exit 2
fi
if ! command -v timeout >/dev/null 2>&1; then
    echo '[ERROR] GNU timeout is required (Ubuntu coreutils).' >&2
    exit 2
fi

mkdir -p "${SCRIPT_DIR}/debug_logs"
RUN_LOG="${SCRIPT_DIR}/debug_logs/$(date +%Y%m%d_%H%M%S)_$$_supervisor.log"
echo "[WATCHDOG] Log: $RUN_LOG"
echo "[WATCHDOG] Total test limit: ${TEST_TIMEOUT_SECONDS}s; cleanup grace: ${STOP_GRACE_SECONDS}s."
echo '[WATCHDOG] Timeout sends TERM, then KILL if the process remains alive.'
echo '[WATCHDOG] This does not cut USB power or guarantee camera recovery.'

# Python's SIGTERM handler raises KeyboardInterrupt, allowing the diagnostic's
# finally block to call pipeline.stop(). GNU timeout is a separate process, so
# it can still send KILL if a native SDK call prevents Python handling TERM.
# Only this invocation is targeted; no broad pkill pattern is used.
timeout --verbose --signal=TERM --kill-after="${STOP_GRACE_SECONDS}s" \
    "${TEST_TIMEOUT_SECONDS}s" "$CAMERA_PYTHON" -u -c '
import runpy
import signal
import sys

def stop_requested(signum, frame):
    # Let cleanup finish without a repeated Ctrl+C interrupting it.
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    raise KeyboardInterrupt("Diagnostic stop requested")

signal.signal(signal.SIGTERM, stop_requested)
signal.signal(signal.SIGINT, stop_requested)
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")
' "${SCRIPT_DIR}/debug_realsense.py" "$@" 2>&1 | tee "$RUN_LOG"
RESULT=${PIPESTATUS[0]}
case "$RESULT" in
    0) MESSAGE='Diagnostic completed successfully; inspect SDK log for internal errors.' ;;
    124) MESSAGE='TIMEOUT: diagnostic exceeded the total test limit (not PASS).' ;;
    137) MESSAGE='Process was killed (SIGKILL); cleanup and USB recovery are not guaranteed.' ;;
    *) MESSAGE="Diagnostic failed/interrupted: exit $RESULT." ;;
esac
echo "[WATCHDOG] $MESSAGE" | tee -a "$RUN_LOG"
exit "$RESULT"
