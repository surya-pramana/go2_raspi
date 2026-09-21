# camera_env.sh — where the RealSense camera host lives.
#
# Sourced by start_realsense.sh / stop_realsense.sh / status_realsense.sh so
# the three agree by construction. They used to carry their own copies of the
# address and the SSH user, which drifted the moment the camera moved off the
# Go2: start_ pointed at the new host while stop_ and status_ still talked to
# the old one, so "stop" quietly did nothing and "status" always read dead.
#
# The camera host is whatever machine the RealSense is physically plugged into
# and where realsense_server.py runs — the Go2's own CPU originally, a
# Raspberry Pi once the camera is wired to one instead. Nothing here assumes
# it is the robot.
#
# Every value is env-overridable, so a one-off run needs no edit:
#   GO2_CAMERA_HOST=192.168.123.50 ./start_realsense.sh
#
# The same GO2_CAMERA_* names are read by the Python side (camera_config.py and
# the backend's camera_runner.py), so exporting one variable moves the whole
# pipeline — scripts, ROS nodes and the operator GUI — to a new camera host.

# IPv4 address of the machine running realsense_server.py.
GO2_CAMERA_HOST="${GO2_CAMERA_HOST:-192.168.123.90}"

# SSH login on that machine ('unitree' on a Go2, typically 'pi' on a Pi OS
# image; this deployment uses a dedicated account).
GO2_CAMERA_USER="${GO2_CAMERA_USER:-sf-system}"

# TCP port realsense_server.py listens on. 9999 avoids the low-numbered ports,
# which would need root to bind on the camera host.
GO2_CAMERA_PORT="${GO2_CAMERA_PORT:-9999}"

# Where realsense_server.py is deployed ON the camera host. start_realsense.sh
# scp's the local copy here before launching it.
GO2_CAMERA_SERVER_PATH="${GO2_CAMERA_SERVER_PATH:-~/src/camera/realsense_server.py}"

# NC GPIO17 + 30 NeoPixels GPIO18, independent WebSocket status channel.
GO2_STATUS_PORT="${GO2_STATUS_PORT:-8765}"
GO2_CAMERA_IO="${GO2_CAMERA_IO:-true}"
# Absolute interpreter path on the Pi, e.g. /home/sf-system/neopixel-test/.venv/bin/python3
GO2_CAMERA_PYTHON="${GO2_CAMERA_PYTHON:-python3}"
# Enable only when the existing NeoPixel driver requires root; sudo -n must work.
GO2_CAMERA_SUDO="${GO2_CAMERA_SUDO:-false}"
