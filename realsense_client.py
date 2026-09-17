#!/usr/bin/env python3
"""
RealSense D435i Stream Client
===============================
Runs on your PC.
Connects to the realsense_server.py host (the machine the camera is plugged
into — the Go2's CPU, or a Raspberry Pi) and displays:
  - Color view (normal camera)
  - Depth view (colorized depth map)
  - Depth with distance info (hover mouse for distance)

Usage:
    python3 realsense_client.py                      # uses GO2_CAMERA_HOST
    python3 realsense_client.py --host 192.168.123.90

Controls:
    q / ESC  — quit
    s        — save current frames as PNG
    d        — toggle depth color map style
"""

import argparse
import csv
import os
import socket
import struct
import time
from datetime import datetime

import cv2
import numpy as np


RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', '..', 'go2_benchmark', 'results')


# ─────────────────────────────────────────────────────────
# Network helpers
# ─────────────────────────────────────────────────────────

def recv_exact(sock, n):
    """Receive exactly n bytes from socket."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 65536))
        if not chunk:
            raise ConnectionError("Server closed connection")
        buf.extend(chunk)
    return bytes(buf)


def recv_frame(sock):
    """Receive one frame packet (4-byte length header + payload)."""
    header = recv_exact(sock, 4)
    length = struct.unpack('>I', header)[0]
    return recv_exact(sock, length)


def clock_sync(sock, rounds=10):
    """Estimate clock offset between client and server.

    Returns offset such that: server_time ≈ client_time + offset
    """
    offsets = []
    for _ in range(rounds):
        t1 = time.time()
        sock.sendall(struct.pack('>d', t1))  # ping
        srv_ts = struct.unpack('>d', recv_exact(sock, 8))[0]  # pong
        t2 = time.time()
        rtt = t2 - t1
        # Estimate: server sent reply at ~midpoint of RTT
        offset = srv_ts - (t1 + rtt / 2.0)
        offsets.append(offset)
    # Use median to reject outliers
    offsets.sort()
    median_offset = offsets[len(offsets) // 2]
    rtt_ms = (t2 - t1) * 1000
    print(f"[SYNC] Clock offset: {median_offset * 1000:.1f} ms, "
          f"last RTT: {rtt_ms:.1f} ms")
    return median_offset


def parse_packet(packet):
    """Parse a packet into color, depth_viz, depth_raw images + timestamps."""
    offset = 0

    # Server timestamps (capture time, encode-done time)
    t_capture, t_encoded = struct.unpack('>dd', packet[offset:offset + 16])
    offset += 16

    # Color JPEG
    color_len = struct.unpack('>I', packet[offset:offset + 4])[0]
    offset += 4
    color_jpg = packet[offset:offset + color_len]
    offset += color_len

    # Depth visualization JPEG
    depth_viz_len = struct.unpack('>I', packet[offset:offset + 4])[0]
    offset += 4
    depth_viz_jpg = packet[offset:offset + depth_viz_len]
    offset += depth_viz_len

    # Raw depth PNG (uint16)
    depth_raw_len = struct.unpack('>I', packet[offset:offset + 4])[0]
    offset += 4
    depth_raw_png = packet[offset:offset + depth_raw_len]
    offset += depth_raw_len

    # Metadata
    h, w = struct.unpack('>HH', packet[offset:offset + 4])

    color_img = cv2.imdecode(
        np.frombuffer(color_jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
    depth_viz = cv2.imdecode(
        np.frombuffer(depth_viz_jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
    depth_raw = cv2.imdecode(
        np.frombuffer(depth_raw_png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

    return color_img, depth_viz, depth_raw, h, w, t_capture, t_encoded


# ─────────────────────────────────────────────────────────
# Display helpers
# ─────────────────────────────────────────────────────────

COLORMAPS = [
    cv2.COLORMAP_JET,
    cv2.COLORMAP_TURBO,
    cv2.COLORMAP_INFERNO,
    cv2.COLORMAP_MAGMA,
    cv2.COLORMAP_VIRIDIS,
]
COLORMAP_NAMES = ['JET', 'TURBO', 'INFERNO', 'MAGMA', 'VIRIDIS']


class DepthViewer:
    """Handles depth visualization with mouse hover distance."""

    def __init__(self):
        self.mouse_x = 0
        self.mouse_y = 0
        self.cmap_idx = 0

    def mouse_cb(self, event, x, y, flags, param):
        self.mouse_x = x
        self.mouse_y = y

    def next_colormap(self):
        self.cmap_idx = (self.cmap_idx + 1) % len(COLORMAPS)

    def render_depth(self, depth_raw, depth_viz):
        """Render depth with custom colormap and distance label."""
        if depth_raw is None:
            return depth_viz

        # Auto-range the colormap per frame between the actual min/max valid
        # depth, so near vs far always span the full colormap (the old fixed
        # 6 m range squashed indoor distances into the bottom third → near and
        # far looked the same color).
        valid = depth_raw[depth_raw > 0]
        if valid.size == 0:
            return depth_viz  # no depth data this frame

        # Robust percentiles mimic the RealSense Viewer histogram and reject
        # a single near/far speckle pixel from collapsing the contrast.
        d_min = float(np.percentile(valid, 2))
        d_max = float(np.percentile(valid, 98))
        if d_max <= d_min:
            d_max = d_min + 1.0  # avoid divide-by-zero

        depth_f = depth_raw.astype(np.float32)
        depth_norm = np.clip((depth_f - d_min) / (d_max - d_min), 0.0, 1.0)
        depth_norm = (depth_norm * 255).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_norm, COLORMAPS[self.cmap_idx])

        # Zero-depth pixels → black
        depth_colored[depth_raw == 0] = [0, 0, 0]

        # Distance text at mouse position
        h, w = depth_raw.shape[:2]
        mx = np.clip(self.mouse_x, 0, w - 1)
        my = np.clip(self.mouse_y, 0, h - 1)
        dist_mm = int(depth_raw[my, mx])
        dist_m = dist_mm / 1000.0

        # Crosshair
        cv2.drawMarker(depth_colored, (mx, my), (255, 255, 255),
                        cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

        # Distance label
        if dist_mm > 0:
            label = f"{dist_m:.2f} m"
            color = (0, 255, 0)
        else:
            label = "No depth"
            color = (0, 0, 255)

        label_x = mx + 15
        label_y = my - 10
        # Keep label on screen
        if label_x + 120 > w:
            label_x = mx - 130
        if label_y < 20:
            label_y = my + 25

        cv2.putText(depth_colored, label, (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(depth_colored, label, (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

        # Colormap name + active auto-range
        cmap_name = COLORMAP_NAMES[self.cmap_idx]
        range_lbl = f"[D] Map: {cmap_name}  {d_min/1000:.1f}-{d_max/1000:.1f} m"
        cv2.putText(depth_colored, range_lbl, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                    cv2.LINE_AA)

        return depth_colored


def add_overlay(img, fps, latency_ms=None, encode_ms=None, net_ms=None):
    """Draw FPS + latency info on top-left."""
    lines = [f"FPS: {fps:.0f}"]
    if latency_ms is not None:
        lines.append(f"Total latency: {latency_ms:.1f} ms")
    if encode_ms is not None:
        lines.append(f"Encode: {encode_ms:.1f} ms")
    if net_ms is not None:
        lines.append(f"Network: {net_ms:.1f} ms")

    y = 24
    for line in lines:
        cv2.putText(img, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        y += 22


def save_frames(color_img, depth_viz, depth_raw):
    """Save current frames to disk."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(f"color_{ts}.png", color_img)
    cv2.imwrite(f"depth_viz_{ts}.png", depth_viz)
    if depth_raw is not None:
        cv2.imwrite(f"depth_raw_{ts}.png", depth_raw)
    print(f"[SAVED] color_{ts}.png, depth_viz_{ts}.png, depth_raw_{ts}.png")


# ─────────────────────────────────────────────────────────
# CSV latency logger
# ─────────────────────────────────────────────────────────

class LatencyLogger:
    """Logs per-frame latency metrics to CSV (same dir as go2_benchmark results)."""

    HEADER = [
        'timestamp', 'elapsed_s', 'frame_num',
        'total_latency_ms', 'encode_ms', 'network_ms',
        'fps', 'packet_bytes', 'width', 'height',
    ]

    def __init__(self):
        os.makedirs(RESULTS_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(RESULTS_DIR, f'camera_{ts}.csv')
        self._f = open(self.path, 'w', newline='')
        self._writer = csv.writer(self._f)
        self._writer.writerow(self.HEADER)
        self._t0 = time.time()
        self._frame = 0
        print(f"[LOG] Recording to {self.path}")

    def log(self, total_ms, encode_ms, net_ms, fps, pkt_bytes, w, h):
        self._frame += 1
        now = time.time()
        self._writer.writerow([
            datetime.now().isoformat(),
            f'{now - self._t0:.3f}',
            self._frame,
            f'{total_ms:.2f}',
            f'{encode_ms:.2f}',
            f'{net_ms:.2f}',
            f'{fps:.1f}',
            pkt_bytes,
            w, h,
        ])
        # Flush every 30 frames so data isn't lost on crash
        if self._frame % 30 == 0:
            self._f.flush()

    def close(self):
        self._f.close()
        print(f"[LOG] Saved {self._frame} frames to {self.path}")
        return self.path


# ─────────────────────────────────────────────────────────
# Main client
# ─────────────────────────────────────────────────────────

def run_client(host, port):
    """Connect to server and display streams."""
    print(f"[CLIENT] Connecting to {host}:{port} ...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)

    try:
        sock.connect((host, port))
    except (ConnectionRefusedError, OSError) as e:
        print(f"[ERROR] Cannot connect to {host}:{port} — {e}")
        print("        Is realsense_server.py running on the Go2?")
        return

    sock.settimeout(None)
    print(f"[CLIENT] Connected!")

    # Clock sync handshake
    clock_offset = clock_sync(sock)

    viewer = DepthViewer()
    logger = LatencyLogger()

    cv2.namedWindow("RealSense Color", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("RealSense Depth", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("RealSense Depth", viewer.mouse_cb)

    fps = 0.0
    frame_count = 0
    t_start = time.time()
    latency_ms = 0.0
    encode_ms = 0.0
    net_ms = 0.0

    try:
        while True:
            packet = recv_frame(sock)
            t_recv = time.time()
            color_img, depth_viz, depth_raw, h, w, t_capture, t_encoded = \
                parse_packet(packet)

            if color_img is None or depth_viz is None:
                continue

            # Latency calculation (using clock offset)
            # Total: capture → client receive
            latency_ms = (t_recv - (t_capture - clock_offset)) * 1000
            # Encode time on server
            encode_ms = (t_encoded - t_capture) * 1000
            # Network: after encoding → client receive
            net_ms = (t_recv - (t_encoded - clock_offset)) * 1000

            # Clamp negatives from clock drift
            latency_ms = max(0.0, latency_ms)
            encode_ms = max(0.0, encode_ms)
            net_ms = max(0.0, net_ms)

            # Log to CSV
            logger.log(latency_ms, encode_ms, net_ms, fps,
                        len(packet), w, h)

            # FPS calculation
            frame_count += 1
            elapsed = time.time() - t_start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                t_start = time.time()

            # Render depth with custom colormap + distance
            depth_display = viewer.render_depth(depth_raw, depth_viz)

            # Add FPS + latency to both views
            add_overlay(color_img, fps, latency_ms, encode_ms, net_ms)
            add_overlay(depth_display, fps, latency_ms, encode_ms, net_ms)

            cv2.imshow("RealSense Color", color_img)
            cv2.imshow("RealSense Depth", depth_display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or ESC
                break
            elif key == ord('s'):
                save_frames(color_img, depth_display, depth_raw)
            elif key == ord('d'):
                viewer.next_colormap()

    except ConnectionError:
        print("[CLIENT] Server disconnected.")
    except KeyboardInterrupt:
        print("\n[CLIENT] Stopped.")
    finally:
        csv_path = logger.close()
        sock.close()
        cv2.destroyAllWindows()

        # Generate visual report
        print("[CLIENT] Generating latency report ...")
        try:
            from visualize_camera_latency import generate_report
            report_path = generate_report(csv_path)
            print(f"[CLIENT] Report: {report_path}")
        except Exception as e:
            print(f"[CLIENT] Could not generate report: {e}")
            print(f"[CLIENT] Run manually: python3 visualize_camera_latency.py {csv_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='RealSense D435i stream client (view on PC)')
    # Defaults mirror camera_env.sh / camera_config.py. The port default used
    # to be 128, which matched nothing: the server listens on 9999, so a manual
    # `realsense_client.py --host <ip>` always failed to connect. It went
    # unnoticed because start_realsense.sh passes --port explicitly.
    parser.add_argument('--host',
                        default=os.environ.get('GO2_CAMERA_HOST',
                                               '192.168.123.90'),
                        help='Address of the host running realsense_server.py')
    parser.add_argument('--port', type=int,
                        default=int(os.environ.get('GO2_CAMERA_PORT', 9999)),
                        help='TCP port (default: 9999)')
    args = parser.parse_args()
    run_client(args.host, args.port)
