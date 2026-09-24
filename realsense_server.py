#!/usr/bin/env python3
"""
RealSense D435i Stream Server
==============================
Runs on the Unitree Go2 docking station.
Captures color + depth frames from Intel RealSense D435i
and streams them to a client PC over TCP socket.
"""

import argparse
import os
import select
import signal
import socket
import struct
import time

import cv2
import numpy as np
import pyrealsense2 as rs

rs.log_to_console(rs.log_severity.debug)

from realsense_io import ButtonStatusServer


def create_pipeline(width=640, height=480, fps=30, retries=4):
    """Configure and start the RealSense D435i pipeline.

    Retries on a busy/just-released camera (common right after the previous
    server process was killed and restarted) instead of crashing.
    """
    last_err = None
    for attempt in range(1, retries + 1):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        try:
            print(f"[STARTUP] Opening camera ({attempt}/{retries}) ...", flush=True)
            profile = pipeline.start(config)
        except RuntimeError as e:
            last_err = e
            print(f"[WARN] pipeline.start failed "
                  f"(attempt {attempt}/{retries}): {e}")
            time.sleep(2.0)  # let the USB device fully release
            continue

        try:
            # Get depth scale for later use
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            print(f"[INFO] Depth scale: {depth_scale:.6f} m/unit")

            # Read calibration before waiting for frames, matching the isolated
            # calibration test. Do not silently swallow 30 ten-second timeouts.
        except BaseException:
            pipeline.stop()
            raise

        return pipeline

    raise RuntimeError(
        f"Could not start RealSense pipeline after {retries} attempts: {last_err}")


def recv_exact(conn, n):
    """Receive exactly n bytes."""
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(min(n - len(buf), 65536))
        if not chunk:
            raise ConnectionError("Client disconnected")
        buf.extend(chunk)
    return bytes(buf)


def send_frame(conn, frame_bytes):
    """Send a single frame: 4-byte length header + payload."""
    header = struct.pack('>I', len(frame_bytes))
    conn.sendall(header + frame_bytes)


def clock_sync_handshake(conn, rounds=10):
    """Clock sync: respond to client ping with server timestamp."""
    for _ in range(rounds):
        # Client sends 8-byte ping (its timestamp)
        recv_exact(conn, 8)
        # Server replies with its own timestamp
        conn.sendall(struct.pack('>d', time.time()))
    print(f"[SERVER] Clock sync done ({rounds} rounds)")


def read_mode_byte(conn, timeout=0.5):
    """Optional post-handshake mode byte (backward compatible).

    New clients may send ONE byte right after clock sync:
        b'C' -> color-only mode: skip the depth-viz JPEG + raw-depth PNG encodes
                entirely (they are ~80% of the per-frame bytes and the PNG is the
                slowest encode on this ARM CPU). The packet keeps the SAME layout
                but with zero-length depth fields, so parsers stay uniform.
    Legacy clients (realsense_client.py) send nothing -> select() times out and
    the server streams the full packet exactly as before.
    """
    ready, _, _ = select.select([conn], [], [], timeout)
    if not ready:
        return b''
    mode = conn.recv(1)
    if mode == b'C':
        print("[SERVER] Client requested COLOR-ONLY mode "
              "(skipping depth encode/send).")
    return mode


def get_color_intrinsics_blob(pipeline):
    """Pack the colour stream intrinsics as 9 big-endian floats:
    fx, fy, ppx, ppy, k1, k2, p1, p2, k3.

    Appended to every frame packet (see serve) so a downstream consumer that
    needs camera calibration — e.g. the AprilTag detector's solvePnP — gets it
    without a separate handshake. Appended at the END of the packet, so the
    existing realsense_client.py viewer (which reads only up to the h,w meta and
    ignores trailing bytes) is unaffected.
    """
    intr = (pipeline.get_active_profile()
            .get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics())
    c = list(intr.coeffs[:5]) + [0.0] * 5
    return struct.pack('>9f', intr.fx, intr.fy, intr.ppx, intr.ppy,
                       c[0], c[1], c[2], c[3], c[4])


def serve(host, port):
    """Main server loop."""
    pipeline = create_pipeline()
    try:
        print("[STARTUP] Reading RGB intrinsics ...", flush=True)
        intrinsics_blob = get_color_intrinsics_blob(pipeline)
        print("[STARTUP] RGB intrinsics ready.", flush=True)
        warm_up_camera(pipeline)
        _serve_pipeline(host, port, pipeline, intrinsics_blob)
    finally:
        pipeline.stop()


def warm_up_camera(pipeline, target_frames=30, timeout_seconds=10):
    """Require 30 valid RGB/depth pairs within a bounded warm-up window."""
    print(f"[STARTUP] Waiting for {target_frames} RGB/depth frames ...", flush=True)
    deadline = time.monotonic() + timeout_seconds
    count = 0
    last_error = "No complete RGB/depth frames"
    while count < target_frames:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(
                f"Camera warm-up timed out: {count}/{target_frames} valid frames "
                f"in {timeout_seconds}s. Last error: {last_error}")
        try:
            frames = pipeline.wait_for_frames(max(1, min(1000, int(remaining * 1000))))
        except RuntimeError as exc:
            last_error = str(exc)
            print(f"[STARTUP][WARN] Frame wait: {exc}", flush=True)
            continue
        if frames.get_color_frame() and frames.get_depth_frame():
            count += 1
    print(f"[STARTUP] {count} RGB/depth frames received.", flush=True)


def _serve_pipeline(host, port, pipeline, intrinsics_blob):
    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen(1)
    print(f"[SERVER] Listening on {host}:{port} ...")
    print("[SERVER] Waiting for client connection ...")

    try:
        while True:
            conn, addr = server_sock.accept()
            print(f"[SERVER] Client connected: {addr}")

            try:
                # Clock sync handshake
                clock_sync_handshake(conn)
                # Optional mode byte (b'C' = color-only). Legacy clients send
                # nothing and get the full packet as before.
                color_only = (read_mode_byte(conn) == b'C')

                while True:
                    try:
                        frames = pipeline.wait_for_frames(5000)
                    except RuntimeError as e:
                        # Frame didn't arrive in time — log and keep going
                        # instead of crashing the whole server.
                        print(f"[SERVER] Frame wait failed: {e}; retrying")
                        continue
                    aligned = align.process(frames)

                    color_frame = aligned.get_color_frame()
                    depth_frame = aligned.get_depth_frame()

                    if not color_frame or not depth_frame:
                        continue

                    t_capture = time.time()

                    # Color image (BGR)
                    color_img = np.asanyarray(color_frame.get_data())

                    # Encode as JPEG for efficient transfer
                    _, color_jpg = cv2.imencode(
                        '.jpg', color_img,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])

                    if color_only:
                        # Color-only mode: no depth encode at all. The uint16
                        # PNG below is the slowest step on this CPU and ~80% of
                        # the packet bytes — skipping it is the whole point.
                        depth_viz_bytes = b''
                        depth_raw_bytes = b''
                    else:
                        # Depth image — colorized for visualization
                        depth_colorized = np.asanyarray(
                            colorizer.colorize(depth_frame).get_data())
                        # Raw depth (uint16, millimeters) for client-side use
                        depth_raw = np.asanyarray(depth_frame.get_data())
                        _, depth_viz_jpg = cv2.imencode(
                            '.jpg', depth_colorized,
                            [cv2.IMWRITE_JPEG_QUALITY, 80])
                        # Compress raw depth with PNG (lossless for uint16)
                        _, depth_raw_png = cv2.imencode('.png', depth_raw)
                        depth_viz_bytes = depth_viz_jpg.tobytes()
                        depth_raw_bytes = depth_raw_png.tobytes()

                    t_encoded = time.time()

                    # Build packet:
                    #   [t_capture(8d)] [t_encoded(8d)]
                    #   [color_len(4)] [color_jpg]
                    #   [depth_viz_len(4)] [depth_viz_jpg]
                    #   [depth_raw_len(4)] [depth_raw_png]
                    #   [height(2)] [width(2)]
                    h, w = color_img.shape[:2]
                    meta = struct.pack('>HH', h, w)
                    timestamps = struct.pack('>dd', t_capture, t_encoded)

                    packet = (
                        timestamps
                        + struct.pack('>I', len(color_jpg)) + color_jpg.tobytes()
                        + struct.pack('>I', len(depth_viz_bytes)) + depth_viz_bytes
                        + struct.pack('>I', len(depth_raw_bytes)) + depth_raw_bytes
                        + meta
                        # Colour intrinsics (9 floats, 36 B) appended LAST so the
                        # legacy viewer ignores them and the AprilTag detector can
                        # read them as packet[-36:]. See get_color_intrinsics_blob.
                        + intrinsics_blob
                    )

                    # Send total packet size first, then the packet
                    send_frame(conn, packet)

            except (ConnectionResetError, BrokenPipeError, ConnectionError):
                print(f"[SERVER] Client {addr} disconnected.")
            except Exception as e:
                # Any other error: log full traceback but keep the server
                # alive so the listening socket survives and the next client
                # (or a client reconnect) still works.
                import traceback
                print(f"[SERVER] Unexpected error on {addr}: {e}")
                traceback.print_exc()
            finally:
                conn.close()

    except KeyboardInterrupt:
        print("\n[SERVER] Shutting down ...")
    finally:
        server_sock.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='RealSense D435i stream server (Go2 docking station)')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Bind address (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=9999,
                        help='TCP port (default: 9999)')
    parser.add_argument('--status-port', type=int,
                        default=int(os.environ.get('GO2_STATUS_PORT', 8765)),
                        help='Independent button/LED WebSocket port (8765)')
    parser.add_argument('--no-io', action='store_true',
                        help='Camera only, for hosts without Raspberry Pi GPIO')
    args = parser.parse_args()

    def shutdown_requested(signum, frame):
        # TERM from the SSH stop script must run the same cleanup as Ctrl+C.
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown_requested)
    io_status = None
    try:
        if not args.no_io:
            io_status = ButtonStatusServer(args.host, args.status_port)
            io_status.start()
        serve(args.host, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        # A second signal must not interrupt LED/GPIO cleanup.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if io_status is not None:
            io_status.close()
