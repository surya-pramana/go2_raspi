#!/usr/bin/env python3
"""
RealSense D435i Stream Server
==============================
Runs on the Unitree Go2 docking station.
Captures RGB frames only from Intel RealSense D435i
and streams them to a client PC over TCP socket.
"""

import argparse
import json
import os
import select
import signal
import socket
import struct
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs

#rs.log_to_console(rs.log_severity.debug)

from realsense_io import ButtonStatusServer


def create_pipeline(width=640, height=480, fps=30):
    """Start RGB only. Fail once; do not reset/retry a failed native SDK session."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    print("[STARTUP] Opening RGB camera ...", flush=True)
    pipeline.start(config)
    print("[STARTUP] RGB pipeline started.", flush=True)
    return pipeline


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

    Consume the legacy b'C' request if present. This branch always sends RGB
    with empty depth fields, whether or not the client sends this byte.
    """
    ready, _, _ = select.select([conn], [], [], timeout)
    if not ready:
        return b''
    mode = conn.recv(1)
    if mode == b'C':
        print("[SERVER] Client requested COLOR-ONLY mode "
              "(skipping depth encode/send).")
    return mode


def read_rgb_calibration(pipeline):
    """Read the actual active profile; never substitute guessed calibration."""
    profile = pipeline.get_active_profile()
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    values = [intr.fx, intr.fy, intr.ppx, intr.ppy, *intr.coeffs]
    if not np.isfinite(values).all() or intr.fx <= 0 or intr.fy <= 0:
        raise RuntimeError("Invalid RGB calibration")
    return dict(width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy,
                ppx=intr.ppx, ppy=intr.ppy, coeffs=list(intr.coeffs),
                model=str(intr.model), serial=profile.get_device().get_info(rs.camera_info.serial_number),
                stream='color', pixel_format='bgr8', protocol_version=2)


class CameraCaptureError(RuntimeError):
    """Fatal capture failure, distinct from a disconnected network client."""


class LatestRGB:
    """One-slot buffer: camera reads never wait for TCP or JPEG encoding."""

    def __init__(self, pipeline, failure_seconds=10):
        self.pipeline = pipeline
        self.failure_seconds = failure_seconds
        self.stop_event = threading.Event()
        self.condition = threading.Condition()
        self.latest = None
        self.sequence = 0
        self.error = None
        self.thread = threading.Thread(target=self._capture, name='rgb-capture')

    def start(self):
        self.thread.start()

    def _capture(self):
        last_frame = time.monotonic()
        try:
            while not self.stop_event.is_set():
                try:
                    frames = self.pipeline.wait_for_frames(1000)
                except RuntimeError as exc:
                    if self.stop_event.is_set():
                        break
                    if time.monotonic() - last_frame >= self.failure_seconds:
                        raise CameraCaptureError(f'No RGB frames for {self.failure_seconds}s: {exc}') from exc
                    continue
                color = frames.get_color_frame()
                if not color:
                    if time.monotonic() - last_frame >= self.failure_seconds:
                        raise CameraCaptureError('Frames received without RGB')
                    continue
                captured_at = time.time()
                # Own the pixels: no SDK frame references escape this thread.
                image = np.asanyarray(color.get_data()).copy()
                del color, frames
                last_frame = time.monotonic()
                with self.condition:
                    self.sequence += 1
                    self.latest = (self.sequence, image, captured_at, last_frame)
                    self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
            print(f'[CAMERA][ERROR] {exc}', flush=True)
        finally:
            self.stop_event.set()
            with self.condition:
                self.condition.notify_all()

    def check(self):
        with self.condition:
            if self.error is not None:
                raise CameraCaptureError(self.error)
            if self.stop_event.is_set():
                raise CameraCaptureError('Capture stopped')

    def next_frame(self, previous_sequence, timeout=1):
        with self.condition:
            self.condition.wait_for(lambda: self.sequence > previous_sequence
                                    or self.stop_event.is_set(), timeout)
            self.check()
            if self.latest is None or self.sequence <= previous_sequence:
                return None
            # Do not send an old cached image to a newly connected client.
            if time.monotonic() - self.latest[3] > 2:
                return None
            return self.latest[:3]

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=7)
        if self.thread.is_alive():
            raise CameraCaptureError('Capture thread still inside SDK; pipeline.stop was not called concurrently. '
                                     'Check the process before restarting.')


def serve(host, port):
    pipeline = create_pipeline()
    capture = None
    try:
        warm_up_camera(pipeline)
        print("[STARTUP] Reading RGB calibration ...", flush=True)
        calibration = read_rgb_calibration(pipeline)
        print(f"[CALIBRATION] {json.dumps(calibration)}", flush=True)
        capture = LatestRGB(pipeline)
        capture.start()
        print('[CAMERA] Continuous RGB capture thread started.', flush=True)
        _serve_pipeline(host, port, capture, calibration)
    finally:
        # Never stop the SDK while the capture thread is reading it.
        if capture is not None:
            capture.close()
        print("[SHUTDOWN] Stopping RGB pipeline ...", flush=True)
        try:
            pipeline.stop()
            print("[SHUTDOWN] RGB pipeline stopped.", flush=True)
        except RuntimeError as exc:
            print(f"[SHUTDOWN][ERROR] Camera stop failed: {exc}", flush=True)


def warm_up_camera(pipeline, target_frames=30, timeout_seconds=10):
    """Require 30 valid RGB frames within a bounded warm-up window."""
    print(f"[STARTUP] Waiting for {target_frames} RGB frames ...", flush=True)
    deadline = time.monotonic() + timeout_seconds
    count = 0
    last_error = "No complete RGB frames"
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
        if frames.get_color_frame():
            count += 1
    print(f"[STARTUP] {count} RGB frames received.", flush=True)


def _serve_pipeline(host, port, capture, calibration):
    # Versioned metadata precedes the legacy 36-byte intrinsics trailer.
    metadata = json.dumps(calibration, allow_nan=False).encode('utf-8')
    calibration_blob = b'RGB2' + struct.pack('>I', len(metadata)) + metadata
    intrinsics_blob = struct.pack('>9f', calibration['fx'], calibration['fy'],
                                 calibration['ppx'], calibration['ppy'], *calibration['coeffs'])

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen(1)
    server_sock.settimeout(1)
    print(f"[SERVER] Listening on {host}:{port} ...")
    print("[SERVER] Waiting for client connection ...")

    try:
        while True:
            capture.check()
            try:
                conn, addr = server_sock.accept()
            except socket.timeout:
                continue
            print(f"[SERVER] Client connected: {addr}")

            try:
                conn.settimeout(10)
                # Clock sync handshake
                clock_sync_handshake(conn)
                # Consume an optional legacy color-only request.
                read_mode_byte(conn)  # Retain the existing optional client handshake.

                sequence = 0
                while True:
                    item = capture.next_frame(sequence)
                    if item is None:
                        continue
                    sequence, color_img, t_capture = item

                    # Encode as JPEG for efficient transfer
                    ok, color_jpg = cv2.imencode(
                        '.jpg', color_img,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])

                    if not ok:
                        raise RuntimeError("JPEG encoding failed")
                    if color_img.shape[:2] != (calibration['height'], calibration['width']):
                        raise RuntimeError("Frame dimensions differ from calibration")
                    depth_viz_bytes = b''
                    depth_raw_bytes = b''

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
                        # Full model/resolution metadata, then the legacy trailer.
                        + calibration_blob
                        + intrinsics_blob
                    )

                    # Send total packet size first, then the packet
                    send_frame(conn, packet)

            except CameraCaptureError:
                raise
            except (ConnectionResetError, BrokenPipeError, ConnectionError, socket.timeout):
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
