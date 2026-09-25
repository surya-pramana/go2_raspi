"""Research CSV and optional packet telemetry; all missing measurements stay blank.

row_type=frame is required when aggregating frame metrics. event/sample rows
retain evidence during freezes. Latencies are software timings, not photon-to-screen.
"""
import csv
import json
import os
import platform
import struct
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path

MAGIC = b'RSM1'


def encode_metrics(metrics):
    data = json.dumps(metrics, allow_nan=False, separators=(',', ':')).encode()
    return MAGIC + struct.pack('>I', len(data)) + data


def decode_metrics(packet):
    """Optional trailer after image dimensions, before the final 36B intrinsics.

    Old viewers ignore this section; consumers reading packet[-36:] still work.
    """
    offset = 16
    for _ in range(3):
        if offset + 4 > len(packet):
            raise ValueError('Truncated image length')
        size = struct.unpack_from('>I', packet, offset)[0]
        offset += 4 + size
        if offset > len(packet):
            raise ValueError('Truncated image payload')
    offset += 4  # height, width
    if offset > len(packet):
        raise ValueError('Truncated image dimensions')
    if packet[offset:offset + 4] != MAGIC:
        return {}  # Legacy server.
    if offset + 8 > len(packet):
        raise ValueError('Truncated telemetry header')
    size = struct.unpack_from('>I', packet, offset + 4)[0]
    if size > 65536 or offset + 8 + size + 36 != len(packet):
        raise ValueError('Invalid telemetry length')
    result = json.loads(packet[offset + 8:offset + 8 + size])
    if not isinstance(result, dict):
        raise ValueError('Invalid telemetry object')
    return result


class ResourceSampler:
    """Cached 1 Hz Linux readings; process CPU uses 100% per busy CPU core."""
    def __init__(self):
        self.last_time = time.monotonic()
        self.last_cpu = time.process_time()
        self.last_system = self._system_cpu()
        self.cached = {}

    @staticmethod
    def _system_cpu():
        try:
            fields = Path('/proc/stat').read_text().splitlines()[0].split()[1:9]
            values = list(map(int, fields))
            return sum(values), values[3] + values[4]
        except (OSError, ValueError, IndexError):
            return None

    def sample(self):
        now = time.monotonic()
        if self.cached and now - self.last_time < 1:
            return dict(self.cached, pi_resource_age_ms=(now - self.last_time) * 1000)
        elapsed = now - self.last_time
        cpu = time.process_time()
        result = {'pi_process_cpu_pct': ((cpu - self.last_cpu) / elapsed * 100
                                        if elapsed >= 0.5 else None)}
        current = self._system_cpu()
        result['pi_system_cpu_pct'] = None
        if current and self.last_system:
            total = current[0] - self.last_system[0]
            idle = current[1] - self.last_system[1]
            if total > 0:
                result['pi_system_cpu_pct'] = 100 * (1 - idle / total)
        try:
            for line in Path('/proc/self/status').read_text().splitlines():
                if line.startswith('VmRSS:'):
                    result['pi_rss_mb'] = int(line.split()[1]) / 1024
        except (OSError, ValueError):
            pass
        try:
            result['pi_temp_c'] = float(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000
        except (OSError, ValueError):
            pass
        self.cached = result
        self.last_time, self.last_cpu, self.last_system = now, cpu, current
        return dict(result, pi_resource_age_ms=0)


class ResearchLogger:
    LEGACY = ['timestamp', 'elapsed_s', 'frame_num', 'total_latency_ms',
              'encode_ms', 'network_ms', 'fps', 'packet_bytes', 'width', 'height']
    EXTRA = ['row_type', 'event', 'detail', 'schema_version', 'run_id',
             'host', 'test_label', 'network_mode', 'freeze_threshold_ms',
             'recv_monotonic_ns', 'recv_interval_ms', 'receive_fps',
             'payload_mbps', 'decode_ms', 'queue_wait_ms', 'client_processing_ms',
             'receive_to_gui_submit_ms', 'raw_total_latency_ms', 'raw_network_ms',
             'clock_offset_ms', 'sync_rtt_median_ms', 'sync_rtt_min_ms',
             'clock_sync_age_s', 'color_frame_id', 'depth_frame_id',
             'color_timestamp_ms', 'depth_timestamp_ms', 'timestamp_domain',
             'color_id_gap', 'depth_id_gap', 'color_id_reset', 'depth_id_reset',
             'pi_frame_interval_ms', 'align_ms', 'configured_fps', 'depth_scale',
             'pi_process_cpu_pct', 'pi_system_cpu_pct', 'pi_rss_mb', 'pi_temp_c',
             'pi_resource_age_ms', 'resource_sample_id', 'camera_model', 'camera_serial', 'camera_firmware',
             'pi_os', 'pi_kernel', 'pi_python', 'sdk_version', 'metadata_json']
    HEADER = LEGACY + EXTRA

    def __init__(self, directory, **context):
        os.makedirs(directory, exist_ok=True)
        self.run_id = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8]
        self.path = str(Path(directory) / f'camera_{self.run_id}.csv')
        self.file = open(self.path, 'x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=self.HEADER, extrasaction='ignore')
        self.writer.writeheader()
        self.file.flush()
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.frames = 0
        self.context = context
        self.previous_ids = {}
        self.closed = False
        self.event('run_start', json.dumps({'client_os': platform.platform(),
                                         'client_python': platform.python_version()}))
        print(f'[LOG] Recording to {self.path}', flush=True)

    def write(self, row_type, **values):
        with self.lock:
            if self.closed:
                return
            row = dict(self.context)
            row.update(values)
            row.update(timestamp=datetime.now().astimezone().isoformat(),
                       elapsed_s=round(time.monotonic() - self.started, 6),
                       row_type=row_type, schema_version=2, run_id=self.run_id)
            if row_type == 'frame':
                self.frames += 1
                row['frame_num'] = self.frames
                for stream in ('color', 'depth'):
                    value = row.get(f'{stream}_frame_id')
                    if value is not None:
                        previous = self.previous_ids.get(stream)
                        if previous is not None:
                            row[f'{stream}_id_gap'] = max(0, value - previous - 1)
                            row[f'{stream}_id_reset'] = value < previous
                        self.previous_ids[stream] = value
            self.writer.writerow(row)
            # Also persist low FPS / error-only runs, not only every 30 frames.
            self.file.flush()

    def event(self, name, detail='', **values):
        self.write('event', event=name, detail=detail, **values)

    def close(self):
        if not self.closed:
            self.event('run_end', f'frames_logged={self.frames}')
            with self.lock:
                self.closed = True
                self.file.close()
        return self.path
