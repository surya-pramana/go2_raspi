import csv
import json
import socket
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import realsense_client as client
from research_metrics import ResearchLogger, ResourceSampler, decode_metrics, encode_metrics


def packet(metrics=None):
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    depth = np.full((8, 8), 1234, dtype=np.uint16)
    jpg = cv2.imencode('.jpg', rgb)[1].tobytes()
    png = cv2.imencode('.png', depth)[1].tobytes()
    payload = struct.pack('>dd', time.time(), time.time())
    for data in (jpg, jpg, png):
        payload += struct.pack('>I', len(data)) + data
    payload += struct.pack('>HH', 8, 8)
    if metrics is not None:
        payload += encode_metrics(metrics)
    return payload + struct.pack('>9f', *range(9))


class ResearchTests(unittest.TestCase):
    def test_wire_backward_compatibility(self):
        legacy = packet()
        extended = packet({'color_frame_id': 42, 'depth_scale': .001})
        self.assertEqual(decode_metrics(legacy), {})
        self.assertEqual(decode_metrics(extended)['color_frame_id'], 42)
        self.assertEqual(legacy[-36:], extended[-36:])
        np.testing.assert_array_equal(client.parse_packet(legacy)[2],
                                      client.parse_packet(extended)[2])
        with self.assertRaises(ValueError):
            decode_metrics(extended[:-1])

    def test_csv_rows_flush_gaps_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = ResearchLogger(directory, test_label='trial,1')
            logger.write('frame', color_frame_id=10, raw_network_ms=-2.5)
            logger.write('frame', color_frame_id=13)
            logger.write('frame', color_frame_id=2)
            logger.event('tcp_disconnected', 'error, with comma\nand newline')
            # Read while still running: every row is flushed.
            with open(logger.path, newline='', encoding='utf-8') as handle:
                rows = list(csv.DictReader(handle))
            frames = [r for r in rows if r['row_type'] == 'frame']
            self.assertEqual(frames[0]['fps'], '')
            self.assertEqual(frames[0]['raw_network_ms'], '-2.5')
            self.assertEqual(frames[1]['color_id_gap'], '2')
            self.assertEqual(frames[2]['color_id_reset'], 'True')
            logger.close()
            other = ResearchLogger(directory)
            self.assertNotEqual(other.path, logger.path)
            other.close()

    def test_linux_resources_missing_values_and_cache(self):
        def read(path):
            return {'/proc/stat': 'cpu 10 0 5 80 5 0 0 0',
                    '/proc/self/status': 'VmRSS: 2048 kB',
                    '/sys/class/thermal/thermal_zone0/temp': '45000'}[path.as_posix()]
        with patch.object(Path, 'read_text', read), \
                patch('research_metrics.time.monotonic', side_effect=[0, 1, 1.2]), \
                patch('research_metrics.time.process_time', side_effect=[0, .5]):
            sampler = ResourceSampler()
            first = sampler.sample()
            cached = sampler.sample()
        self.assertEqual(first['pi_process_cpu_pct'], 50)
        self.assertEqual(first['pi_rss_mb'], 2)
        self.assertEqual(first['pi_temp_c'], 45)
        self.assertAlmostEqual(cached['pi_resource_age_ms'], 200)

    def test_client_records_real_tcp_frames_and_disconnect(self):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen()
        port = listener.getsockname()[1]
        failures = []

        def server():
            try:
                with listener:
                    conn, _ = listener.accept()
                    with conn:
                        for _ in range(10):
                            client.recv_exact(conn, 8)
                            conn.sendall(struct.pack('>d', time.time()))
                        time.sleep(1.15)  # Record zero-rate sample and freeze before video.
                        for frame_id in (100, 102):
                            data = packet({'color_frame_id': frame_id, 'depth_frame_id': frame_id,
                                           'configured_fps': 30, 'pi_temp_c': 42})
                            conn.sendall(struct.pack('>I', len(data)) + data)
                            time.sleep(.1)
            except Exception as exc:
                failures.append(exc)

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(client, 'RESULTS_DIR', directory), \
                patch.object(cv2, 'namedWindow'), patch.object(cv2, 'setMouseCallback'), \
                patch.object(cv2, 'imshow'), patch.object(cv2, 'waitKey', return_value=-1), \
                patch.object(cv2, 'destroyAllWindows'):
            client.run_client('127.0.0.1', port, no_io=True, test_label='synthetic', freeze_ms=200)
            files = list(Path(directory).glob('*.csv'))
            self.assertEqual(len(files), 1)
            with files[0].open(newline='', encoding='utf-8') as handle:
                rows = list(csv.DictReader(handle))
            frames = [r for r in rows if r['row_type'] == 'frame']
            self.assertEqual(len(frames), 2)
            self.assertEqual(frames[1]['color_id_gap'], '1')
            self.assertGreater(float(frames[1]['recv_interval_ms']), 0)
            self.assertGreaterEqual(float(frames[0]['decode_ms']), 0)
            self.assertEqual(frames[0]['pi_temp_c'], '42')
            self.assertIn('tcp_disconnected', [r['event'] for r in rows])
            self.assertIn('video_freeze_start', [r['event'] for r in rows])
            self.assertIn('video_freeze_end', [r['event'] for r in rows])
            self.assertTrue(any(r['row_type'] == 'sample' and float(r['receive_fps']) == 0
                                for r in rows))
            self.assertEqual(rows[-1]['event'], 'run_end')
        thread.join(timeout=2)
        self.assertFalse(failures)


if __name__ == '__main__':
    unittest.main()
