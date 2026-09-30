import json
import struct
import unittest
import time
import threading
import tempfile
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock, patch
import cv2
import numpy as np
from apriltag_pose import AprilTagPose, camera_parameters
from realsense_client import parse_packet


CAL = dict(width=640, height=480, fx=600., fy=600., ppx=320., ppy=240.,
           model='distortion.inverse_brown_conrady', coeffs=[0.] * 5)


class RGBPoseTests(unittest.TestCase):
    def test_saved_calibration_skips_intrinsics_and_stops(self):
        server = self.load_server()
        path = Path(__file__).with_name('calibration_346522076825_rgb_640x480.json')
        pipeline = MagicMock()
        active = pipeline.get_active_profile.return_value
        active.get_device.return_value.get_info.return_value = '346522076825'
        profile = active.get_stream.return_value.as_video_stream_profile.return_value
        profile.width.return_value = 640
        profile.height.return_value = 480
        profile.fps.return_value = 30
        profile.format.return_value = server.rs.format.bgr8
        with patch.object(server, 'create_pipeline', return_value=pipeline) as create, \
             patch.object(server, 'warm_up_camera'), \
             patch.object(server, 'read_rgb_calibration') as read, \
             patch.object(server, 'LatestRGB') as capture, \
             patch.object(server, '_serve_pipeline') as network:
            server.serve('localhost', 9999, str(path))
        create.assert_called_once_with(serial='346522076825')
        read.assert_not_called()
        profile.get_intrinsics.assert_not_called()
        capture.return_value.start.assert_called_once()
        capture.return_value.close.assert_called_once()
        pipeline.stop.assert_called_once()
        self.assertEqual(network.call_args.args[3]['calibration_source'], 'file')
        active.get_device.return_value.get_info.return_value = 'other-camera'
        with self.assertRaisesRegex(ValueError, 'serial mismatch'):
            server.verify_saved_profile(pipeline, server.load_rgb_calibration(path))
        active.get_device.return_value.get_info.return_value = '346522076825'
        profile.width.return_value = 1280
        with self.assertRaisesRegex(ValueError, 'profile'):
            server.verify_saved_profile(pipeline, server.load_rgb_calibration(path))

    def test_invalid_saved_calibration(self):
        server = self.load_server()
        original = json.loads(Path(__file__).with_name(
            'calibration_346522076825_rgb_640x480.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.json'
            for changes in ({'fx': 0}, {'fx': float('nan')}, {'serial': ''},
                            {'width': 1280}, {'fps': 15}, {'coeffs': [0]},
                            {'coeffs': [.1, 0, 0, 0, 0]}, {'model': 'unknown'}):
                with self.subTest(changes=changes):
                    path.write_text(json.dumps(dict(original, **changes)))
                    with self.assertRaises(ValueError):
                        server.load_rgb_calibration(path)

    def load_server(self):
        spec = importlib.util.spec_from_file_location('capture_test_server',
                    Path(__file__).with_name('realsense_server.py'))
        server = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', {'pyrealsense2': MagicMock()}):
            spec.loader.exec_module(server)
        return server

    def test_capture_runs_without_consumer_and_skips_old_frames(self):
        server = self.load_server()
        pipeline = MagicMock()
        pixels = np.zeros((2, 2, 3), np.uint8)
        read_threads = []
        def read(timeout):
            time.sleep(.005)
            read_threads.append(threading.current_thread().name)
            pixels[:] = len(read_threads) % 255
            frame = MagicMock()
            frame.get_color_frame.return_value.get_data.return_value = pixels
            return frame
        pipeline.wait_for_frames.side_effect = read
        capture = server.LatestRGB(pipeline)
        capture.start()
        try:
            first = capture.next_frame(0)
            snapshot = first[1].copy()
            with capture.condition:
                self.assertTrue(capture.condition.wait_for(
                    lambda: capture.sequence >= first[0] + 5, timeout=2))
            latest = capture.next_frame(first[0])
            self.assertGreaterEqual(latest[0], first[0] + 5)
            np.testing.assert_array_equal(first[1], snapshot)
            self.assertEqual(set(read_threads), {'rgb-capture'})
        finally:
            capture.close()
        self.assertFalse(capture.thread.is_alive())
        pipeline.stop.assert_not_called()

    def test_capture_failure_wakes_consumer(self):
        server = self.load_server()
        pipeline = MagicMock()
        pipeline.wait_for_frames.side_effect = RuntimeError('USB timeout')
        capture = server.LatestRGB(pipeline, failure_seconds=0)
        capture.start()
        try:
            with self.assertRaisesRegex(server.CameraCaptureError, 'USB timeout'):
                capture.next_frame(0)
        finally:
            capture.close()

    def test_server_rgb_only_and_cleanup(self):
        sdk = MagicMock()
        spec = importlib.util.spec_from_file_location('rgb_test_server',
                    Path(__file__).with_name('realsense_server.py'))
        server = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', {'pyrealsense2': sdk}):
            spec.loader.exec_module(server)
        pipeline = server.create_pipeline()
        sdk.config.return_value.enable_stream.assert_called_once_with(
            sdk.stream.color, 640, 480, sdk.format.bgr8, 30)
        with patch.object(server, 'create_pipeline', return_value=pipeline), \
             patch.object(server, 'warm_up_camera'), \
             patch.object(server, 'read_rgb_calibration', side_effect=RuntimeError('calibration failed')):
            with self.assertRaises(RuntimeError):
                server.serve('127.0.0.1', 9999)
        pipeline.stop.assert_called_once()
        sdk.align.assert_not_called()

    def test_rgb_packet(self):
        _, jpg = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
        meta = json.dumps(CAL).encode()
        packet = (struct.pack('>ddI', 1., 2., len(jpg)) + jpg.tobytes()
                  + struct.pack('>IIHH', 0, 0, 480, 640)
                  + b'RGB2' + struct.pack('>I', len(meta)) + meta + bytes(36))
        result = parse_packet(packet)
        self.assertEqual(result[0].shape, (480, 640, 3))
        self.assertIsNone(result[1])
        self.assertIsNone(result[2])
        self.assertEqual(result[-1], CAL)
        with self.assertRaises(ValueError):
            parse_packet(packet[:-1])

    def test_calibration_rejected(self):
        with self.assertRaises(ValueError):
            camera_parameters(CAL, 320, 240)
        with self.assertRaises(ValueError):
            camera_parameters(dict(CAL, coeffs=[.1, 0, 0, 0, 0]), 640, 480)
        with self.assertRaises(ValueError):
            camera_parameters(dict(CAL, fx=0), 640, 480)

    def test_known_pose(self):
        detector = AprilTagPose(.1)
        matrix, distortion = camera_parameters(CAL, 640, 480)
        rotation = np.array([2.9, .1, .05])
        translation = np.array([.02, -.03, 1.2])
        corners, _ = cv2.projectPoints(detector.points, rotation, translation, matrix, distortion)
        pose = detector.estimate(corners, CAL, 640, 480)
        np.testing.assert_allclose(pose['tvec'], translation, atol=1e-6)
        np.testing.assert_allclose(cv2.Rodrigues(pose['rvec'])[0],
                                   cv2.Rodrigues(rotation)[0], atol=1e-6)
        self.assertLess(pose['reprojection_px'], 1e-6)

    def test_generated_tag(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, 7, 160)
        image = np.full((480, 640), 255, np.uint8)
        image[160:320, 240:400] = marker
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        poses = AprilTagPose(.1).process(image, CAL)
        self.assertEqual([p['tag_id'] for p in poses], [7])
        self.assertAlmostEqual(poses[0]['tvec'][2], 600 * .1 / 159, delta=.01)


if __name__ == '__main__':
    unittest.main()
