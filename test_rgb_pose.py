import json
import struct
import unittest
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
