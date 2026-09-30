"""RGB AprilTag pose in camera coordinates; no depth or RealSense dependency."""
import cv2
import numpy as np


def camera_parameters(calibration, width, height):
    if calibration.get('width') != width or calibration.get('height') != height:
        raise ValueError('Calibration resolution does not match the image')
    values = np.asarray([calibration[k] for k in ('fx', 'fy', 'ppx', 'ppy')], dtype=float)
    coeffs = np.asarray(calibration['coeffs'], dtype=float)
    if not np.isfinite(values).all() or (values[:2] <= 0).any():
        raise ValueError('Invalid RGB intrinsics')
    if coeffs.shape != (5,) or not np.isfinite(coeffs).all():
        raise ValueError('Invalid distortion coefficients')
    model = calibration['model'].split('.')[-1]
    # Inverse/modified Brown-Conrady are NOT OpenCV's forward Brown model.
    # Zero coefficients make these models equivalent to an undistorted pinhole.
    if model not in ('none', 'brown_conrady', 'inverse_brown_conrady', 'modified_brown_conrady'):
        raise ValueError(f'Unsupported distortion model: {model}')
    if model != 'brown_conrady' and np.any(coeffs != 0):
        raise ValueError(f'Nonzero {model} distortion requires explicit conversion')
    fx, fy, cx, cy = values
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]]), coeffs


class AprilTagPose:
    def __init__(self, size_m, family='36h11', max_error_px=2.0):
        if not np.isfinite(size_m) or size_m <= 0:
            raise ValueError('Tag size must be positive, in metres')
        if not np.isfinite(max_error_px) or max_error_px <= 0:
            raise ValueError('Reprojection threshold must be positive')
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, 'DICT_APRILTAG_' + family))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        self.size_m, self.max_error_px = size_m, max_error_px
        s = size_m / 2
        # IPPE_SQUARE: top-left, top-right, bottom-right, bottom-left.
        self.points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])

    def estimate(self, corners, calibration, width, height):
        matrix, distortion = camera_parameters(calibration, width, height)
        image_points = np.asarray(corners, dtype=float).reshape(4, 2)
        result = cv2.solvePnPGeneric(self.points, image_points, matrix, distortion,
                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        if result[0]:
            for rvec, tvec in zip(result[1], result[2]):
                rotation, _ = cv2.Rodrigues(rvec)
                if np.any((self.points @ rotation.T + tvec.reshape(3))[:, 2] <= 0):
                    continue
                projected, _ = cv2.projectPoints(self.points, rvec, tvec, matrix, distortion)
                error = float(np.sqrt(np.mean(np.sum(
                    (projected.reshape(4, 2) - image_points) ** 2, axis=1))))
                if np.isfinite(error) and np.isfinite(tvec).all():
                    candidates.append((error, rvec, tvec))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > self.max_error_px:
            return None
        error, rvec, tvec = candidates[0]
        return dict(rvec=rvec.reshape(3), tvec=tvec.reshape(3),
                    range_m=float(np.linalg.norm(tvec)), reprojection_px=error,
                    second_error_px=candidates[1][0] if len(candidates) > 1 else None)

    def process(self, image, calibration):
        h, w = image.shape[:2]
        matrix, distortion = camera_parameters(calibration, w, h)
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        poses = []
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            for tag_corners, tag_id in zip(corners, ids.flatten()):
                pose = self.estimate(tag_corners, calibration, w, h)
                if pose is None:
                    continue
                pose['tag_id'] = int(tag_id)
                poses.append(pose)
                cv2.drawFrameAxes(image, matrix, distortion, pose['rvec'], pose['tvec'], self.size_m / 2)
                origin = tuple(np.asarray(tag_corners).reshape(4, 2)[0].astype(int))
                cv2.putText(image, f"ID {tag_id} range={pose['range_m']:.3f}m",
                            origin, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
        return poses
