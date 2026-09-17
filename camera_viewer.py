#!/usr/bin/env python3
"""camera_viewer — a bare live camera window. Image only, no text, no overlay.

Same idea as apriltag_reloc_viewer.py, minus everything that isn't the picture:
no status banner, no tag ids, no frame-age label. It just shows the newest frame
of whatever image topic you point it at.

Like the reloc viewer it is a pure ROS subscriber — it does NOT open the camera.
realsense_server.py on the Go2 serves only ONE TCP client (listen(1), one accept
at a time), and in the usual setup that client is the AprilTag detector — so the
detector's debug stream is the only camera topic on the graph, and it is what
this viewer defaults to:

  /apriltag/image/compressed      apriltag_checkpoint_tcp.launch.py — the DEFAULT.
                                  Heads up: the tag outlines and the "id=… m"
                                  text are BAKED INTO the image by the detector
                                  (apriltag_core.py TagPoseDetector.draw), so
                                  those stay visible; this viewer draws nothing
                                  of its own but cannot strip them either.
  /head_camera/color/compressed   a clean, undrawn camera stream — but only when
                                  head_camera_node / head_camera_tcp_bridge is
                                  the running TCP client INSTEAD of the detector
                                  (they cannot share the server).

Run (on a PC with a display):
    ros2 launch go2_perception camera_viewer.launch.py
    ros2 launch go2_perception camera_viewer.launch.py topic:=/head_camera/color/compressed
  or directly:
    python3 camera_viewer.py --ros-args -p topic:=/apriltag/image/compressed

If the window stays blank, nobody is publishing the topic — check with
`ros2 topic list` and point `topic:=` at one that exists.

Parameters:
    topic       image topic (default /apriltag/image/compressed). A topic
                ending in /compressed is read as sensor_msgs/CompressedImage,
                anything else as sensor_msgs/Image.
    window      window title (default 'Camera')
    fullscreen  start fullscreen (default false)

Controls:  q / ESC — quit,  f — toggle fullscreen
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import CompressedImage, Image


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        p = self.declare_parameter
        self.topic = str(p('topic', '/apriltag/image/compressed').value)
        self.window = str(p('window', 'Camera').value)
        self.fullscreen = bool(p('fullscreen', False).value)

        # Publishers use BEST_EFFORT sensor-data QoS (see apriltag_tcp_detector_
        # node), so we must match or we'd receive nothing. depth=1: only the
        # newest frame matters for live viewing.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        if self.topic.endswith('/compressed'):
            self.create_subscription(
                CompressedImage, self.topic, self._on_compressed, qos)
        else:
            self.create_subscription(Image, self.topic, self._on_raw, qos)

        self._frame = None
        self.get_logger().info(f'camera_viewer up — {self.topic} (q/ESC quits)')

    # ── subscriptions ────────────────────────────────────────────
    def _on_compressed(self, msg: CompressedImage):
        if msg.data:
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                self._frame = img

    def _on_raw(self, msg: Image):
        img = self._from_image_msg(msg)
        if img is not None:
            self._frame = img

    @staticmethod
    def _from_image_msg(msg: Image):
        """sensor_msgs/Image → BGR numpy, without pulling in cv_bridge."""
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        enc = msg.encoding.lower()
        try:
            if enc in ('bgr8', 'rgb8'):
                img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
                return img if enc == 'bgr8' else cv2.cvtColor(
                    img, cv2.COLOR_RGB2BGR)
            if enc == 'mono8':
                gray = buf.reshape(msg.height, msg.step)[:, :msg.width]
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except ValueError:
            return None            # truncated frame — skip it
        return None                # unsupported encoding

    # ── rendering ────────────────────────────────────────────────
    def render(self):
        """Show the newest frame. Returns False when the user quits."""
        if self._frame is not None:
            cv2.imshow(self.window, self._frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('f'):
            self._toggle_fullscreen()
        return key not in (ord('q'), 27)

    def _toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        cv2.setWindowProperty(
            self.window, cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    # WINDOW_NORMAL so the window is resizable and fullscreen can be toggled.
    cv2.namedWindow(node.window, cv2.WINDOW_NORMAL)
    if node.fullscreen:
        cv2.setWindowProperty(node.window, cv2.WND_PROP_FULLSCREEN,
                              cv2.WINDOW_FULLSCREEN)
    try:
        while rclpy.ok():
            # spin_once handles ONE callback per call, so drain the queue first
            # and draw only the FRESHEST frame; cv2 must render on the MAIN
            # thread (same pattern as apriltag_reloc_viewer.py).
            rclpy.spin_once(node, timeout_sec=0.01)
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0.0)
            if not node.render():
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
