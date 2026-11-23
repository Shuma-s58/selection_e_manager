"""3-second snapshot scheduler. The image subscriber never runs OCR."""
import json
import time
import threading
import uuid
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool


class CameraNode(Node):
    def __init__(self):
        super().__init__('selection_e_camera')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_topic_b', '')
        self.declare_parameter('capture_period_s', 3.0)
        self.declare_parameter('max_frame_age_s', 3.0)
        self.declare_parameter('output_dir', '/tmp/selection_e/captures')
        self.declare_parameter('max_recent_files', 3)
        self.declare_parameter('file_retention_s', 600.0)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = {'A': None, 'B': None}
        self.active = False
        self.next_camera = 'A'
        self.dual_camera = bool(self.get_parameter('image_topic_b').value)
        self.recent = []
        self.directory = Path(self.get_parameter('output_dir').value).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 lambda msg: self.on_image('A', msg), 10)
        if self.dual_camera:
            self.create_subscription(Image, self.get_parameter('image_topic_b').value,
                                     lambda msg: self.on_image('B', msg), 10)
        self.create_subscription(Bool, '/selection_e/active', self.on_active, 10)
        self.pub = self.create_publisher(String, '/selection_e/captured', 20)
        period = float(self.get_parameter('capture_period_s').value)
        if period <= 0:
            raise ValueError('capture_period_s must be positive')
        self.create_timer(period / 2 if self.dual_camera else period, self.capture)
        self.create_timer(30.0, self.cleanup)

    def cleanup(self):
        # Retain files longer than the OCR backlog and passage windows.
        cutoff = time.time() - float(self.get_parameter('file_retention_s').value)
        for file in self.directory.glob('*.jpg'):
            try:
                if file.stat().st_mtime < cutoff:
                    file.unlink(missing_ok=True)
            except OSError as exc:
                self.get_logger().warning(f'Cleanup failed: {exc}')

    def on_image(self, camera_id, msg):
        with self.lock:
            if self.active:
                self.latest[camera_id] = (msg, self.get_clock().now().nanoseconds / 1e9)

    def on_active(self, msg):
        with self.lock:
            if msg.data and not self.active:
                self.next_camera = 'A'
            self.active = msg.data
            if not msg.data:
                self.latest = {'A': None, 'B': None}

    def capture(self):
        with self.lock:
            if not self.active:
                return
            camera_id = self.next_camera
            if self.dual_camera:
                self.next_camera = 'B' if camera_id == 'A' else 'A'
            latest = self.latest[camera_id]
        if latest is None:
            return
        msg, received_at = latest
        stamp = self.get_clock().now().nanoseconds / 1e9
        if stamp - received_at > float(self.get_parameter('max_frame_age_s').value):
            self.get_logger().warning(f'Skipping stale frame from camera {camera_id}')
            return
        ident = uuid.uuid4().hex[:12]
        path = self.directory / f'{int(stamp * 1000)}_{camera_id}_{ident}.jpg'
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not cv2.imwrite(str(path), frame):
                raise RuntimeError('cv2.imwrite returned False')
        except Exception as exc:
            self.get_logger().error(f'Capture failed: {exc}')
            return
        payload = {'image_id': ident, 'camera_id': camera_id,
                   'captured_at': stamp, 'image_path': str(path)}
        out = String()
        out.data = json.dumps(payload)
        self.pub.publish(out)
        self.recent.append(path)
        limit = max(1, int(self.get_parameter('max_recent_files').value))
        # IMPORTANT: OCR reads the original file asynchronously. Do not delete queued
        # originals here. An external cleanup policy handles files after processing.
        if len(self.recent) > limit:
            self.recent.pop(0)
        self.get_logger().info(f'Captured {path.name}')


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
