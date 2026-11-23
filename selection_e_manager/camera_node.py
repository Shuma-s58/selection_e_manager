"""Read V4L2 camera devices directly and save periodic JPEG snapshots for OCR."""
import json
import time
import threading
import uuid
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class CameraNode(Node):
    def __init__(self):
        super().__init__('selection_e_camera')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('video_device_b', '/dev/video1')
        self.declare_parameter('capture_period_s', 3.0)
        self.declare_parameter('max_frame_age_s', 3.0)
        self.declare_parameter('output_dir', '/tmp/selection_e/captures')
        self.declare_parameter('max_recent_files', 3)
        self.declare_parameter('file_retention_s', 600.0)
        self.lock = threading.Lock()
        self.latest = {'A': None, 'B': None}
        self.active = False
        self.next_camera = 'A'
        self.devices = {'A': str(self.get_parameter('video_device').value),
                        'B': str(self.get_parameter('video_device_b').value)}
        if not self.devices['A']:
            raise ValueError('video_device must be set')
        self.dual_camera = bool(self.devices['B'])
        self.stop_event = threading.Event()
        self.workers = []
        self.recent = []
        self.directory = Path(self.get_parameter('output_dir').value).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Bool, '/selection_e/active', self.on_active, 10)
        self.pub = self.create_publisher(String, '/selection_e/captured', 20)
        period = float(self.get_parameter('capture_period_s').value)
        if period <= 0:
            raise ValueError('capture_period_s must be positive')
        if float(self.get_parameter('max_frame_age_s').value) <= 0:
            raise ValueError('max_frame_age_s must be positive')
        for camera_id in ('A', 'B') if self.dual_camera else ('A',):
            worker = threading.Thread(target=self.read_camera, args=(camera_id,),
                                      name=f'selection-e-camera-{camera_id}', daemon=True)
            worker.start()
            self.workers.append(worker)
        self.create_timer(period / 2 if self.dual_camera else period, self.capture)
        self.create_timer(30.0, self.cleanup)

    def read_camera(self, camera_id):
        """Drain the device continuously so periodic snapshots use recent frames."""
        device = self.devices[camera_id]
        cap = None
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    active = self.active
                if not active:
                    if cap is not None:
                        cap.release()
                        cap = None
                    self.stop_event.wait(0.2)
                    continue
                if cap is None:
                    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                    if not cap.isOpened():
                        self.get_logger().warning(f'Cannot open camera {camera_id}: {device}')
                        cap.release()
                        cap = None
                        self.stop_event.wait(2.0)
                        continue
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.get_logger().warning(f'Camera {camera_id} read failed: {device}')
                    cap.release()
                    cap = None
                    self.stop_event.wait(1.0)
                    continue
                received_at = self.get_clock().now().nanoseconds / 1e9
                with self.lock:
                    if self.active:
                        self.latest[camera_id] = (frame, received_at)
        finally:
            if cap is not None:
                cap.release()

    def cleanup(self):
        # Retain files longer than the OCR backlog and passage windows.
        cutoff = time.time() - float(self.get_parameter('file_retention_s').value)
        for file in self.directory.glob('*.jpg'):
            try:
                if file.stat().st_mtime < cutoff:
                    file.unlink(missing_ok=True)
            except OSError as exc:
                self.get_logger().warning(f'Cleanup failed: {exc}')

    def on_active(self, msg):
        with self.lock:
            if msg.data and not self.active:
                self.next_camera = 'A'
                self.latest = {'A': None, 'B': None}
            self.active = bool(msg.data)
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
        frame, received_at = latest
        stamp = self.get_clock().now().nanoseconds / 1e9
        if stamp - received_at > float(self.get_parameter('max_frame_age_s').value):
            self.get_logger().warning(f'Skipping stale frame from camera {camera_id}')
            return
        ident = uuid.uuid4().hex[:12]
        path = self.directory / f'{int(stamp * 1000)}_{camera_id}_{ident}.jpg'
        try:
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
        # OCR reads the file asynchronously; keep queued originals for cleanup.
        if len(self.recent) > limit:
            self.recent.pop(0)
        self.get_logger().info(f'Captured {path.name}')

    def shutdown(self):
        self.stop_event.set()
        for worker in self.workers:
            worker.join(timeout=3.0)
            if worker.is_alive():
                self.get_logger().warning(f'Camera worker {worker.name} did not stop')


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
