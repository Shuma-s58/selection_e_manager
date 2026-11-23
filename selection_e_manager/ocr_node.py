"""ROS 2 OCR bridge: automatically starts a persistent Florence-2 process in a venv.

ROS imports execute only in the ROS interpreter, while transformers/torch execute
in the Python environment selected by florence_python (as in box_mission).
"""
import json
import os
import queue
import re
import subprocess
import threading
import time
import traceback
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class OcrNode(Node):
    def __init__(self):
        super().__init__('selection_e_ocr')
        self.declare_parameter('florence_python', '~/florence2_work/.venv/bin/python')
        self.declare_parameter('florence_script', '~/florence2_work/florence2_number.py')
        self.declare_parameter('model', 'florence-community/Florence-2-large')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('queue_capacity', 5)
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('remove_failed_images', True)
        self.declare_parameter('startup_timeout_s', 300.0)
        self.active = False
        self.queue = queue.Queue(maxsize=max(1, int(self.get_parameter('queue_capacity').value)))
        self.publisher = self.create_publisher(String, '/selection_e/recognized', 20)
        self.status_pub = self.create_publisher(String, '/selection_e/ocr_status', 10)
        self.create_subscription(String, '/selection_e/captured', self.on_captured, 20)
        self.create_subscription(Bool, '/selection_e/active', self.on_active, 10)
        self.stop_event = threading.Event()
        self.process = None
        self.ready = False
        self.worker = threading.Thread(target=self.loop, name='selection-e-ocr-bridge', daemon=True)
        self.worker.start()

    def setting(self, key):
        return self.get_parameter(key).value

    def status(self, state, info=''):
        payload = String()
        payload.data = json.dumps({'state': state, 'detail': str(info)}, ensure_ascii=False)
        self.status_pub.publish(payload)
        self.get_logger().info(f'OCR: {state}: {info}')

    def on_active(self, msg):
        self.active = bool(msg.data)

    def on_captured(self, msg):
        if not self.active:
            return
        try:
            item = json.loads(msg.data)
            if not all(k in item for k in ('image_id', 'image_path', 'captured_at')):
                raise ValueError('Capture metadata is missing required fields')
            self.queue.put_nowait(item)
        except queue.Full:
            self.get_logger().warning('OCR queue full; skipping newly captured image')
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f'Invalid captured payload: {exc}')

    def validate_environment(self):
        executable = Path(str(self.setting('florence_python'))).expanduser().absolute()
        script = Path(str(self.setting('florence_script'))).expanduser().resolve()
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError(f'Florence Python not executable: {executable}')
        if not script.is_file():
            raise RuntimeError(f'Florence script missing: {script}')
        return str(executable), str(script)

    def start_subprocess(self):
        python, script = self.validate_environment()
        worker_script = Path(__file__).with_name('florence_worker.py')
        cmd = [python, '-u', str(worker_script), '--script', script,
               '--model', str(self.setting('model')), '--device', str(self.setting('device'))]
        self.status('loading', f'Python={python}; script={script}')
        self.process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=None, text=True, bufsize=1)
        # Nonblocking startup timeout: reader thread handles a slow model download/load.
        outcome = queue.Queue(maxsize=1)
        def read_ready():
            try:
                outcome.put(('line', self.process.stdout.readline()))
            except Exception as exc:
                outcome.put(('error', str(exc)))
        threading.Thread(target=read_ready, daemon=True).start()
        deadline = time.monotonic() + max(1.0, float(self.setting('startup_timeout_s')))
        while not self.stop_event.is_set():
            try:
                kind, result = outcome.get(timeout=0.2)
                if kind == 'error':
                    raise RuntimeError(result)
                if not result:
                    raise RuntimeError('Florence subprocess exited before ready (inspect stderr)')
                message = json.loads(result)
                if not message.get('ready'):
                    raise RuntimeError(f'Unexpected Florence startup output: {result!r}')
                self.ready = True
                self.status('ready', 'Florence model loaded; waiting for captured images')
                return
            except queue.Empty:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Florence model startup timed out')
        raise RuntimeError('Stopping')

    def recognize(self, data):
        if self.setting('mock_mode'):
            match = re.search(r'_num([0-9]+)_', data['image_path'])
            nums = [match.group(1)] if match else []
            return {'numbers': nums, 'regions': [{'text': n, 'bbox': [0, 0, 1, 1]} for n in nums],
                    'success': bool(nums)}
        if not self.ready or self.process is None or self.process.poll() is not None:
            raise RuntimeError('Florence process not ready')
        self.process.stdin.write(json.dumps({'image_path': data['image_path']}) + '\n')
        self.process.stdin.flush()
        reply = self.process.stdout.readline()
        if not reply:
            raise RuntimeError('Florence worker exited during inference')
        return json.loads(reply)

    def loop(self):
        if not self.setting('mock_mode'):
            # Start and pre-load Florence-2 automatically at ROS node startup.
            try:
                self.start_subprocess()
            except Exception as exc:
                self.status('error', str(exc))
                self.get_logger().error(traceback.format_exc(limit=3))
                self.close_process()
                # Never silently fall back to a different Python / CPU model.
                return
        else:
            self.status('ready', 'mock_mode; real Florence model not loaded')
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                data = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if not self.active:
                    continue
                response = self.recognize(data)
                numbers = list(dict.fromkeys(str(n) for n in response.get('numbers', [])))
                ok = bool(response.get('success')) and bool(numbers)
                result = {'image_id': data['image_id'],
                          'camera_id': data.get('camera_id', 'A'),
                          'success': ok, 'numbers': numbers}
                msg = String()
                msg.data = json.dumps(result, ensure_ascii=False)
                self.publisher.publish(msg)
                if not ok and self.setting('remove_failed_images'):
                    Path(data['image_path']).unlink(missing_ok=True)
                self.get_logger().info(f'OCR result image={data["image_id"]}: {numbers}')
            except Exception as exc:
                self.get_logger().error(f'OCR failed: {exc}')
                out = String()
                out.data = json.dumps({'image_id': data['image_id'],
                                       'camera_id': data.get('camera_id', 'A'),
                                       'success': False, 'numbers': []})
                self.publisher.publish(out)
                if self.process is not None and self.process.poll() is not None:
                    self.status('error', 'Florence worker stopped unexpectedly')
                    break
            finally:
                self.queue.task_done()

    def close_process(self):
        if self.process is None:
            return
        proc = self.process
        self.process = None
        self.ready = False
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
        for stream in (proc.stdin, proc.stdout):
            if stream:
                stream.close()

    def shutdown(self):
        self.stop_event.set()
        self.close_process()
        self.worker.join(timeout=3)


def main():
    rclpy.init()
    node = OcrNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
