"""Async Open JTalk speaker compatible with tsukuba_E_speak's voice configuration."""
import queue
import re
import subprocess
import tempfile
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeakerNode(Node):
    def __init__(self):
        super().__init__('selection_e_speaker')
        self.declare_parameter('dictionary_path', '/var/lib/mecab/dic/open-jtalk/naist-jdic')
        self.declare_parameter('voice_path', '/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice')
        self.declare_parameter('dry_run', False)
        self.queue = queue.Queue(maxsize=10)
        self.create_subscription(String, '/selection_e/speak', self.on_message, 10)
        threading.Thread(target=self.worker, daemon=True).start()

    def on_message(self, msg):
        if not re.fullmatch(r'[0-9]+', msg.data):
            self.get_logger().warning('Rejecting non-digit number')
            return
        try:
            self.queue.put_nowait(msg.data)
        except queue.Full:
            self.get_logger().warning('Speech queue full, dropped number')

    def worker(self):
        while rclpy.ok():
            try:
                number = self.queue.get(timeout=0.3)
            except queue.Empty:
                continue
            text = f'チーム番号は{int(number)}です。'
            try:
                if self.get_parameter('dry_run').value:
                    self.get_logger().info(f'[DRY RUN] {text}')
                else:
                    with tempfile.TemporaryDirectory() as tmp:
                        wav = str(Path(tmp) / 'speech.wav')
                        subprocess.run(['open_jtalk', '-x', str(self.get_parameter('dictionary_path').value),
                                        '-m', str(self.get_parameter('voice_path').value), '-ow', wav],
                                       input=text, text=True, encoding='utf-8', check=True, timeout=25)
                        subprocess.run(['aplay', wav], check=True, timeout=60)
                    self.get_logger().info(f'Spoke {text}')
            except (OSError, subprocess.SubprocessError) as exc:
                self.get_logger().error(f'Speech failed: {exc}')
            finally:
                self.queue.task_done()


def main():
    rclpy.init()
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
