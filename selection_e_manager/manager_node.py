"""Mission state machine: SetBool start/stop, direction-aware image selection."""
import json
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from .common import parse_passage, choose_preferred_number


class ManagerNode(Node):
    def __init__(self):
        super().__init__('selection_e_manager')
        for key, default in (
            ('lookback_s', 30.0), ('post_wait_s', 18.0),
            ('front_ocr_grace_s', 15.0), ('rear_ocr_grace_s', 10.0), ('min_votes', 1),
            ('dedup_s', 8.0), ('result_retention_s', 120.0),
            ('max_results', 150), ('max_pending', 12),
            ('preferred_camera', 'A')):
            self.declare_parameter(key, default)
        self.active = False
        self.started_at = float('inf')
        self.results = deque(maxlen=int(self.get_parameter('max_results').value))
        self.captures = deque(maxlen=200)
        self.completed_ids = set()
        self.pending = []
        self.last_events = {}
        self.create_service(SetBool, '/selection_e/control', self.control)
        self.create_subscription(String, '/passing_direction', self.on_passage, 30)
        self.create_subscription(String, '/selection_e/recognized', self.on_recognized, 30)
        self.create_subscription(String, '/selection_e/captured', self.on_captured, 30)
        self.active_pub = self.create_publisher(Bool, '/selection_e/active', 10)
        self.speak_pub = self.create_publisher(String, '/selection_e/speak', 10)
        self.event_pub = self.create_publisher(String, '/selection_e/decision', 20)
        self.create_timer(0.25, self.tick)
        self.create_timer(1.0, self.publish_active)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def setting(self, name):
        return self.get_parameter(name).value

    def publish_active(self):
        msg = Bool()
        msg.data = self.active
        self.active_pub.publish(msg)

    def control(self, request, response):
        if request.data:
            if not self.active:
                self.active = True
                self.started_at = self.now()
                self.results.clear()
                self.captures.clear()
                self.completed_ids.clear()
                self.pending.clear()
                self.last_events.clear()
                self.get_logger().info('Mission started')
        else:
            self.active = False
            self.pending.clear()
            self.get_logger().info('Mission stopped')
        self.publish_active()
        response.success = True
        response.message = 'mission running' if self.active else 'mission stopped'
        return response

    def on_captured(self, msg):
        if not self.active:
            return
        try:
            item = json.loads(msg.data)
            if float(item['captured_at']) >= self.started_at:
                self.captures.append(item)
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Invalid capture metadata')

    def on_recognized(self, msg):
        if not self.active:
            return
        try:
            item = json.loads(msg.data)
            if float(item['captured_at']) >= self.started_at:
                self.completed_ids.add(item['image_id'])
                if item.get('success'):
                    self.results.append(item)
                    self.get_logger().info(f'Retained OCR result {item.get("numbers")} from {item["captured_at"]}')
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Invalid OCR result')

    def on_passage(self, msg):
        if not self.active:
            return
        event = parse_passage(msg.data)
        if not event:
            self.get_logger().warning('Unknown passing_direction payload; see README')
            return
        now = self.now()
        direction, target = event['direction'], event['track_id']
        # Reject identical track + direction in a small time window.
        # Events without IDs are deduplicated by direction, conservative single-target policy.
        key = (target or 'unknown', direction)
        if now - self.last_events.get(key, float('-inf')) < float(self.setting('dedup_s')):
            return
        self.last_events[key] = now
        if len(self.pending) >= int(self.setting('max_pending')):
            self.get_logger().warning('Pending event queue full; ignoring passage')
            return
        self.pending.append({'direction': direction, 'track_id': target,
                             'at': now, 'created': now})
        self.get_logger().info(f'Passage {direction} track={target} at {now:.2f}')

    def eligible(self, event):
        at = event['at']
        if event['direction'] == 'FRONT_TO_REAR':
            start, end = max(self.started_at, at - float(self.setting('lookback_s'))), at
        else:
            start, end = at, at + float(self.setting('post_wait_s'))
        # Choose the last three captures per camera, then include their OCR
        # results. An older success must not override newer empty frames.
        recent_ids = set()
        for camera_id in ('A', 'B'):
            captures = [x for x in self.captures
                        if x.get('camera_id', 'A') == camera_id
                        and start <= float(x['captured_at']) < end]
            recent_ids.update(x['image_id'] for x in captures[-3:])
        return [x for x in self.results if x.get('image_id') in recent_ids]

    def send_decision(self, event, number, status, camera_id=None):
        detail = dict(direction=event['direction'], track_id=event['track_id'],
                      passage_at=event['at'], number=number, camera_id=camera_id,
                      status=status)
        out = String()
        out.data = json.dumps(detail, ensure_ascii=False)
        self.event_pub.publish(out)
        self.get_logger().info(f'Decision: {out.data}')
        if number:
            request = String()
            request.data = str(number)
            self.speak_pub.publish(request)

    def tick(self):
        if not self.active:
            return
        now = self.now()
        retention = float(self.setting('result_retention_s'))
        while self.results and float(self.results[0]['captured_at']) < now - retention:
            self.results.popleft()
        keep = []
        for event in self.pending:
            candidates = self.eligible(event)
            preferred = str(self.setting('preferred_camera')).upper()
            if preferred not in ('A', 'B'):
                self.get_logger().warning('preferred_camera must be A or B; using A')
                preferred = 'A'
            number, camera_id = choose_preferred_number(
                candidates, preferred, int(self.setting('min_votes')))
            if event['direction'] == 'FRONT_TO_REAR':
                # Wait for OCR of all captured pre-crossing images, up to a bounded grace period.
                captures = [x for x in self.captures if max(self.started_at, event['at'] - float(self.setting('lookback_s')))
                            <= float(x['captured_at']) < event['at']]
                recent_captures = []
                for source in ('A', 'B'):
                    recent_captures.extend(
                        [x for x in captures if x.get('camera_id', 'A') == source][-3:])
                all_done = all(c.get('image_id') in self.completed_ids for c in recent_captures)
                ready = all_done or now >= event['at'] + float(self.setting('front_ocr_grace_s'))
            else:
                # For post-crossing, allow 1+ recognized frames; wait for capture window
                # to close before deciding to improve majority vote consistency.
                ready = now >= event['at'] + float(self.setting('post_wait_s')) + float(self.setting('rear_ocr_grace_s'))
            if ready:
                self.send_decision(event, number, 'spoken' if number else 'no_unambiguous_number', camera_id)
            else:
                keep.append(event)
        self.pending = keep


def main():
    rclpy.init()
    node = ManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
