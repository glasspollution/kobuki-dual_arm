"""Perception-lite for the bimanual demo.

Detects the color-coded objects in the RGB image, then for each blob
publishes bearing, ground-plane range and apparent real-world height on
/detections as a JSON string:

    {"bottle": {"found": true, "bearing": 0.02, "range_base": 1.43,
                "est_h": 0.062, "area": 812}, ...}

Ranging: the camera is at a known height and pitch; the blob's bottom pixel
row is back-projected onto the floor plane. `range_base` is the horizontal
distance from the BASE CENTER to that floor point. `est_h` is the apparent
physical height of the blob at that range (used by the mission to classify
lying vs upright: a lying bottle is ~6 cm tall, an upright one ~20 cm).
"""
import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class PerceptionNode(Node):

    def __init__(self):
        super().__init__(
            'perception_node',
            automatically_declare_parameters_from_overrides=True)

        def p(name, default):
            try:
                v = self.get_parameter(name).value
                return default if v is None else v
            except Exception:  # noqa: BLE001
                return default

        self.cam_height = float(p('cam_height', 0.42))
        self.cam_pitch = float(p('cam_pitch', 0.2618))
        self.cam_x = float(p('cam_x', -0.075))
        self.min_area = int(p('min_area', 120))
        self.objects = list(p('objects', ['bottle', 'bowl', 'cracker_box']))

        self.hsv = {}
        for name in self.objects:
            lo = p(f'{name}.hsv_lo', None)
            hi = p(f'{name}.hsv_hi', None)
            if lo is None or hi is None:
                self.get_logger().warn(f'no HSV range for {name}, skipping')
                continue
            self.hsv[name] = (np.array([int(v) for v in lo], dtype=np.uint8),
                              np.array([int(v) for v in hi], dtype=np.uint8))

        # intrinsics (fallback: 640x480, hfov 1.204 as modeled in the URDF)
        self.fx = 640.0 / (2.0 * math.tan(1.204 / 2.0))
        self.fy = self.fx
        self.cx = 320.0
        self.cy = 240.0

        self.pub = self.create_publisher(String, '/detections', 10)
        self.create_subscription(Image, '/camera', self.on_image, 5)
        self.create_subscription(CameraInfo, '/camera_info', self.on_info, 5)
        self.get_logger().info(
            f'perception up, objects: {list(self.hsv.keys())}')

    def on_info(self, msg):
        if msg.k[0] > 1.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def ground_range(self, v_bottom):
        """Horizontal camera->floor-point distance for image row v."""
        depression = self.cam_pitch + math.atan2(v_bottom - self.cy, self.fy)
        if depression <= 0.02:
            return None
        return self.cam_height / math.tan(depression)

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'unsupported encoding {msg.encoding}',
                                   throttle_duration_sec=10.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        else:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        out = {}
        for name, (lo, hi) in self.hsv.items():
            mask = cv2.inRange(hsv, lo, hi)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            det = {'found': False}
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area >= self.min_area:
                    x, y, w, h = cv2.boundingRect(c)
                    u_mid = x + w / 2.0
                    v_bot = float(y + h)
                    rng = self.ground_range(v_bot)
                    if rng is not None:
                        bearing = -math.atan2(u_mid - self.cx, self.fx)
                        est_h = h * rng / self.fy
                        det = {
                            'found': True,
                            'bearing': round(bearing, 4),
                            'range_base': round(rng + self.cam_x, 4),
                            'est_h': round(est_h, 4),
                            'area': int(area),
                        }
            out[name] = det

        self.pub.publish(String(data=json.dumps(out)))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
