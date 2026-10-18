"""Camera -> ball detection (HSV baseline or YOLO) -> distance/bearing -> /cmd_vel_raw (then obstacle_avoid -> heading_hold).

Extras for the report:
  * auto-labelling: when collect_dir is set, HSV detections are saved as a YOLO dataset
    (train YOLOv8n on it with scripts/train.py, then run detector:=yolo model_path:=best.pt)
  * metrics.csv: detection rate, latency, distance error per frame
"""
import csv
import math
import os
import random
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

BALL_RADIUS_M = 0.10


def detect_hsv(rgb):
    """Return (cx, cy, r_px) of the largest red blob or None."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 20:
        return None
    (x, y), r = cv2.minEnclosingCircle(c)
    return x, y, r


class Follower(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        self.declare_parameter('detector', 'hsv')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('collect_dir', '')
        self.declare_parameter('target_dist', 1.2)   # [m]
        self.declare_parameter('hfov', 1.2)          # [rad]
        self.declare_parameter('k_ang', 1.8)
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('max_lin', 0.4)
        self.declare_parameter('max_ang', 1.5)
        self.declare_parameter('metrics_file', 'metrics.csv')
        g = lambda n: self.get_parameter(n).value
        self.mode, self.conf = g('detector'), g('conf')
        self.d_star, self.hfov = g('target_dist'), g('hfov')
        self.k_ang, self.k_lin = g('k_ang'), g('k_lin')
        self.max_lin, self.max_ang = g('max_lin'), g('max_ang')
        self.collect_dir = g('collect_dir')

        self.model = None
        if self.mode == 'yolo':
            from ultralytics import YOLO
            self.model = YOLO(g('model_path'))
            self.custom = os.path.basename(g('model_path')) != 'yolov8n.pt'
        self.get_logger().info(f'detector={self.mode}')

        self.pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.dbg = self.create_publisher(Image, 'debug_image', 1)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 1)
        self.last_seen, self.last_dir, self.n, self.saved = 0.0, 1.0, 0, 0
        self.log = open(g('metrics_file'), 'w', newline='')
        self.csv = csv.writer(self.log)
        self.csv.writerow(['t', 'detected', 'dist_m', 'dist_err_m', 'bearing_rad', 'latency_ms'])
        if self.collect_dir:
            for s in ('train', 'val'):
                os.makedirs(f'{self.collect_dir}/images/{s}', exist_ok=True)
                os.makedirs(f'{self.collect_dir}/labels/{s}', exist_ok=True)
            with open(f'{self.collect_dir}/data.yaml', 'w') as f:
                f.write(f'path: {os.path.abspath(self.collect_dir)}\ntrain: images/train\nval: images/val\nnames:\n  0: ball\n')

    def detect_yolo(self, rgb):
        res = self.model.predict(rgb, imgsz=320, conf=self.conf, device='cpu', verbose=False)[0]
        best = None
        for b in res.boxes:
            cls = int(b.cls[0])
            if not self.custom and cls != 32:      # COCO "sports ball"
                continue
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            c = float(b.conf[0])
            if best is None or c > best[0]:
                best = (c, (x1 + x2) / 2, (y1 + y2) / 2, ((x2 - x1) + (y2 - y1)) / 4)
        return None if best is None else best[1:]

    def on_image(self, msg):
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        h, w = rgb.shape[:2]
        f = (w / 2) / math.tan(self.hfov / 2)                   # focal length [px]
        t0 = time.perf_counter()
        det = self.detect_yolo(rgb) if self.mode == 'yolo' else detect_hsv(rgb)
        lat = (time.perf_counter() - t0) * 1000
        now = self.get_clock().now().nanoseconds * 1e-9
        cmd = Twist()
        dist = err = bearing = float('nan')

        if det:
            cx, cy, r = det
            self.last_seen = now
            bearing = math.atan2(cx - w / 2, f)                 # +: ball is to the right
            dist = f * BALL_RADIUS_M / max(r, 1.0)              # pinhole model
            err = dist - self.d_star
            self.last_dir = -1.0 if bearing > 0 else 1.0
            cmd.angular.z = float(np.clip(-self.k_ang * bearing, -self.max_ang, self.max_ang))
            v = self.k_lin * err * max(0.0, math.cos(bearing * 2))   # slow down when ball is off-axis
            cmd.linear.x = float(np.clip(v, -0.2, self.max_lin))
            cv2.circle(rgb, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
            cv2.putText(rgb, f'{self.mode} d={dist:.2f}m', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if self.collect_dir and self.mode == 'hsv' and self.n % 3 == 0:
                self.save_sample(msg, rgb_raw=np.frombuffer(msg.data, np.uint8).reshape(h, w, 3), c=(cx, cy, r))
        elif now - self.last_seen > 0.5:                        # lost -> search
            cmd.angular.z = 0.6 * self.last_dir
            cv2.putText(rgb, 'SEARCH', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        self.pub.publish(cmd)
        out = Image(header=msg.header, height=h, width=w, encoding='rgb8', step=w * 3, data=rgb.tobytes())
        self.dbg.publish(out)
        self.csv.writerow([f'{now:.2f}', int(det is not None), f'{dist:.3f}', f'{err:.3f}', f'{bearing:.3f}', f'{lat:.1f}'])
        self.n += 1

    def save_sample(self, msg, rgb_raw, c):
        h, w = rgb_raw.shape[:2]
        cx, cy, r = c
        split = 'val' if random.random() < 0.2 else 'train'
        name = f'{self.saved:06d}'
        cv2.imwrite(f'{self.collect_dir}/images/{split}/{name}.jpg', cv2.cvtColor(rgb_raw, cv2.COLOR_RGB2BGR))
        with open(f'{self.collect_dir}/labels/{split}/{name}.txt', 'w') as fh:
            fh.write(f'0 {cx / w:.6f} {cy / h:.6f} {2 * r / w:.6f} {2 * r / h:.6f}\n')
        self.saved += 1

    def destroy_node(self):
        self.log.close()
        super().destroy_node()


def main():
    rclpy.init()
    rclpy.spin(Follower())
