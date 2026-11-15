#!/usr/bin/env python3
"""Live check of aruco_detector_node (Requirement 1). Needs Gazebo + the detector already running
(check_perception.sh starts both). Prints PASS/FAIL per check and saves /marker_image to a png.

  python3 check_perception.py [out_dir]
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, Int32

FX_SIM, L = 381.36, 0.2
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/perception_check'


class Checker(Node):
    def __init__(self):
        super().__init__('perception_checker')
        self.info = None
        self.det = []          # (time, data)
        self.debug = None
        self.marker_imgs = []
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/detected_marker',
                                 lambda m: self.det.append((time.time(), list(m.data))), 10)
        self.create_subscription(Image, '/aruco_debug', self.on_debug, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/marker_image', lambda m: self.marker_imgs.append(m), latched)
        self.reached_pub = self.create_publisher(Int32, '/marker_reached', 10)

    def on_info(self, m):
        self.info = m

    def on_debug(self, m):
        self.debug = m


def spin_for(node, seconds, until=None):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if until is not None and until():
            return True
    return until() if until else True


results = []


def check(name, ok, info=''):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {info}", flush=True)


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    n = Checker()

    # 1. camera_info
    spin_for(n, 20, lambda: n.info is not None)
    if n.info is None:
        check('/camera/camera_info received', False, 'nothing in 20 s: is Gazebo running?')
    else:
        fx, cx = n.info.k[0], n.info.k[2]
        check('/camera/camera_info received', True, f'fx={fx:.2f} cx={cx:.2f} {n.info.width}x{n.info.height}')
        check('fx is the simulated one (381.36)', abs(fx - FX_SIM) < 1.0)

    # 2. /detected_marker rate and content
    spin_for(n, 20, lambda: len(n.det) > 0)
    n.det.clear()
    spin_for(n, 3.0)
    if not n.det:
        check('/detected_marker published', False, 'nothing: is aruco_detector_node running?')
    else:
        rate = len(n.det) / 3.0
        check('/detected_marker rate ~ camera rate', rate > 5.0, f'{rate:.1f} Hz')
        data = n.det[-1][1]
        check('layout [cx, fx, id, u, v, s, ...]', len(data) >= 2 and (len(data) - 2) % 4 == 0, str([round(x, 1) for x in data]))
        check('fx inside the message = camera_info fx', n.info is not None and abs(data[1] - n.info.k[0]) < 1e-3, f'{data[1]:.2f}')
        markers = {int(round(data[k])): (data[k + 1], data[k + 2], data[k + 3]) for k in range(2, len(data), 4)}
        if 1 in markers:
            u, v, s = markers[1]
            d = FX_SIM * L / s
            check('marker 1 visible from the start pose', True, f'u={u:.0f} v={v:.0f} s={s:.1f}px -> d=fx*L/s={d:.2f} m')
            check('marker 1 roughly centred (|u-cx| < 40 px)', abs(u - data[0]) < 40)
            check('s matches box 1 at ~3.75 m (expected ~20 px)', 14.0 < s < 28.0)
        else:
            check('marker 1 visible from the start pose', False, f'ids seen: {sorted(markers)} (did the robot move?)')
        bad = [i for i in markers if i not in (1, 2, 3, 4)]
        check('no unknown ids (false positives)', not bad, str(bad))

    # 3. debug stream
    spin_for(n, 5, lambda: n.debug is not None)
    check('/aruco_debug published', n.debug is not None,
          '' if n.debug is None else f'{n.debug.width}x{n.debug.height} {n.debug.encoding}')

    # 4. /marker_image only after /marker_reached
    spin_for(n, 1.0)
    check('no /marker_image before any /marker_reached', len(n.marker_imgs) == 0)
    spin_for(n, 5, lambda: n.reached_pub.get_subscription_count() > 0)
    check('detector subscribed to /marker_reached', n.reached_pub.get_subscription_count() > 0)
    n.reached_pub.publish(Int32(data=1))
    spin_for(n, 5, lambda: len(n.marker_imgs) > 0)
    check('/marker_reached 1 -> /marker_image published', len(n.marker_imgs) == 1)
    if n.marker_imgs:
        m = n.marker_imgs[-1]
        check('/marker_image is bgr8 640x480', m.encoding == 'bgr8' and m.width == 640 and m.height == 480,
              f'{m.width}x{m.height} {m.encoding}')
        try:
            import numpy as np
            import cv2
            img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
            green = int(np.all(img == (0, 255, 0), axis=2).sum())
            check('green circle + id drawn', green > 200, f'{green} green px')
            path = os.path.join(OUT, 'marker_image_id1.png')
            cv2.imwrite(path, img)
            print(f'      saved {path}')
        except Exception as e:  # noqa: BLE001
            print(f'      could not save image: {e}')

    # marker never seen -> nothing published
    n.reached_pub.publish(Int32(data=3))
    spin_for(n, 2.0)
    check('/marker_reached 3 (never seen) -> no new image', len(n.marker_imgs) == 1)

    print(f'\n{sum(results)}/{len(results)} checks passed', flush=True)
    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
