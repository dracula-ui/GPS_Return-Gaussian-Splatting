#!/usr/bin/env python3
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rtlf_ros.ros_io import decode_depth, decode_rgb


class Grab(Node):
    def __init__(self):
        super().__init__("save_frame")
        self.rgb, self.depth = None, None
        self.create_subscription(Image, "/rtlf/rgbd/image", lambda m: setattr(self, "rgb", m) if self.rgb is None else None, qos_profile_sensor_data)
        self.create_subscription(Image, "/rtlf/rgbd/depth_image", lambda m: setattr(self, "depth", m) if self.depth is None else None, qos_profile_sensor_data)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    rclpy.init(); n = Grab()
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.rgb is not None and n.depth is not None:
            break
    if n.rgb is None or n.depth is None:
        print("NO IMAGES received (rgb: %s, depth: %s)" % (n.rgb is not None, n.depth is not None)); return
    rgb = decode_rgb(n.rgb); d = decode_depth(n.depth)
    cv2.imwrite(os.path.join(out, "frame_rgb.png"), rgb[:, :, ::-1])
    v = d > 0
    dv = np.zeros_like(d); dv[v] = np.clip(d[v] / 40.0, 0, 1)
    cv2.imwrite(os.path.join(out, "frame_depth.png"), cv2.applyColorMap((dv * 255).astype(np.uint8), cv2.COLORMAP_JET))
    print("rgb  : %dx%d mean=%.1f std=%.1f (std near 0 = blank image)" % (rgb.shape[1], rgb.shape[0], rgb.mean(), rgb.std()))
    print("depth: valid pixels %.1f%%, min %.2f m, max %.2f m" % (100 * v.mean(), d[v].min() if v.any() else 0, d.max()))
    print("saved in", os.path.abspath(out))


if __name__ == "__main__":
    main()
