"""Image decoding without cv_bridge, TUM dataset writer."""
import os

import numpy as np


def decode_rgb(msg):
    """sensor_msgs/Image rgb8 or bgr8 -> HxWx3 uint8 RGB."""
    a = np.frombuffer(msg.data, dtype=np.uint8)
    h, w = msg.height, msg.width
    a = a.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
    if msg.encoding == "bgr8":
        a = a[:, :, ::-1]
    elif msg.encoding != "rgb8":
        raise ValueError("unsupported color encoding %s" % msg.encoding)
    return np.ascontiguousarray(a)


def decode_depth(msg):
    """sensor_msgs/Image 32FC1 (m) or 16UC1 (mm) -> HxW float32 metres; invalid (inf/nan/<=0) -> 0."""
    h, w = msg.height, msg.width
    if msg.encoding == "32FC1":
        a = np.frombuffer(msg.data, dtype=np.float32).reshape(h, msg.step // 4)[:, :w].copy()
    elif msg.encoding == "16UC1":
        a = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, msg.step // 2)[:, :w].astype(np.float32) / 1000.0
    else:
        raise ValueError("unsupported depth encoding %s" % msg.encoding)
    a[~np.isfinite(a)] = 0.0
    a[a < 0] = 0.0
    return a


class TumWriter:
    """Writes a TUM-RGBD style folder that MonoGS can read offline (type: tum)."""

    def __init__(self, root, depth_scale):
        self.root, self.depth_scale = root, depth_scale
        os.makedirs(os.path.join(root, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(root, "depth"), exist_ok=True)
        self.f_rgb = open(os.path.join(root, "rgb.txt"), "w")
        self.f_depth = open(os.path.join(root, "depth.txt"), "w")
        self.f_gt = open(os.path.join(root, "groundtruth.txt"), "w")
        for f, what in ((self.f_rgb, "color images"), (self.f_depth, "depth images"), (self.f_gt, "camera pose (NED) tx ty tz qx qy qz qw")):
            f.write("# %s\n# timestamp [file|pose]\n" % what)
        self.n = 0

    def write(self, t, rgb, depth_m, p_cam, q_wxyz):
        import cv2
        ts = "%.6f" % t
        cv2.imwrite(os.path.join(self.root, "rgb", ts + ".png"), rgb[:, :, ::-1])
        d16 = np.clip(depth_m * self.depth_scale, 0, 65535).astype(np.uint16)
        cv2.imwrite(os.path.join(self.root, "depth", ts + ".png"), d16)
        self.f_rgb.write("%s rgb/%s.png\n" % (ts, ts))
        self.f_depth.write("%s depth/%s.png\n" % (ts, ts))
        w, x, y, z = q_wxyz
        self.f_gt.write("%s %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (ts, p_cam[0], p_cam[1], p_cam[2], x, y, z, w))
        self.n += 1
        if self.n % 20 == 0:
            for f in (self.f_rgb, self.f_depth, self.f_gt):
                f.flush()

    def close(self):
        for f in (self.f_rgb, self.f_depth, self.f_gt):
            f.close()
