#!/usr/bin/env python3
"""Stand-in for the MonoGS server (same ZMQ protocol, no GPU, no CUDA).

It returns the true camera pose that the bridge attaches to each frame plus a random-walk drift
(--drift metres per metre travelled). Use it to test the whole ROS pipeline (bridge, mission, s2/s4 methods)
before MonoGS is installed. It is NOT a SLAM system.

    python3 fake_slam_server.py [--hz 4] [--drift 0.015]
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros_ws", "src", "rtlf_ros"))
from rtlf_ros import geometry as G      # noqa: E402
from rtlf_ros import protocol as P      # noqa: E402


class FakeSlam:
    def __init__(self, drift=0.015, seed=0):
        self.drift, self.rng = drift, np.random.default_rng(seed)
        self.err = np.zeros(3); self.prev = None; self.idx = 0

    def process(self, head):
        R = np.array(head["init_R"]).reshape(3, 3); T = np.array(head["init_T"])
        p_true, R_nc = G.cam_pose_from_w2c(R, T)
        if self.prev is not None:
            d = float(np.linalg.norm(p_true - self.prev))
            self.err += self.drift * d * self.rng.normal(size=3) / np.sqrt(3) + self.rng.normal(0, 0.003 * d, 3)
        self.prev = p_true
        R_w2c, T_w2c = G.w2c_from_cam_pose(p_true + self.err, R_nc)
        init = self.idx == 0
        out = P.pack_pose(self.idx, head["stamp"], R_w2c, T_w2c, init, time.time())
        self.idx += 1
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default=P.DEFAULT_FRAMES)
    ap.add_argument("--poses", default=P.DEFAULT_POSES)
    ap.add_argument("--hz", type=float, default=4.0)
    ap.add_argument("--drift", type=float, default=0.015)
    a = ap.parse_args()
    import zmq
    ctx = zmq.Context.instance()
    pull = ctx.socket(zmq.PULL); pull.setsockopt(zmq.CONFLATE, 1); pull.bind(a.frames)
    pub = ctx.socket(zmq.PUB); pub.bind(a.poses)
    slam = FakeSlam(a.drift)
    print("fake SLAM server: frames %s, poses %s, %.1f Hz, drift %.3f m/m" % (a.frames, a.poses, a.hz, a.drift), flush=True)
    last = 0.0
    try:
        while True:
            if pull.poll(200):
                head, _rgb, _depth = P.unpack_frame(pull.recv())
                pub.send(slam.process(head))
                time.sleep(max(0.0, 1.0 / a.hz - (time.time() - last))); last = time.time()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
