#!/usr/bin/env python3
"""Runs mission_node.Mission end to end against stubbed rclpy/message types and a fake PX4 link
(kinematic quadcopter). No ROS needed. Catches wiring/frame errors in the node code, including the
camera-pose <-> vehicle-pose conversions used for real SLAM.   python3 tests/test_nodes_stub.py"""
import json
import os
import sys
import tempfile
import types
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "rtlf_ros"))

# ---------------------------------------------------------------- stubs
class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)

def _stamp(t=0.0):
    return Obj(sec=int(t), nanosec=int((t - int(t)) * 1e9))

class PoseStamped:
    def __init__(self):
        self.header = Obj(stamp=_stamp(), frame_id="")
        self.pose = Obj(position=Obj(x=0.0, y=0.0, z=0.0), orientation=Obj(w=1.0, x=0.0, y=0.0, z=0.0))

class Bool:
    def __init__(self, data=False):
        self.data = data

class Clock:
    def __init__(self):
        self.t = 0.0
    def now(self):
        t = self.t
        return Obj(nanoseconds=int(t * 1e9), to_msg=lambda: _stamp(t))

class Param:
    def __init__(self, v):
        self.value = v

class Node:
    def __init__(self, name):
        self._params = {}; self._clock = Clock(); self.published = {}
    def declare_parameter(self, k, v):
        self._params[k] = v
    def get_parameter(self, k):
        return Param(self._params[k])
    def create_publisher(self, typ, topic, q):
        self.published[topic] = []
        return Obj(publish=lambda m, t=topic: self.published[t].append(m))
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def get_clock(self): return self._clock
    def get_logger(self):
        return Obj(info=lambda m: None, warn=lambda m: None, error=lambda m: print("ERR", m))
    def destroy_node(self): pass

def install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
    mod("rclpy"); mod("rclpy.node", Node=Node); mod("rclpy.qos", qos_profile_sensor_data=None)
    mod("geometry_msgs"); mod("geometry_msgs.msg", PoseStamped=PoseStamped)
    mod("sensor_msgs"); mod("sensor_msgs.msg", Image=object)
    mod("std_msgs"); mod("std_msgs.msg", Bool=Bool, String=object)
    mod("message_filters", Subscriber=lambda *a, **k: None, ApproximateTimeSynchronizer=lambda *a, **k: Obj(registerCallback=lambda f: None))

install_stubs()
from rtlf_ros import mission_node as MN       # noqa: E402
from rtlf_ros import geometry as G, sim_params as SP   # noqa: E402


class FakeLink:
    """Kinematic quadcopter with first-order velocity response; level attitude, yaw 0."""
    def __init__(self, clock):
        self.clock = clock; self.error = None; self.ready = True
        self.pos = np.array([0.0, 0.0, 0.0]); self.vel = np.zeros(3); self.cmd = np.zeros(3)
        self.hist = []; self.armed = False; self.landed = False
    def start(self): pass
    def step(self, t, dt):
        if self.armed:
            self.vel += (self.cmd - self.vel) * min(1.0, dt / 0.4); self.pos += self.vel * dt
        self.pos[2] = min(self.pos[2], 0.0)
        self.hist.append((t, *self.pos, 1.0, 0.0, 0.0, 0.0))
    def latest(self): return self.hist[-1] if self.hist else None
    def truth_at(self, t):
        r = min(self.hist, key=lambda x: abs(x[0] - t)); return r[1], r[2], r[3], r[4:8]
    def arm_and_offboard(self): self.armed = True
    def send_velocity(self, vn, ve, vd, yaw_deg=0.0): self.cmd = np.array([vn, ve, vd])
    def land_and_wait(self, timeout=0): self.landed = True


def run_node(method, pose_source, seed=1, drift=0.0, slam_hz=4.0, max_s=2000.0):
    out = tempfile.mkdtemp()
    node = MN.Mission.__new__(MN.Mission)
    Node.__init__(node, "rtlf_mission")
    # replicate Mission.__init__ with the fake link
    orig_link = MN.Px4Link
    MN.Px4Link = lambda *a, **k: FakeLink(node._clock)
    try:
        MN.Mission.__init__(node)   # declares defaults; we then override params and rebuild
    finally:
        MN.Px4Link = orig_link
    # rebuild with the requested parameters
    node._params.update(method=method, pose_source=pose_source, seed=seed, out=out)
    node.slam_mode = pose_source == "slam"
    from rtlf_ros import core
    args = core.make_args(method=method, seed=seed)
    est = core.SlamEstimator(node.clock_s) if node.slam_mode else None
    node.sim = core.Sim(args, est); node.est = node.sim.est
    node.link = FakeLink(node._clock); node.tag = "t"; node.out = out
    node.t_prev = None; node.stage = "wait"; node.done = False; node.t_enable = None
    rng = np.random.default_rng(seed); err = np.zeros(3); prev = None; last = -1.0
    dt = 0.05; t = 0.0
    while not node.done and t < max_s:
        t += dt; node._clock.t = t
        node.link.step(t, dt)
        node.tick()
        enabled = any(getattr(m, "data", False) for m in node.published.get("/rtlf/slam/enable", []))
        if node.slam_mode and enabled and node.published.get("/rtlf/truth/camera_pose") and t - last >= 1.0 / slam_hz:
            tr = node.published["/rtlf/truth/camera_pose"][-1]
            p = np.array([tr.pose.position.x, tr.pose.position.y, tr.pose.position.z])
            if prev is not None:
                err += drift * np.linalg.norm(p - prev) * rng.normal(size=3) / np.sqrt(3)
            prev = p
            m = PoseStamped(); m.header.stamp = tr.header.stamp
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p + err
            o = tr.pose.orientation; m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = o.w, o.x, o.y, o.z
            node._on_slam(m); last = t
    return node, out, t


class NodeTests(unittest.TestCase):
    def test_emulated_b4_succeeds_and_writes_outputs(self):
        node, out, t = run_node("b4", "emulated", seed=2)
        self.assertTrue(node.done); self.assertTrue(node.link.landed)
        r = json.load(open(os.path.join(out, "t", "result.json")))
        self.assertTrue(r["success"], r); self.assertTrue(os.path.exists(os.path.join(out, "t", "plot.png")))

    def test_slam_geometry_chain_zero_drift(self):
        # camera pose published from truth -> converted back to vehicle position must equal truth
        node, out, t = run_node("s4", "slam", seed=3, drift=0.0)
        r = json.load(open(os.path.join(out, "t", "result.json")))
        self.assertTrue(r["success"], r)
        self.assertLess(r["est_err_max_m"], 0.5, r)          # lever arm / frame conversions are consistent

    def test_slam_small_drift_s2_and_s4(self):
        for m in ("s2", "s4"):
            node, out, t = run_node(m, "slam", seed=5, drift=0.01)
            r = json.load(open(os.path.join(out, "t", "result.json")))
            self.assertEqual(r["collisions"], 0, r); self.assertEqual(r["pose_source"], "slam")

    def test_slam_never_responds(self):
        node, out, t = run_node("s4", "slam", seed=1, slam_hz=1e-9, max_s=200.0)
        self.assertTrue(node.done)
        r = json.load(open(os.path.join(out, "t", "result.json")))
        self.assertEqual(r["reason"], "slam_not_responding")


class BridgeTests(unittest.TestCase):
    def test_bridge_roundtrip_with_fake_server(self):
        import threading, time, zmq
        from rtlf_ros import protocol as P, slam_bridge_node as SB
        sys.path.insert(0, os.path.join(ROOT, "slam"))
        from fake_slam_server import FakeSlam
        P.DEFAULT_FRAMES, P.DEFAULT_POSES = "tcp://127.0.0.1:5851", "tcp://127.0.0.1:5852"
        ctx = zmq.Context(); pull = ctx.socket(zmq.PULL); pull.setsockopt(zmq.CONFLATE, 1); pull.bind(P.DEFAULT_FRAMES)
        pub = ctx.socket(zmq.PUB); pub.bind(P.DEFAULT_POSES); stop = threading.Event()
        def serve():
            slam = FakeSlam(0.0)
            try:
                while not stop.is_set():
                    if pull.poll(50):
                        h, _, _ = P.unpack_frame(pull.recv()); pub.send(slam.process(h))
            except zmq.ZMQError:
                pass
        threading.Thread(target=serve, daemon=True).start()
        b = SB.SlamBridge(); time.sleep(0.5)
        # truth camera pose at t=1.0
        tr = PoseStamped(); tr.header.stamp = _stamp(1.0); tr.pose.position.x, tr.pose.position.y, tr.pose.position.z = 5.0, 2.0, -15.0
        b._on_truth(tr)
        def img(enc, data, step):
            return Obj(height=48, width=64, step=step, encoding=enc, data=data, header=Obj(stamp=_stamp(1.0)))
        rgb = img("rgb8", np.zeros((48, 64, 3), np.uint8).tobytes(), 64 * 3)
        dep = img("32FC1", np.full((48, 64), 7.0, np.float32).tobytes(), 64 * 4)
        b._on_frames(rgb, dep); self.assertEqual(b.sent, 0)            # not enabled yet -> nothing sent
        b._on_enable(Bool(True)); b._on_frames(rgb, dep); self.assertEqual(b.sent, 1)
        for _ in range(100):
            b._poll_poses()
            if b.recv: break
            time.sleep(0.05)
        stop.set(); ctx.destroy(linger=0)
        self.assertEqual(b.recv, 1)
        m = b.published["/rtlf/slam/camera_pose"][0]
        np.testing.assert_allclose([m.pose.position.x, m.pose.position.y, m.pose.position.z], [5.0, 2.0, -15.0], atol=1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
