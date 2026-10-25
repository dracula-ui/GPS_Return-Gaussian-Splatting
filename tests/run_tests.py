#!/usr/bin/env python3
"""Self-contained tests (no ROS, GPU, Gazebo or PX4 needed):   python3 tests/run_tests.py"""
import json
import os
import py_compile
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import xml.dom.minidom as minidom

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG = os.path.join(ROOT, "ros_ws", "src", "rtlf_ros")
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(ROOT, "slam"))
from rtlf_ros import core, geometry as G, protocol as P, scenario as S, sim_params as SP   # noqa: E402


class Geometry(unittest.TestCase):
    def test_rotation_properties(self):
        self.assertAlmostEqual(np.linalg.det(G.R_BC), 1.0)
        np.testing.assert_allclose(G.R_BC @ G.R_BC.T, np.eye(3), atol=1e-12)

    def test_camera_looks_forward(self):
        p, R = G.camera_pose_from_vehicle([0, 0, -15], (1, 0, 0, 0), SP.LEVER_FRD)
        np.testing.assert_allclose(R[:, 2], [1, 0, 0], atol=1e-12)     # optical axis = north at yaw 0
        np.testing.assert_allclose(R[:, 1], [0, 0, 1], atol=1e-12)     # image-down = NED down
        np.testing.assert_allclose(p, [0.12, 0.0, -15.10], atol=1e-12)

    def test_yaw_90_looks_east(self):
        q = (np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4))
        _, R = G.camera_pose_from_vehicle([0, 0, 0], q, SP.LEVER_FRD)
        np.testing.assert_allclose(R[:, 2], [0, 1, 0], atol=1e-9)

    def test_roundtrips(self):
        rng = np.random.default_rng(1)
        for _ in range(20):
            q = rng.normal(size=4); q /= np.linalg.norm(q)
            p = rng.normal(size=3) * 50
            pc, R = G.camera_pose_from_vehicle(p, q, SP.LEVER_FRD)
            np.testing.assert_allclose(G.vehicle_from_camera(pc, q, SP.LEVER_FRD), p, atol=1e-9)
            Rw, Tw = G.w2c_from_cam_pose(pc, R)
            pc2, R2 = G.cam_pose_from_w2c(Rw, Tw)
            np.testing.assert_allclose(pc2, pc, atol=1e-9); np.testing.assert_allclose(R2, R, atol=1e-9)
            q2 = G.R_to_quat(R); np.testing.assert_allclose(G.quat_to_R(*q2), R, atol=1e-9)


class Protocol(unittest.TestCase):
    def test_frame_and_pose(self):
        rgb = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
        depth = np.random.default_rng(1).random((48, 64)).astype(np.float32)
        head, r2, d2 = P.unpack_frame(P.pack_frame({"stamp": 1.5, "init_R": list(np.eye(3).ravel())}, rgb, depth))
        self.assertEqual((head["w"], head["h"], head["stamp"]), (64, 48, 1.5))
        np.testing.assert_array_equal(r2, rgb); np.testing.assert_array_equal(d2, depth)
        d = P.unpack_pose(P.pack_pose(3, 2.0, np.eye(3), [1, 2, 3], True, 9.0))
        self.assertEqual((d["idx"], d["init"]), (3, True)); np.testing.assert_allclose(d["T"], [1, 2, 3])


class MissionEmulated(unittest.TestCase):
    def run_m(self, m, seed):
        return core.run_kinematic(core.Sim(core.make_args(method=m, seed=seed))).result

    def test_b1_collides(self):
        for s in (1, 2, 3):
            r = self.run_m("b1", s); self.assertEqual(r["reason"], "collision"); self.assertFalse(r["success"])

    def test_b3_b4_succeed_and_b4_shorter(self):
        pr = {"b3": [], "b4": []}
        for s in range(1, 6):
            for m in ("b3", "b4"):
                r = self.run_m(m, s); self.assertTrue(r["success"], (m, s, r)); pr[m].append(r["path_ratio"])
        self.assertLess(np.mean(pr["b4"]), np.mean(pr["b3"]))

    def test_trusted_anchor_vs_naive(self):
        a = core.run_kinematic(core.Sim(core.make_args(method="b4", seed=2))).result
        b = core.run_kinematic(core.Sim(core.make_args(method="b4", seed=2, naive_anchor=True))).result
        self.assertLess(a["anchor_bias_m"], 3.0); self.assertGreater(b["anchor_bias_m"], a["anchor_bias_m"])

    def test_route_clear_of_buildings(self):
        for (n0, e0), (n1, e1) in zip(S.ROUTE[:-1], S.ROUTE[1:]):
            for t in np.linspace(0, 1, 200):
                self.assertFalse(S.in_building(n0 + t * (n1 - n0), e0 + t * (e1 - e0), 5.0))


class SlamEstimatorMission(unittest.TestCase):
    """Closed loop with a simulated SLAM (truth + drift, 4 Hz, latency) feeding SlamEstimator."""

    def run_s(self, method, seed, drift=0.012, hz=4.0, stop_after=None):
        clock = {"t": 0.0}
        est = core.SlamEstimator(lambda: clock["t"])
        sim = core.Sim(core.make_args(method=method, seed=seed), est)
        rng = np.random.default_rng(seed)
        pos = np.zeros(3); vel = np.zeros(3); dt = 0.05; err = np.zeros(2); last = 0.0; prev = None
        for k in range(300000):
            clock["t"] += dt
            truth = pos[:2].copy()
            if sim.phase != "CLIMB":
                if prev is not None:
                    err += drift * np.linalg.norm(truth - prev) * rng.normal(size=2) / np.sqrt(2)
                prev = truth.copy()
                if clock["t"] - last >= 1.0 / hz and (stop_after is None or clock["t"] < stop_after):
                    est.update(truth + err, truth); last = clock["t"]
            if sim.phase == "OUTBOUND" and not est.has_pose:
                est.update(truth + err, truth); last = clock["t"]
            cmd = np.array(sim.tick(dt, pos[0], pos[1], -pos[2]))
            vel += (cmd - vel) * min(1.0, dt / 0.4); pos += vel * dt
            if sim.finished:
                break
        return sim.result

    def test_s2_s4_with_real_estimator_interface(self):
        for m in ("s2", "s4"):
            r = self.run_s(m, 3)
            self.assertEqual(r["estimator"], "slam"); self.assertTrue(r["success"], (m, r))
            self.assertEqual(r["collisions"], 0)

    def test_slam_dropout_hovers_then_aborts(self):
        r = self.run_s("s4", 4, stop_after=60.0)         # SLAM poses stop 60 s in -> stale during the return
        self.assertFalse(r["success"]); self.assertEqual(r["reason"], "slam_lost")


class FakeSlamLoop(unittest.TestCase):
    def test_zmq_roundtrip(self):
        import zmq
        from fake_slam_server import FakeSlam
        ctx = zmq.Context()
        fa, po = "tcp://127.0.0.1:5751", "tcp://127.0.0.1:5752"
        pull = ctx.socket(zmq.PULL); pull.setsockopt(zmq.CONFLATE, 1); pull.bind(fa)
        pub = ctx.socket(zmq.PUB); pub.bind(po)
        stop = threading.Event()

        def serve():
            slam = FakeSlam(0.01)
            while not stop.is_set():
                if pull.poll(50):
                    h, _, _ = P.unpack_frame(pull.recv()); pub.send(slam.process(h))
        th = threading.Thread(target=serve, daemon=True); th.start()
        push = ctx.socket(zmq.PUSH); push.connect(fa)
        sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b""); sub.connect(po)
        time.sleep(0.5)
        rgb = np.zeros((48, 64, 3), np.uint8); depth = np.ones((48, 64), np.float32)
        got = []
        for i in range(10):
            p_cam = np.array([i * 1.0, 0.0, -15.0]); _, R_nc = G.camera_pose_from_vehicle([0, 0, 0], (1, 0, 0, 0), SP.LEVER_FRD)
            R, T = G.w2c_from_cam_pose(p_cam, R_nc)
            push.send(P.pack_frame({"stamp": i * 0.1, "init_R": R.ravel().tolist(), "init_T": T.tolist()}, rgb, depth))
            if sub.poll(1000):
                d = P.unpack_pose(sub.recv()); pc, _ = G.cam_pose_from_w2c(d["R"], d["T"]); got.append(np.linalg.norm(pc - p_cam))
        stop.set(); th.join(1); ctx.destroy(linger=0)
        self.assertGreaterEqual(len(got), 8)
        self.assertLess(max(got), 1.0)                  # truth + small drift


class Assets(unittest.TestCase):
    def test_world_generation_valid_xml(self):
        with tempfile.TemporaryDirectory() as d:
            subprocess.run([sys.executable, os.path.join(ROOT, "sim", "make_world.py"), "--out", d], check=True, capture_output=True)
            for rel in ("worlds/%s.sdf" % SP.WORLD_NAME, "models/%s/model.sdf" % SP.MODEL_URI, "server.config"):
                minidom.parse(os.path.join(d, rel))
            w = open(os.path.join(d, "worlds/%s.sdf" % SP.WORLD_NAME)).read()
            self.assertIn(SP.MODEL_NAME, w); self.assertEqual(w.count('name="bldg_'), len(S.BUILDINGS))
            self.assertTrue(os.path.getsize(os.path.join(d, "models/rtlf_textures/materials/textures/ground.png")) > 10000)

    def test_all_python_compiles(self):
        for base, _, files in os.walk(ROOT):
            if "generated" in base or ".venv" in base:
                continue
            for f in files:
                if f.endswith(".py"):
                    py_compile.compile(os.path.join(base, f), doraise=True)

    def test_monogs_patch_applies(self):
        ref = "/tmp/ref/monogs"
        if not os.path.isdir(os.path.join(ref, ".git")):
            self.skipTest("MonoGS clone not present")
        r = subprocess.run(["git", "apply", "--check", os.path.join(ROOT, "slam", "patches", "monogs_ros.patch")], cwd=ref, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
