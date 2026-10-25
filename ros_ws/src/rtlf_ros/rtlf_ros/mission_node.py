#!/usr/bin/env python3
"""Mission node: flies the outbound route, injects the GNSS degradation, then returns to the last trusted fix.

Pose source after GNSS loss
  pose_source:=emulated  parametric drift model (methods b1..b4)           -> no camera/SLAM needed
  pose_source:=slam      real SLAM pose from /rtlf/slam/camera_pose (s2, s4) -> needs slam_bridge + SLAM server

PX4 control is through MAVSDK (offboard velocity), PX4's own estimate is the ground truth (it is never denied;
only the algorithm's view is degraded). Optional record_dir saves a TUM-format RGB-D dataset with ground-truth
camera poses so MonoGS can be run offline to measure its real drift (see slam/analyze_drift.py).
"""
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from . import core
from . import geometry as G
from . import sim_params as SP
from .px4_link import Px4Link
from .ros_io import TumWriter, decode_depth, decode_rgb


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class Mission(Node):
    def __init__(self):
        super().__init__("rtlf_mission")
        dp = self.declare_parameter
        dp("method", "b4"); dp("seed", 1); dp("pose_source", "emulated"); dp("naive_anchor", False)
        dp("eps", 4.0); dp("kappa_scale", 1.0); dp("tol", 3.0); dp("cruise", -1.0); dp("ret_timeout", 300.0)
        dp("slam_frac", 0.03); dp("address", "udpin://0.0.0.0:14540"); dp("out", "results"); dp("tag", "")
        dp("record_dir", ""); dp("record_hz", 10.0)
        dp("rgb_topic", "/rtlf/rgbd/image"); dp("depth_topic", "/rtlf/rgbd/depth_image")
        dp("slam_wait_s", 90.0)
        dp("yaw_mode", "fixed"); dp("yaw_rate", 20.0)
        g = lambda k: self.get_parameter(k).value

        self.slam_mode = g("pose_source") == "slam"
        method = g("method")
        if self.slam_mode and method not in ("s2", "s4"):
            self.get_logger().warn("pose_source:=slam needs method s2 or s4; using s4"); method = "s4"
        if not self.slam_mode and method not in core.EMULATED:
            self.get_logger().warn("emulated pose needs method b1..b4; using b4"); method = "b4"
        self.args = core.make_args(method=method, seed=g("seed"), naive_anchor=g("naive_anchor"), eps=g("eps"),
                                   kappa_scale=g("kappa_scale"), tol=g("tol"), cruise=g("cruise"),
                                   ret_timeout=g("ret_timeout"), slam_frac=g("slam_frac"))
        est = core.SlamEstimator(self.clock_s) if self.slam_mode else None
        self.sim = core.Sim(self.args, est)
        self.est = self.sim.est

        self.link = Px4Link(g("address"), clock=self.clock_s, log=lambda m: self.get_logger().info(m))
        self.link.start()
        self.t_prev = None; self.stage = "wait"; self.done = False; self.t_enable = None
        self.t_next_try = 0.0; self.n_try = 0
        self.yaw_cmd = 0.0
        self.early_exit = False
        self.out = g("out"); self.tag = g("tag") or ("%s%s_s%d" % (method, "_naive" if g("naive_anchor") else "", g("seed")))

        self.pub_truth = self.create_publisher(PoseStamped, "/rtlf/truth/camera_pose", 20)
        self.pub_enable = self.create_publisher(Bool, "/rtlf/slam/enable", 1)
        self.create_subscription(PoseStamped, "/rtlf/slam/camera_pose", self._on_slam, 20)
        self.writer = None; self.last_rec = -1e9
        if g("record_dir"):
            self.writer = TumWriter(g("record_dir"), SP.DEPTH_SCALE_PNG)
            rgb = Subscriber(self, Image, g("rgb_topic"), qos_profile=qos_profile_sensor_data)
            dep = Subscriber(self, Image, g("depth_topic"), qos_profile=qos_profile_sensor_data)
            self.sync = ApproximateTimeSynchronizer([rgb, dep], queue_size=10, slop=0.25)
            self.sync.registerCallback(self._on_images)
            self.get_logger().info("recording TUM dataset to %s" % g("record_dir"))
        self.create_timer(0.05, self.tick)
        self.get_logger().info("mission ready: method=%s pose_source=%s cruise=%.1f m/s" % (method, g("pose_source"), self.sim.cruise))

    # ---- time
    def clock_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- callbacks
    def _on_slam(self, msg):
        t = stamp_s(msg.header.stamp)
        tr = self.link.truth_at(t)
        if tr is None:
            return
        n, e, d, q = tr
        p = msg.pose.position
        p_veh = G.vehicle_from_camera([p.x, p.y, p.z], q, SP.LEVER_FRD)
        self.est.update(p_veh[:2], [n, e])

    def _on_images(self, rgb_msg, depth_msg):
        t = stamp_s(rgb_msg.header.stamp)
        if t - self.last_rec < 1.0 / self.get_parameter("record_hz").value:
            return
        tr = self.link.truth_at(t)
        if tr is None or self.writer is None:
            return
        n, e, d, q = tr
        p_cam, R_nc = G.camera_pose_from_vehicle([n, e, d], q, SP.LEVER_FRD)
        self.writer.write(t, decode_rgb(rgb_msg), decode_depth(depth_msg), p_cam, G.R_to_quat(R_nc))
        self.last_rec = t

    def _publish_truth(self, n, e, d, q):
        p_cam, R_nc = G.camera_pose_from_vehicle([n, e, d], q, SP.LEVER_FRD)
        qq = G.R_to_quat(R_nc)
        m = PoseStamped(); m.header.frame_id = "ned"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p_cam)
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = map(float, qq)
        self.pub_truth.publish(m)

    def _yaw_for(self, vn, ve, dt):
        mode = self.get_parameter("yaw_mode").value
        if mode == "fixed":
            return 0.0
        if float(np.hypot(vn, ve)) > 0.3:
            tgt = float(np.degrees(np.arctan2(ve, vn)))
            if mode == "retrace_back" and self.sim.phase in ("RETURN", "RECOVER"):
                tgt += 180.0
            d = (tgt - self.yaw_cmd + 180.0) % 360.0 - 180.0
            lim = float(self.get_parameter("yaw_rate").value) * dt
            self.yaw_cmd = (self.yaw_cmd + float(np.clip(d, -lim, lim)) + 180.0) % 360.0 - 180.0
        return self.yaw_cmd

    # ---- main loop (20 Hz, sim time)
    def tick(self):
        if self.done:
            return
        now = self.clock_s()
        if self.link.error:
            self.get_logger().error("PX4 link failed: %s" % str(self.link.error).replace("\n", " ")[:200])
            if self.stage == "wait":
                # the MAVSDK helper died before take-off (seen intermittently at arm time): nothing has been flown, so do not
                # try to land; exit at once and let the launcher script start a fresh attempt
                self.get_logger().error("PX4 link failed before take-off; exiting so the launcher can retry")
                self.early_exit = True; self.done = True
                if self.writer:
                    self.writer.close()
                return
            self._finish_and_exit("link_error"); return
        if not self.link.ready:
            return
        tel = self.link.latest()
        if tel is None:
            return
        _, n, e, d_down, qw, qx, qy, qz = tel
        alt = -d_down
        self._publish_truth(n, e, d_down, (qw, qx, qy, qz))

        if self.stage == "wait":
            if now < self.t_next_try:
                return
            self.get_logger().info("arming and starting offboard")
            try:
                self.link.arm_and_offboard(); self.stage = "fly"; self.t_prev = now
            except Exception as ex:
                self.n_try += 1
                self.get_logger().error("arm/offboard failed (%d/20): %s" % (self.n_try, str(ex).replace("\n", " ")[:300]))
                self.t_next_try = now + 5.0
                if self.n_try >= 20:
                    self._finish_and_exit("arm_failed")
            return
        dt = float(np.clip(now - self.t_prev, 1e-3, 0.5)); self.t_prev = now

        sim = self.sim
        if self.slam_mode and sim.phase == "OUTBOUND" and not self.est.has_pose:
            # at cruise altitude: enable SLAM and hover until its first pose arrives (initialised from truth)
            if self.t_enable is None:
                self.t_enable = now; self.get_logger().info("at altitude: enabling SLAM, hovering until first pose")
            self.pub_enable.publish(Bool(data=True))
            vd = float(np.clip(0.8 * (alt - core.S.ALT), -1.5, 1.5))
            self.link.send_velocity(0.0, 0.0, vd)
            if now - self.t_enable > self.get_parameter("slam_wait_s").value:
                self.get_logger().error("no SLAM pose received - is the SLAM server running?")
                sim.result = dict(success=False, reason="slam_not_responding"); self._finish_and_exit("slam_not_responding")
            return

        vn, ve, vd = sim.tick(dt, n, e, alt)
        self.link.send_velocity(vn, ve, vd, self._yaw_for(vn, ve, dt))
        if sim.finished:
            self._finish_and_exit("done")

    def _finish_and_exit(self, why):
        self.done = True
        if self.writer:
            self.writer.close()
        try:
            self.link.land_and_wait()
        except Exception as ex:
            self.get_logger().warn("land: %s" % ex)
        if self.sim.result is None:
            self.sim.result = dict(success=False, reason=why)
        self.sim.result["pose_source"] = self.get_parameter("pose_source").value
        if self.slam_mode:
            self.sim.result["slam_updates"] = self.est.n_updates
        outdir = os.path.join(self.out, self.tag)
        core.save_outputs(self.sim, outdir)
        self.get_logger().info("RESULT " + json.dumps(self.sim.result, default=lambda o: o.item() if hasattr(o, "item") else str(o)))
        self.get_logger().info("saved -> %s" % outdir)


def main():
    rclpy.init()
    node = Mission()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    early = node.early_exit
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if early:
        raise SystemExit(3)


if __name__ == "__main__":
    main()
