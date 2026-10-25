#!/usr/bin/env python3
"""ROS 2 <-> SLAM server bridge.

Subscribes to the RGB-D camera, sends frames to the SLAM server (MonoGS, or fake_slam_server.py) over ZMQ and
republishes the returned pose as geometry_msgs/PoseStamped (camera -> NED world) on /rtlf/slam/camera_pose.
Frames are only sent once the mission node enables SLAM (/rtlf/slam/enable) at cruise altitude, and each frame
carries the true camera pose (from /rtlf/truth/camera_pose). MonoGS uses that pose ONLY to initialise its first
frame (this is what makes the map GNSS-aligned) and for its own offline ATE bookkeeping.
"""
import collections
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from . import geometry as G
from . import protocol as P
from .ros_io import decode_depth, decode_rgb


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class SlamBridge(Node):
    def __init__(self):
        super().__init__("slam_bridge")
        self.declare_parameter("rgb_topic", "/rtlf/rgbd/image")
        self.declare_parameter("depth_topic", "/rtlf/rgbd/depth_image")
        self.declare_parameter("frames_addr", P.DEFAULT_FRAMES)
        self.declare_parameter("poses_addr", P.DEFAULT_POSES)
        self.declare_parameter("send_hz", 10.0)
        import zmq
        self.zmq = zmq
        self.ctx = zmq.Context.instance()
        self.push = self.ctx.socket(zmq.PUSH); self.push.setsockopt(zmq.SNDHWM, 2); self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(self.get_parameter("frames_addr").value)
        self.sub = self.ctx.socket(zmq.SUB); self.sub.setsockopt(zmq.SUBSCRIBE, b""); self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.connect(self.get_parameter("poses_addr").value)

        self.enabled = False
        self.truth = collections.deque(maxlen=400)       # (t, p_cam(3), R_nc(3x3))
        self.last_sent = -1e9
        self.sent = 0; self.recv = 0; self.t0 = time.time()

        self.create_subscription(Bool, "/rtlf/slam/enable", self._on_enable, 10)
        self.create_subscription(PoseStamped, "/rtlf/truth/camera_pose", self._on_truth, 50)
        self.pub_pose = self.create_publisher(PoseStamped, "/rtlf/slam/camera_pose", 10)
        self.pub_status = self.create_publisher(String, "/rtlf/slam/status", 10)
        rgb = Subscriber(self, Image, self.get_parameter("rgb_topic").value, qos_profile=qos_profile_sensor_data)
        dep = Subscriber(self, Image, self.get_parameter("depth_topic").value, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb, dep], queue_size=5, slop=0.03)
        self.sync.registerCallback(self._on_frames)
        self.create_timer(0.02, self._poll_poses)
        self.create_timer(2.0, self._status)
        self.get_logger().info("slam_bridge up: frames->%s poses<-%s" % (self.get_parameter("frames_addr").value, self.get_parameter("poses_addr").value))

    def _on_enable(self, msg):
        if msg.data and not self.enabled:
            self.get_logger().info("SLAM enabled: frames will be sent to the server")
        self.enabled = bool(msg.data)

    def _on_truth(self, msg):
        p = msg.pose.position; o = msg.pose.orientation
        self.truth.append((stamp_s(msg.header.stamp), np.array([p.x, p.y, p.z]), G.quat_to_R(o.w, o.x, o.y, o.z)))

    def _truth_near(self, t):
        if not self.truth:
            return None
        best = min(self.truth, key=lambda x: abs(x[0] - t))
        return best if abs(best[0] - t) < 0.2 else None

    def _on_frames(self, rgb_msg, depth_msg):
        if not self.enabled:
            return
        t = stamp_s(rgb_msg.header.stamp)
        if t - self.last_sent < 1.0 / self.get_parameter("send_hz").value:
            return
        tr = self._truth_near(t)
        if tr is None:
            return
        R, T = G.w2c_from_cam_pose(tr[1], tr[2])
        header = dict(stamp=t, init_R=R.reshape(-1).tolist(), init_T=T.tolist())
        try:
            buf = P.pack_frame(header, decode_rgb(rgb_msg), decode_depth(depth_msg))
            self.push.send(buf, self.zmq.NOBLOCK)
            self.last_sent = t; self.sent += 1
        except self.zmq.Again:
            pass
        except Exception as ex:
            self.get_logger().warn("frame not sent: %s" % ex)

    def _poll_poses(self):
        while self.sub.poll(0):
            d = P.unpack_pose(self.sub.recv())
            p_cam, R_nc = G.cam_pose_from_w2c(d["R"], d["T"])
            q = G.R_to_quat(R_nc)
            m = PoseStamped()
            m.header.frame_id = "ned"
            sec = int(d["stamp"]); m.header.stamp.sec = sec; m.header.stamp.nanosec = int((d["stamp"] - sec) * 1e9)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p_cam)
            m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = map(float, q)
            self.pub_pose.publish(m)
            self.recv += 1

    def _status(self):
        el = max(time.time() - self.t0, 1e-6)
        s = String(); s.data = json.dumps(dict(enabled=self.enabled, frames_sent=self.sent, poses_received=self.recv, wall_s=round(el, 1)))
        self.pub_status.publish(s)


def main():
    rclpy.init()
    node = SlamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
