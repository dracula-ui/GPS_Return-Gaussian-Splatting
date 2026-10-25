"""Live dataset for MonoGS that receives RGB-D frames from the ROS slam_bridge_node over ZMQ and publishes the
tracked camera poses back. Copied into MonoGS/utils/ by slam/install_monogs.sh (with rtlf_protocol.py).

MonoGS initialises its first frame at the pose supplied with that frame, so the map starts in the vehicle's local
(GNSS-aligned) NED frame. Later poses are tracked by MonoGS alone; the supplied poses are only used by MonoGS for
its own ATE bookkeeping, never for tracking.
"""
import signal
import time

import numpy as np
import torch

from gaussian_splatting.utils.graphics_utils import focal2fov
from utils import rtlf_protocol as P
from utils.dataset import BaseDataset


class RosDataset(BaseDataset):
    def __init__(self, args, path, config):
        super().__init__(args, path, config)
        import zmq
        ds = config["Dataset"]
        cal = ds["Calibration"]
        self.fx, self.fy, self.cx, self.cy = cal["fx"], cal["fy"], cal["cx"], cal["cy"]
        self.width, self.height = int(cal["width"]), int(cal["height"])
        self.fovx = focal2fov(self.fx, self.width)
        self.fovy = focal2fov(self.fy, self.height)
        self.K = np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])
        self.has_depth = True
        self.disorted = False
        self.num_imgs = 10 ** 9
        self.stop = False
        self._last = None
        self.stamps = {}

        ctx = zmq.Context.instance()
        self.sock_frames = ctx.socket(zmq.PULL)
        self.sock_frames.setsockopt(zmq.CONFLATE, 1)          # keep only the newest frame
        self.sock_frames.bind(ds.get("zmq_frames", P.DEFAULT_FRAMES))
        self.sock_poses = ctx.socket(zmq.PUB)
        self.sock_poses.bind(ds.get("zmq_poses", P.DEFAULT_POSES))
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)
        print("[RosDataset] waiting for frames on %s, publishing poses on %s" % (ds.get("zmq_frames", P.DEFAULT_FRAMES), ds.get("zmq_poses", P.DEFAULT_POSES)), flush=True)

    def _on_signal(self, *_):
        print("[RosDataset] stop requested: finishing and saving results", flush=True)
        self.stop = True
        self.num_imgs = 0

    def __getitem__(self, idx):
        msg = None
        while msg is None:
            if self.stop:
                msg = self._last
                break
            if self.sock_frames.poll(200):
                msg = self.sock_frames.recv()
                self._last = msg
        if msg is None:                                         # stopped before any frame: return a black dummy
            rgb = np.zeros((self.height, self.width, 3), np.uint8); depth = np.zeros((self.height, self.width), np.float32)
            head = dict(stamp=0.0, init_R=np.eye(3).reshape(-1).tolist(), init_T=[0.0, 0.0, 0.0])
        else:
            head, rgb, depth = P.unpack_frame(msg)
        R = np.array(head["init_R"]).reshape(3, 3)
        T = np.array(head["init_T"])
        pose = np.eye(4); pose[:3, :3] = R; pose[:3, 3] = T      # world-to-camera, used only for frame 0 / ATE
        image = (torch.from_numpy(np.asarray(rgb) / 255.0).clamp(0.0, 1.0).permute(2, 0, 1)
                 .to(device=self.device, dtype=self.dtype))
        self.stamps[idx] = head["stamp"]
        return image, np.array(depth, dtype=np.float32), torch.from_numpy(pose).to(device=self.device)

    def on_pose(self, idx, viewpoint, init=False):
        """Called by the patched FrontEnd after each frame is initialised/tracked."""
        R = viewpoint.R.detach().cpu().numpy()
        T = viewpoint.T.detach().cpu().numpy()
        self.sock_poses.send(P.pack_pose(idx, self.stamps.get(idx, 0.0), R, T, init, time.time()))
