"""Wire format between the ROS-side slam_bridge_node and the SLAM server (MonoGS or the fake server).

Frames   : bridge --PUSH--> server (PULL, CONFLATE: only the newest frame is kept)
           one message = <u32 header_len><json header><rgb uint8 HxWx3><depth float32 HxW (metres, 0 = invalid)>
Poses    : server --PUB--> bridge (SUB)
           one message = json {"idx","stamp","R":[9],"T":[3],"init":bool,"t_wall":float}
           R, T are MonoGS world-to-camera: X_cam = R X_world + T
Pure python + numpy + json so it can be imported inside the MonoGS conda environment (python 3.7+).
"""
import json
import struct

import numpy as np

DEFAULT_FRAMES = "tcp://127.0.0.1:5555"
DEFAULT_POSES = "tcp://127.0.0.1:5556"


def pack_frame(header, rgb, depth):
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    depth = np.ascontiguousarray(depth, dtype=np.float32)
    h, w = depth.shape
    if rgb.shape != (h, w, 3):
        raise ValueError("rgb shape %s does not match depth %s" % (rgb.shape, depth.shape))
    head = dict(header)
    head["w"], head["h"] = int(w), int(h)
    hb = json.dumps(head).encode()
    return struct.pack("<I", len(hb)) + hb + rgb.tobytes() + depth.tobytes()


def unpack_frame(buf):
    (n,) = struct.unpack("<I", buf[:4])
    head = json.loads(buf[4:4 + n].decode())
    w, h = head["w"], head["h"]
    off = 4 + n
    rgb = np.frombuffer(buf, dtype=np.uint8, count=h * w * 3, offset=off).reshape(h, w, 3)
    off += h * w * 3
    depth = np.frombuffer(buf, dtype=np.float32, count=h * w, offset=off).reshape(h, w)
    return head, rgb, depth


def pack_pose(idx, stamp, R, T, init=False, t_wall=0.0):
    return json.dumps(dict(idx=int(idx), stamp=float(stamp), R=[float(x) for x in np.asarray(R).reshape(-1)],
                           T=[float(x) for x in np.asarray(T).reshape(-1)], init=bool(init), t_wall=float(t_wall))).encode()


def unpack_pose(buf):
    d = json.loads(buf.decode())
    d["R"] = np.array(d["R"]).reshape(3, 3)
    d["T"] = np.array(d["T"])
    return d
