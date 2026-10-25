#!/usr/bin/env python3
"""Write MonoGS configs (live ROS mode and offline TUM mode) with the simulated camera intrinsics.
    python3 make_configs.py --monogs ~/MonoGS [--dataset datasets/sim/run1] [--lock-rotation]
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros_ws", "src", "rtlf_ros"))
from rtlf_ros import sim_params as P   # noqa: E402

CAL = """  Calibration:
    fx: {fx:.6f}
    fy: {fy:.6f}
    cx: {cx:.6f}
    cy: {cy:.6f}
    k1: 0.0
    k2: 0.0
    p1: 0.0
    p2: 0.0
    k3: 0.0
    distorted: False
    width: {w}
    height: {h}
    depth_scale: {ds}
"""

TRAIN = """  alpha: {alpha}
  lock_rotation: {lock}
  motion_prior: {mp}
  force_kf_every: {fk}
  lr:
    cam_rot_delta: {rlr}
    cam_trans_delta: {tlr}
"""

LIVE = """inherit_from: "configs/rgbd/tum/base_config.yaml"

Results:
  save_results: True
  save_dir: "results"
  save_trj: False
  use_gui: False
  eval_rendering: False
  use_wandb: False

Dataset:
  type: "ros"
  sensor_type: "depth"
  dataset_path: "datasets/sim/live"
  zmq_frames: "tcp://127.0.0.1:5555"
  zmq_poses: "tcp://127.0.0.1:5556"
{cal}
Training:
  tracking_itr_num: {itr}
  kf_interval: 5
  window_size: 8
""" + TRAIN

OFFLINE = """inherit_from: "configs/rgbd/tum/base_config.yaml"

Results:
  save_results: True
  save_dir: "results"
  save_trj: False
  use_gui: False
  eval_rendering: False
  use_wandb: False

Dataset:
  type: "tum"
  sensor_type: "depth"
  dataset_path: "{dataset}"
{cal}
Training:
""" + TRAIN


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--monogs", required=True)
    ap.add_argument("--dataset", default="datasets/sim/run1")
    ap.add_argument("--itr", type=int, default=100, help="tracking iterations per frame (lower = faster)")
    ap.add_argument("--trans-lr", type=float, default=0.01)
    ap.add_argument("--alpha", type=float, default=0.5)
    ap.add_argument("--lock-rotation", action="store_true", default=os.environ.get("LOCK_ROT") == "1")
    ap.add_argument("--motion-prior", action="store_true", default=os.environ.get("MOTION") == "1")
    ap.add_argument("--force-kf", type=int, default=int(os.environ.get("FORCE_KF", "0")))
    a = ap.parse_args()
    lock = bool(a.lock_rotation)
    tr = dict(alpha=a.alpha, lock=lock, mp=bool(a.motion_prior), fk=a.force_kf, rlr=0.0 if lock else 0.003, tlr=a.trans_lr)
    out = os.path.join(a.monogs, "configs", "sim"); os.makedirs(out, exist_ok=True)
    cal = lambda ds: CAL.format(fx=P.FX, fy=P.FY, cx=P.CX, cy=P.CY, w=P.WIDTH, h=P.HEIGHT, ds=ds)
    open(os.path.join(out, "sim_live.yaml"), "w").write(LIVE.format(cal=cal(1.0), itr=a.itr, **tr))
    open(os.path.join(out, "sim_offline.yaml"), "w").write(OFFLINE.format(cal=cal(P.DEPTH_SCALE_PNG), dataset=a.dataset, **tr))
    print("wrote", out, "(rotation locked)" if lock else "")


if __name__ == "__main__":
    main()
