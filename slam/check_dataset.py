#!/usr/bin/env python3
"""Check a recorded TUM dataset without SLAM: warp frame i into frame j with depth + GT poses and compare colours."""
import argparse, os, sys
import cv2
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros_ws", "src", "rtlf_ros"))
from rtlf_ros import sim_params as P

def quat_R(x, y, z, w):
    n = (x*x + y*y + z*z + w*w) ** 0.5; x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def load(root):
    def lines(f):
        return [l.split() for l in open(os.path.join(root, f)).read().splitlines() if l.strip() and not l.startswith("#")]
    rgb = {l[0]: l[1] for l in lines("rgb.txt")}; dep = {l[0]: l[1] for l in lines("depth.txt")}
    gt = {l[0]: [float(v) for v in l[1:8]] for l in lines("groundtruth.txt")}
    return [t for t in rgb if t in dep and t in gt], rgb, dep, gt

def frame(root, t, rgb, dep, gt):
    img = cv2.imread(os.path.join(root, rgb[t]), cv2.IMREAD_GRAYSCALE).astype(np.float32)
    d = cv2.imread(os.path.join(root, dep[t]), cv2.IMREAD_UNCHANGED).astype(np.float32) / P.DEPTH_SCALE_PNG
    g = gt[t]
    return img, d, np.array(g[:3]), quat_R(*g[3:7])

def warp_error(fi, fj, K):
    img_i, d_i, p_i, R_i = fi; img_j, _, p_j, R_j = fj
    h, w = img_i.shape; v, u = np.mgrid[0:h, 0:w].astype(np.float32)
    ok = (d_i > 0.3) & (d_i < 200.0)
    X_c = np.stack([(u-K[0,2])/K[0,0]*d_i, (v-K[1,2])/K[1,1]*d_i, d_i], -1)
    X_j = ((X_c @ R_i.T + p_i) - p_j) @ R_j
    z = X_j[..., 2]; ok &= z > 0.3
    zz = np.where(z > 0.3, z, 1)
    uj = K[0,0]*X_j[...,0]/zz + K[0,2]; vj = K[1,1]*X_j[...,1]/zz + K[1,2]
    ok &= (uj >= 0) & (uj <= w-1) & (vj >= 0) & (vj <= h-1)
    if ok.sum() < 200: return None
    samp = cv2.remap(img_j, uj.astype(np.float32), vj.astype(np.float32), cv2.INTER_LINEAR)
    return (float(np.mean(np.abs(samp[ok]-img_i[ok]))), float(np.mean(np.abs(img_j[ok]-img_i[ok]))),
            float(np.median(np.hypot(uj[ok]-u[ok], vj[ok]-v[ok]))))

ap = argparse.ArgumentParser(); ap.add_argument("root"); ap.add_argument("--frames", type=int, nargs="*"); ap.add_argument("--gap", type=int, nargs="*", default=[1, 5])
a = ap.parse_args(); root = os.path.expanduser(a.root)
ts, rgb, dep, gt = load(root)
K = np.array([[P.FX, 0, P.CX], [0, P.FY, P.CY], [0, 0, 1.0]])
print("frames:", len(ts), "camera %dx%d fx=%.1f depth_scale=%g" % (P.WIDTH, P.HEIGHT, P.FX, P.DEPTH_SCALE_PNG))
idx = a.frames or [int(len(ts)*f) for f in (0.1, 0.3, 0.5, 0.7)]
for i in idx:
    d = cv2.imread(os.path.join(root, dep[ts[i]]), cv2.IMREAD_UNCHANGED).astype(np.float32) / P.DEPTH_SCALE_PNG
    print("frame %d: image %s, depth valid (0.3-200 m) %.0f%%, zero %.0f%%, median %.1f m, max %.1f m, alt %.1f m" % (
        i, cv2.imread(os.path.join(root, rgb[ts[i]])).shape, 100*np.mean((d > .3) & (d < 200)), 100*np.mean(d == 0),
        np.median(d[(d > .3) & (d < 200)]) if np.any((d > .3) & (d < 200)) else -1, d.max(), -gt[ts[i]][2]))
print("\nframe gap moved(m)  shift(px)  err_warped err_unwarped ratio")
for i in idx:
    for g in a.gap:
        j = i + g
        if j >= len(ts): continue
        fi, fj = frame(root, ts[i], rgb, dep, gt), frame(root, ts[j], rgb, dep, gt)
        r = warp_error(fi, fj, K); mv = np.linalg.norm(fi[2]-fj[2])
        print("%5d %3d  %6.2f   " % (i, g, mv) + ("(too few valid pixels)" if r is None else "%7.1f   %8.2f   %8.2f   %5.2f" % (r[2], r[0], r[1], r[0]/max(r[1], 1e-6))))
print("\nratio << 1 (< 0.6): consistent.  ratio ~ 1: poses/intrinsics/depth do not match the images.")
