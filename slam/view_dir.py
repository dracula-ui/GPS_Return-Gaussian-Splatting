#!/usr/bin/env python3
import sys, os, numpy as np
def quat_R(x, y, z, w):
    n = (x*x + y*y + z*z + w*w) ** 0.5; x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
root = os.path.expanduser(sys.argv[1]); step = int(sys.argv[2]) if len(sys.argv) > 2 else 100
L = [l.split() for l in open(os.path.join(root, "groundtruth.txt")).read().splitlines() if l.strip() and not l.startswith("#")]
gt = [[float(v) for v in l[1:8]] for l in L]
print("frame   pos(N,E,D)            optical axis (N,E,D)     travel dir (N,E,D)   cos(axis,travel)")
for i in range(0, len(gt) - 10, step):
    a = np.array(gt[i][:3]); b = np.array(gt[i + 10][:3]); R = quat_R(*gt[i][3:7])
    ax = R[:, 2]; v = b - a; n = np.linalg.norm(v)
    v = v / n if n > 0.05 else np.zeros(3)
    print("%5d  %7.1f %7.1f %6.1f   %6.2f %6.2f %6.2f     %6.2f %6.2f %6.2f     %5.2f" % (i, *a, *ax, *v, float(ax @ v)))
