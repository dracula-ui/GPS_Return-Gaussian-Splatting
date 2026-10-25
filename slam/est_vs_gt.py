#!/usr/bin/env python3
import json, sys, numpy as np
d = json.load(open(sys.argv[1]))
est = np.array([np.array(m)[:3, 3] for m in d["trj_est"]]); gt = np.array([np.array(m)[:3, 3] for m in d["trj_gt"]])
o = np.argsort(d["trj_id"]); est, gt = est[o], gt[o]; ids = np.array(d["trj_id"])[o]
dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(gt, axis=0), axis=1))])
print("kf  frame  dist   gt(N,E,D)                 est(N,E,D)                err")
for k in range(len(gt)):
    print("%2d %5d %6.1f  %7.1f %7.1f %6.1f   %7.1f %7.1f %6.1f   %6.1f" % (k, ids[k], dist[k], *gt[k], *est[k], np.linalg.norm(est[k]-gt[k])))
