#!/usr/bin/env python3
import sys, json, numpy as np
d = json.load(open(sys.argv[1])); o = np.argsort(d["trj_id"]); ids = np.array(d["trj_id"])[o]
E = [np.array(d["trj_est"][i]) for i in o]; G = [np.array(d["trj_gt"][i]) for i in o]
def hdg(R): a = R[:3, 2]; return np.degrees(np.arctan2(a[1], a[0]))      # heading of the optical axis (deg from north)
print("kf frame  gt_heading est_heading  rot_err_deg   gt_axis_down est_axis_down")
for k in range(len(E)):
    Re, Rg = E[k][:3, :3], G[k][:3, :3]
    ang = np.degrees(np.arccos(np.clip((np.trace(Re.T @ Rg) - 1) / 2, -1, 1)))
    print("%2d %5d   %8.1f   %8.1f     %6.1f        %6.2f      %6.2f" % (k, ids[k], hdg(Rg), hdg(Re), ang, Rg[2, 2], Re[2, 2]))
