#!/usr/bin/env python3
"""Pass/fail verdict for a MonoGS run: python3 verdict.py <results folder | trj_final.json> [--table]"""
import argparse, json, os
import numpy as np

def load(path):
    if os.path.isdir(path):
        path = os.path.join(path, "plot", "trj_final.json")
    d = json.load(open(path)); o = np.argsort(d["trj_id"])
    return np.array(d["trj_id"])[o], [np.array(d["trj_est"][i]) for i in o], [np.array(d["trj_gt"][i]) for i in o]

def judge(E, G):
    pe = np.array([m[:3, 3] for m in E]); pg = np.array([m[:3, 3] for m in G])
    gt_len = float(np.sum(np.linalg.norm(np.diff(pg, axis=0), axis=1)))
    est_len = float(np.sum(np.linalg.norm(np.diff(pe, axis=0), axis=1)))
    dg, de = pg[-1] - pg[0], pe[-1] - pe[0]
    progress = float(de @ dg / max(dg @ dg, 1e-9))
    hg, he = dg[:2], de[:2]
    cosang = float(hg @ he / max(np.linalg.norm(hg) * np.linalg.norm(he), 1e-9))
    heading_err = float(np.degrees(np.arccos(np.clip(cosang, -1, 1))))
    final_err = float(np.linalg.norm(pe[-1] - pg[-1]))
    rot = [float(np.degrees(np.arccos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1)))) for a, b in zip(E, G)]
    m = dict(keyframes=len(E), true_path_m=gt_len, est_path_m=est_len, progress_along_truth=progress, heading_error_deg=heading_err,
             final_error_m=final_err, final_error_pct=100 * final_err / max(gt_len, 1e-9), mean_rot_err_deg=float(np.mean(rot)), max_rot_err_deg=float(np.max(rot)))
    if est_len < 0.25 * gt_len:
        v = "FAIL - STUCK: the estimate barely moved (%.0f%% of the true path length)" % (100 * est_len / gt_len)
    elif progress < 0:
        v = "FAIL - BACKWARDS: the estimate moved opposite to the true direction"
    elif heading_err > 30:
        v = "FAIL - WRONG HEADING: the estimate travelled %.0f deg off the true direction" % heading_err
    elif progress < 0.7:
        v = "PARTIAL - right direction but too short: %.0f%% of the true distance" % (100 * progress)
    elif m["final_error_pct"] > 10:
        v = "PARTIAL - right direction and distance but drift is high: %.1f%% of the path" % m["final_error_pct"]
    else:
        v = "PASS - tracks the motion: final error %.1f m = %.1f%% of a %.0f m path" % (final_err, m["final_error_pct"], gt_len)
    return v, m, pe, pg

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("path"); ap.add_argument("--table", action="store_true")
    a = ap.parse_args()
    ids, E, G = load(os.path.expanduser(a.path))
    v, m, pe, pg = judge(E, G)
    if a.table:
        dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pg, axis=0), axis=1))])
        print("kf  frame  dist   gt(N,E,D)                 est(N,E,D)                err   rot_err")
        for k in range(len(E)):
            ang = np.degrees(np.arccos(np.clip((np.trace(E[k][:3, :3].T @ G[k][:3, :3]) - 1) / 2, -1, 1)))
            print("%2d %5d %6.1f  %7.1f %7.1f %6.1f   %7.1f %7.1f %6.1f   %6.1f  %5.1f" % (k, ids[k], dist[k], *pg[k], *pe[k], np.linalg.norm(pe[k] - pg[k]), ang))
    print(json.dumps({k: round(x, 2) if isinstance(x, float) else x for k, x in m.items()}, indent=2))
    print("\nVERDICT:", v)

if __name__ == "__main__":
    main()
