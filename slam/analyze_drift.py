#!/usr/bin/env python3
"""Measure the real SLAM drift from a MonoGS run and suggest planner parameters.

    python3 analyze_drift.py <MonoGS results dir>/plot/trj_final.json  [--out drift.json]

MonoGS starts the map at the ground-truth pose of frame 0, so estimate and ground truth share a frame and the
(unaligned) position error is a true drift measurement. Reports error vs. distance travelled, the best-fit drift
rate kappa [m per m] (least squares through the origin), and drift as a percentage of distance.
"""
import argparse
import json

import numpy as np


def analyze(path):
    d = json.load(open(path))
    est = np.array([np.array(m)[:3, 3] for m in d["trj_est"]])
    gt = np.array([np.array(m)[:3, 3] for m in d["trj_gt"]])
    order = np.argsort(d["trj_id"])
    est, gt = est[order], gt[order]
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt, axis=0), axis=1))])
    err = np.linalg.norm(est - gt, axis=1)
    m = dist > 5.0
    kappa = float(np.sum(err[m] * dist[m]) / np.sum(dist[m] ** 2)) if m.any() else float("nan")
    return dict(n_keyframes=int(len(gt)), path_length_m=float(dist[-1]), final_error_m=float(err[-1]),
                max_error_m=float(err.max()), rmse_m=float(np.sqrt(np.mean(err ** 2))),
                kappa_m_per_m=kappa, drift_pct=100.0 * kappa,
                note="kappa is the least-squares slope of error vs distance; planner NOMINAL_KAPPA is (0.01, 0.03) "
                     "(textured, sparse) so --kappa-scale ~ kappa/0.01 for textured terrain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trj")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    r = analyze(a.trj)
    print(json.dumps(r, indent=2))
    if a.out:
        json.dump(r, open(a.out, "w"), indent=2)


if __name__ == "__main__":
    main()
