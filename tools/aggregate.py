#!/usr/bin/env python3
"""Aggregate results/*/result.json into summary.csv and a LaTeX table (mean +- std over seeds).
    python3 tools/aggregate.py results            # copy results/table.tex next to the paper as results_table.tex
"""
import glob
import json
import os
import sys

import numpy as np

root = sys.argv[1] if len(sys.argv) > 1 else "results"
NAMES = {"b1": "B1: IMU dead reckoning", "b2": "B2: VIO-only retrace", "b3": "B3: 3DGS retrace-only (emulated)",
         "b4": "B4: Ours, planner (emulated)", "s2": "S2: retrace, real SLAM", "s4": "S4: Ours, planner, real SLAM"}
rows = {}
for p in sorted(glob.glob(os.path.join(root, "*", "result.json"))):
    r = json.load(open(p))
    if "method" not in r:
        continue
    rows.setdefault(r["method"] + (" [naive anchor]" if r.get("naive_anchor") else ""), []).append(r)


def ms(v):
    return "n/a" if not v else "%.2f $\\pm$ %.2f" % (np.mean(v), np.std(v))


lines, csv = [], ["method,n,success_pct,return_err_mean,return_err_std,path_ratio_mean,drift_pct_mean,collisions"]
for key in sorted(rows):
    R = rows[key]; n = len(R)
    succ = 100.0 * sum(bool(r.get("success")) for r in R) / n
    err = [r["return_err_m"] for r in R if "return_err_m" in r]
    pr = [r["path_ratio"] for r in R if "path_ratio" in r]
    dr = [r["drift_pct"] for r in R if "drift_pct" in r]
    col = sum(r.get("collisions", 0) for r in R)
    name = NAMES.get(key.split()[0], key) + (" [naive anchor]" if "naive" in key else "")
    lines.append("%s & %s & %.0f & %s & %s & %d \\\\" % (name, ms(err), succ, ms(pr), ms(dr), col))
    csv.append("%s,%d,%.1f,%s,%s,%s,%s,%d" % (key, n, succ, np.mean(err) if err else "", np.std(err) if err else "",
                                              np.mean(pr) if pr else "", np.mean(dr) if dr else "", col))
tex = ("\\begin{tabular}{lccccc}\n\\hline\nMethod & Return error (m) & Success (\\%) & Path ratio & Drift (\\%) & Collisions \\\\\n\\hline\n"
       + "\n".join(lines) + "\n\\hline\n\\end{tabular}\n")
open(os.path.join(root, "summary.csv"), "w").write("\n".join(csv) + "\n")
open(os.path.join(root, "table.tex"), "w").write(tex)
print("\n".join(csv)); print("\nwrote", os.path.join(root, "summary.csv"), "and table.tex")
