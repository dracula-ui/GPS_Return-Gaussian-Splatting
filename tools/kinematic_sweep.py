#!/usr/bin/env python3
"""Many-seed sweep of the emulated methods (b1-b4, b4 with naive anchor) on the kinematic stand-in for PX4/Gazebo.
Same mission state machine and planner as the Gazebo runs (core.Sim); only the vehicle dynamics are simplified.
    python3 tools/kinematic_sweep.py [n_seeds=30] [kappa_scale ...]      -> results_kin/k<scale>/<tag>/result.json + table.tex
"""
import json, os, subprocess, sys
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "rtlf_ros"))
from rtlf_ros import core


def one(job):
    ks, m, naive, seed = job
    sim = core.run_kinematic(core.Sim(core.make_args(method=m, seed=seed, kappa_scale=ks, naive_anchor=naive)))
    r = dict(sim.result); r["kappa_scale"] = ks
    tag = "%s%s_s%d" % (m, "_naive" if naive else "", seed)
    d = os.path.join(ROOT, "results_kin", "k%g" % ks, tag); os.makedirs(d, exist_ok=True)
    json.dump(r, open(os.path.join(d, "result.json"), "w"), indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    return tag


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    scales = [float(x) for x in sys.argv[2:]] or [1.0, 4.0]
    jobs = [(ks, m, nv, s) for ks in scales for s in range(1, n + 1)
            for m, nv in (("b1", False), ("b2", False), ("b3", False), ("b4", False), ("b4", True))]
    with Pool() as p:
        p.map(one, jobs, chunksize=4)
    for ks in scales:
        print("\n=== kappa_scale %g (%d seeds) ===" % (ks, n))
        subprocess.run([sys.executable, os.path.join(HERE, "aggregate.py"), os.path.join(ROOT, "results_kin", "k%g" % ks)], check=True)
