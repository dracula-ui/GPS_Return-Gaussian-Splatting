#!/usr/bin/env python3
"""Idempotent patch of MonoGS utils/slam_frontend.py: optional Training.lock_rotation."""
import sys
p = sys.argv[1]; s = open(p).read()
if "lock_rotation" in s:
    print("already patched"); sys.exit(0)
old = "        viewpoint.update_RT(prev.R, prev.T)\n"
assert s.count(old) == 1, "pattern not found exactly once"
new = old + '''        if self.config["Training"].get("lock_rotation", False):
            _p_prev = -prev.R.T @ prev.T                      # previous camera position in the world
            _R = viewpoint.R_gt.clone().to(prev.R)            # attitude for this frame (from the flight controller)
            viewpoint.update_RT(_R, -_R @ _p_prev)
'''
open(p, "w").write(s.replace(old, new)); print("patched", p)
