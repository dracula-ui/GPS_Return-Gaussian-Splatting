#!/usr/bin/env python3
import os, sys
name, n = sys.argv[1], int(sys.argv[2]); start = int(sys.argv[3]) if len(sys.argv) > 3 else 0
src = os.path.expanduser("~/rtlf_data/" + name); dst = "%s_%d_%d" % (src, start, n)
os.makedirs(dst, exist_ok=True)
for d in ("rgb", "depth"):
    if not os.path.lexists(os.path.join(dst, d)): os.symlink(os.path.join(src, d), os.path.join(dst, d))
def parts(f):
    L = open(os.path.join(src, f)).read().splitlines()
    return [l for l in L if l.startswith("#")], [l for l in L if l and not l.startswith("#")]
hr, rgb = parts("rgb.txt"); hd, dep = parts("depth.txt"); hg, gt = parts("groundtruth.txt")
rgb, dep = rgb[start:start + n], dep[start:start + n]
t0, t1 = float(rgb[0].split()[0]) - 0.5, float(rgb[-1].split()[0]) + 0.5
gt = [l for l in gt if t0 <= float(l.split()[0]) <= t1]
for f, h, d in (("rgb.txt", hr, rgb), ("depth.txt", hd, dep), ("groundtruth.txt", hg, gt)):
    open(os.path.join(dst, f), "w").write("\n".join(h + d) + "\n")
print(dst, len(rgb), "frames")
