#!/usr/bin/env python3
"""Paper figures from your own result files. Run from the repo root (~/Project_APT/rtlf_ros).

  # Fig. 4: B3/B4 success vs drift scale, from results_kin/k*/ (python3 tools/kinematic_sweep.py 30 1.0 2.0 4.0 6.4 10.0)
  python3 tools/paper_figs.py sweep --out fig_sweep.pdf

  # Fig. 1: mission map. --run uses results/<tag>/log.csv from a Gazebo run; --kin re-runs the kinematic stand-in
  python3 tools/paper_figs.py map --run results_gz/b3_s1:B3 --run results_gz/b4_s1:B4 --kin b1:1:1.0:B1 --out fig_map.pdf

  # Fig. 2: MonoGS tracking. One --stretch per recording: <MonoGS results folder>:<label>[:<distance where tracking was lost, m>]
  python3 tools/paper_figs.py tracking \
      --stretch ~/MonoGS/results/datasets_sim/2026-10-21-18-38-52:run9 \
      --stretch ~/MonoGS/results/datasets_sim/2026-10-21-19-57-03:east:54 \
      --stretch ~/MonoGS/results/datasets_sim/2026-10-21-20-31-59:north:32 \
      --stretch <west folder>:west --stretch <return folder>:return --out fig_tracking.pdf

  # Fig. 3: failure frames, from the two montages saved earlier
  python3 tools/paper_figs.py frames --north ~/fail_north760.png --east ~/fail_east160.png --out fig_frames.pdf

Every number comes from the files you point it at; nothing is typed in.
"""
import argparse, csv, glob, json, math, os, sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "ros_ws", "src", "rtlf_ros"))
sys.path.insert(0, os.path.join(ROOT, "slam"))

plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
                     "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7, "legend.fontsize": 6, "xtick.labelsize": 6.5,
                     "ytick.labelsize": 6.5, "axes.linewidth": 0.5, "savefig.dpi": 300, "pdf.fonttype": 42})
BLUE, ORANGE, AQUA, VIOLET, RED = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#d03b3b"
INK, MUTED, GRID = "#0b0b0b", "#898781", "#e1e0d9"
CYCLE = [BLUE, ORANGE, AQUA, VIOLET, "#e87ba4"]


def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(True, color=GRID, lw=0.4); ax.set_axisbelow(True)


def wilson(k, n, z=1.96):
    p = k / n; d = 1 + z * z / n; c = p + z * z / (2 * n); h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return max(0.0, 100 * (c - h) / d), min(100.0, 100 * (c + h) / d)


# ---------------------------------------------------------------- sweep
def cmd_sweep(a):
    root = os.path.expanduser(a.root)
    scales = sorted(float(os.path.basename(p)[1:]) for p in glob.glob(os.path.join(root, "k*")))
    fig, ax = plt.subplots(figsize=(3.45, 1.6))
    for key, c, mk, ls, lab in (("b3", AQUA, "o", "--", "B3 retrace + reloc."), ("b4", BLUE, "s", "-", "B4 planner")):
        xs, ys, lo, hi = [], [], [], []
        for ks in scales:
            R = [json.load(open(p)) for p in sorted(glob.glob(os.path.join(root, "k%g" % ks, key + "_s*", "result.json")))]
            k = sum(bool(r.get("success")) for r in R); n = len(R); l, h = wilson(k, n)
            xs.append(0.01 * ks); ys.append(100.0 * k / n); lo.append(100.0 * k / n - l); hi.append(h - 100.0 * k / n)
            print(key, "scale", ks, "%d/%d" % (k, n), "CI %.0f-%.0f%%" % (l, h))
        ax.errorbar(xs, ys, yerr=[lo, hi], color=c, marker=mk, ms=3.5, lw=1.1, ls=ls, capsize=1.8, elinewidth=0.6, label=lab)
    if a.band:
        lo_b, hi_b = a.band; ax.axvspan(lo_b, hi_b, color=GRID, lw=0)
        ax.text(math.sqrt(lo_b * hi_b), 7, "measured\nrange", ha="center", fontsize=5.5, color=MUTED)
    ax.set_xscale("log"); ax.set_xticks([0.01 * s for s in scales]); ax.set_xticklabels(["%g" % (0.01 * s) for s in scales])
    ax.minorticks_off(); ax.set_ylim(-3, 105)
    ax.set_xlabel("textured-ground drift rate $\\kappa$ (m/m) of the emulated estimator"); ax.set_ylabel("success (%)")
    style(ax); ax.legend(loc="lower left", frameon=False, handlelength=2.0, borderpad=0.1)
    fig.savefig(a.out, bbox_inches="tight"); print("wrote", a.out)


# ---------------------------------------------------------------- map
def cmd_map(a):
    from rtlf_ros import scenario as S
    fig, ax = plt.subplots(figsize=(3.45, 1.65))
    for (cn, ce, sn, se, _h) in S.BUILDINGS:
        ax.add_patch(Rectangle((ce - se / 2, cn - sn / 2), se, sn, fc="#b9b8b2", ec="none", zorder=1))
    for (n0, n1, e0, e1) in S.SPARSE:
        ax.add_patch(Rectangle((e0, n0), e1 - e0, n1 - n0, fill=False, ls="--", ec=MUTED, lw=0.7, zorder=1))
        ax.text(e1 - 2, n1 + 2, "textureless slab", ha="right", va="bottom", fontsize=5.5, color=MUTED)
    paths, out, anchor, i = [], None, None, 0
    for spec in a.run or []:
        d, lab = spec.rsplit(":", 1); d = os.path.expanduser(d)
        rows = list(csv.DictReader(open(os.path.join(d, "log.csv"))))
        o = np.array([[float(r["n"]), float(r["e"])] for r in rows if r["phase"] == "OUTBOUND"])
        r = np.array([[float(r["n"]), float(r["e"])] for r in rows if r["phase"] == "RETURN"])
        if out is None:
            out = o
        paths.append((lab, r))
    if a.kin:
        from rtlf_ros import core
        for spec in a.kin:
            m, seed, ks, lab = spec.split(":")
            sim = core.run_kinematic(core.Sim(core.make_args(method=m, seed=int(seed), kappa_scale=float(ks))))
            if out is None:
                out = np.array(sim.tr_out)
            if anchor is None and sim.mon.anchor:
                anchor = sim.anchor_truth
            paths.append((lab, np.array(sim.tr_ret)))
    ax.plot(out[:, 1], out[:, 0], color=INK, lw=0.9, label="outbound", zorder=3)
    styles = [(ORANGE, ":"), (AQUA, "--"), (BLUE, "-"), (VIOLET, "-."), ("#e87ba4", "-")]
    for j, (lab, r) in enumerate(paths):
        c, ls = styles[j % len(styles)]
        ax.plot(r[:, 1], r[:, 0], color=c, ls=ls, lw=1.2, label=lab, zorder=4)
    if anchor is not None:
        ax.plot(anchor[1], anchor[0], marker="*", color=RED, ms=7, mec="white", mew=0.4, ls="", zorder=6, label="trusted anchor")
    ax.plot(out[0, 1], out[0, 0], "s", color=INK, ms=3.5, zorder=6)
    ax.set_aspect("equal"); ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); style(ax)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.30), ncol=3, frameon=False, columnspacing=0.9, handlelength=1.9)
    fig.savefig(a.out, bbox_inches="tight"); print("wrote", a.out)


# ---------------------------------------------------------------- tracking
def load_stretch(path):
    import verdict
    ids, E, G = verdict.load(os.path.expanduser(path))
    pe = np.array([m[:3, 3] for m in E]); pg = np.array([m[:3, 3] for m in G])
    dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pg, axis=0), axis=1))])
    return dist, pg, pe, np.linalg.norm(pe - pg, axis=1)


def kappa(dist, err):
    m = dist > 5
    return float(np.sum(err[m] * dist[m]) / np.sum(dist[m] ** 2)) if m.any() else float("nan")


def cmd_tracking(a):
    S = []
    for spec in a.stretch:
        parts = spec.split(":"); lost = float(parts[2]) if len(parts) > 2 else None
        S.append((parts[0], parts[1], lost) + load_stretch(parts[0]))
    fig, axs = plt.subplots(1, 3, figsize=(7.1, 1.75), gridspec_kw={"width_ratios": [1.15, 1.05, 0.95], "wspace": 0.34})
    ax_a, ax_b, ax_c = axs; vals = []
    for j, (path, lab, lost, dist, pg, pe, err) in enumerate(S):
        c = CYCLE[j % len(CYCLE)]; gi = int(np.searchsorted(dist, lost, side="right")) - 1 if lost else len(dist) - 1
        # (a) top view relative to the stretch start: N and E are columns 0 and 1 of the poses
        ax_a.plot(pg[:, 1] - pg[0, 1], pg[:, 0] - pg[0, 0], color=INK, lw=0.7, zorder=2)
        ax_a.plot(pe[:, 1] - pe[0, 1], pe[:, 0] - pe[0, 0], color=c, lw=1.1, zorder=3, label=lab)
        if lost:
            ax_a.plot(pe[gi, 1] - pe[0, 1], pe[gi, 0] - pe[0, 0], "x", color=RED, ms=5, mew=1.1, zorder=5)
        # (b) error vs distance, with the through-origin fit to the good part
        ax_b.plot(dist, err, color=c, lw=1.1, label=lab)
        k = kappa(dist[:gi + 1], err[:gi + 1]); xx = np.array([0, dist[-1]]); ax_b.plot(xx, k * xx, color=c, lw=0.6, ls="--")
        if lost:
            ax_b.plot(dist[gi], err[gi], "x", color=RED, ms=5, mew=1.1, zorder=5)
        vals.append((lab, k, bool(lost))); print("%-8s kappa %.4f  (%s)  path %.1f m  final error %.1f m" % (lab, k, "good part only" if lost else "whole stretch", dist[-1], err[-1]))
    ax_a.plot([], [], color=INK, lw=0.7, label="true path"); ax_a.plot([], [], "x", color=RED, ms=5, mew=1.1, label="tracking lost")
    ax_a.set_aspect("equal"); ax_a.set_xlabel("East from start (m)"); ax_a.set_ylabel("North from start (m)")
    ax_a.set_title("(a) estimated vs true path", loc="left"); style(ax_a); ax_a.legend(frameon=False, handlelength=1.5, labelspacing=0.2, borderpad=0.1)
    ax_b.set_xlabel("distance flown (m)"); ax_b.set_ylabel("position error (m)"); ax_b.set_title("(b) error vs distance", loc="left")
    style(ax_b); ax_b.legend(frameon=False, handlelength=1.4, borderpad=0.1, loc="upper left")
    ks = np.array([v[1] for v in vals])
    for i, (lab, k, part) in enumerate(vals):
        ax_c.bar(i, k, width=0.62, color=BLUE, hatch="////" if part else None, edgecolor="white" if part else BLUE, lw=0)
        ax_c.text(i, k + 0.002, "%.3f" % k, ha="center", va="bottom", fontsize=5.5)
    ax_c.axhline(ks.mean(), color=ORANGE, lw=0.9, ls="--", label="mean %.3f" % ks.mean())
    ax_c.axhline(0.01, color=INK, lw=0.8, ls=":", label="originally assumed 0.01")
    ax_c.set_xticks(range(len(vals))); ax_c.set_xticklabels([v[0] for v in vals], fontsize=5.5)
    ax_c.set_ylabel("drift rate $\\kappa$ (m/m)"); ax_c.set_ylim(0, max(ks) * 1.35); ax_c.set_title("(c) drift rate per stretch", loc="left")
    style(ax_c); ax_c.grid(False); ax_c.legend(frameon=False, handlelength=1.6, borderpad=0.1, loc="upper left", bbox_to_anchor=(-0.02, 1.02))
    print("mean kappa %.4f  s.d. %.4f  n=%d" % (ks.mean(), ks.std(ddof=1) if len(ks) > 1 else float("nan"), len(ks)))
    fig.savefig(a.out, bbox_inches="tight"); print("wrote", a.out)


# ---------------------------------------------------------------- frames
def cmd_frames(a):
    from PIL import Image
    rows = [(a.north, "(a) north leg, textureless slab"), (a.east, "(b) east leg, no nearby structure")]
    fig, axs = plt.subplots(2, 1, figsize=(7.1, 1.75), gridspec_kw={"hspace": 0.03})
    for ax, (p, t) in zip(axs, rows):
        im = np.array(Image.open(os.path.expanduser(p)).convert("RGB"))
        ax.imshow(im[20:225]); ax.axis("off")                      # top half of the montage = the RGB row
        ax.text(6, 8, t, fontsize=6, color="white", va="top", ha="left", bbox=dict(boxstyle="square,pad=0.2", fc="black", ec="none", alpha=0.55))
    fig.savefig(a.out, bbox_inches="tight", pad_inches=0.02); print("wrote", a.out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sweep"); s.add_argument("--root", default="results_kin"); s.add_argument("--out", default="fig_sweep.pdf")
    s.add_argument("--band", type=float, nargs=2, metavar=("LO", "HI"), default=None, help="shade a measured kappa range, e.g. --band 0.02 0.064")
    m = sub.add_parser("map"); m.add_argument("--run", action="append"); m.add_argument("--kin", action="append"); m.add_argument("--out", default="fig_map.pdf")
    t = sub.add_parser("tracking"); t.add_argument("--stretch", action="append", required=True); t.add_argument("--out", default="fig_tracking.pdf")
    f = sub.add_parser("frames"); f.add_argument("--north", required=True); f.add_argument("--east", required=True); f.add_argument("--out", default="fig_frames.pdf")
    a = ap.parse_args()
    {"sweep": cmd_sweep, "map": cmd_map, "tracking": cmd_tracking, "frames": cmd_frames}[a.cmd](a)


if __name__ == "__main__":
    main()
