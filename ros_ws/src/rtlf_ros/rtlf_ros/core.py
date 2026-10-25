"""Return-to-last-fix logic: trusted-anchor GNSS monitor, drift-aware planner, mission state machine.

Pure Python + numpy (no ROS, no simulator), so it can be unit tested anywhere.

Pose sources
------------
* DriftEstimator : emulated pose (parametric drift model) used for baselines B1-B4.
* SlamEstimator  : pose from a real SLAM system (MonoGS over ROS/ZMQ) used for methods S2 / S4.

Methods
-------
b1 straight line on emulated IMU dead reckoning     b2 retrace, emulated VIO drift (no relocalization)
b3 retrace with emulated Gaussian-map relocalization b4 drift-aware planner (emulated estimator)
s2 retrace on the REAL SLAM pose                      s4 drift-aware planner on the REAL SLAM pose
"""
import csv
import heapq
import json
import math
import os
import threading
import types

import numpy as np

from . import scenario as S

KF_SPACING = 4.0
RELOC_R = 8.0
CHI2_2D_99 = 9.210
T_D = 3.0
SATS_MIN, HDOP_MAX = 8, 2.0
GPS_EVERY = 4                      # GNSS update every 4 control ticks (5 Hz at 20 Hz control)
KAPPA = {"b1": (0.04, 0.04), "b2": (0.02, 0.06), "b3": (0.01, 0.03), "b4": (0.01, 0.03)}  # drift/m (textured, sparse)
RELOC = {"b1": False, "b2": False, "b3": True, "b4": True}
NOMINAL_KAPPA = (0.01, 0.03)       # planner's drift model (calibrate with slam/analyze_drift.py)
LAMBDAS = [0, 10, 20, 50, 100, 200, 500, 1000]
EMULATED = ("b1", "b2", "b3", "b4")
RETRACE = ("b2", "b3", "s2")
PLANNED = ("b4", "s4")


def make_args(**kw):
    d = dict(method="b4", seed=1, naive_anchor=False, eps=4.0, kappa_scale=1.0, tol=3.0,
             ret_timeout=300.0, cruise=-1.0, slam_frac=0.03, slam_timeout=3.0, slam_lost_abort=20.0)
    d.update(kw)
    return types.SimpleNamespace(**d)


# ----------------------------------------------------------------------------- map / estimators
class Keyframes:
    def __init__(self):
        self.truth, self.est, self.err = [], [], []

    def add(self, truth, est, err):
        self.truth.append(np.array(truth, float)); self.est.append(np.array(est, float)); self.err.append(np.array(err, float))

    def __len__(self):
        return len(self.truth)

    def nearest_truth(self, p):
        T = np.asarray(self.truth)
        d = np.linalg.norm(T - p, axis=1)
        j = int(np.argmin(d))
        return j, float(d[j])


class DriftEstimator:
    """Stand-in for a SLAM pose: error grows by kappa*distance (higher on sparse texture) in a slowly rotating
    direction. Methods with a Gaussian map relocalize against keyframes (error pulled toward the error stored
    with the keyframe, so keyframes made before GNSS loss reset drift to ~0)."""
    kind = "emulated"
    has_pose = True

    def __init__(self, method, rng, kscale):
        self.m, self.rng = method, rng
        self.k = np.array(KAPPA[method]) * kscale * rng.lognormal(0, 0.3)
        self.err = np.zeros(2); self.active = False; self.reloc_on = False
        self.h = rng.uniform(0, 2 * math.pi); self.dist = 0.0; self.max_err = 0.0

    def step(self, truth, prev, dt, kfs):
        d = float(np.linalg.norm(truth - prev))
        if self.active:
            self.dist += d
            self.h += self.rng.normal(0, 0.35 * math.sqrt(dt))
            k = self.k[1] if S.in_sparse(*truth) else self.k[0]
            self.err += k * d * np.array([math.cos(self.h), math.sin(self.h)]) + self.rng.normal(0, 0.02 * d, 2)
            if self.reloc_on and RELOC[self.m] and len(kfs):
                j, dd = kfs.nearest_truth(truth)
                if dd < RELOC_R:
                    self.err += min(1.0, dt) * (kfs.err[j] - self.err)
            self.max_err = max(self.max_err, float(np.linalg.norm(self.err)))

    def pos(self, truth):
        return truth + self.err

    def stale(self):
        return 0.0


class SlamEstimator:
    """Pose from a real SLAM system. `update()` is called from the ROS callback with the SLAM vehicle position
    (north, east) and the simulator-truth position at the SLAM frame's timestamp. Between SLAM updates the
    estimate is propagated by the truth displacement since that stamp. This stands in for IMU pre-integration
    over the SLAM latency (a few tenths of a second), where inertial error is negligible."""
    kind = "slam"

    def __init__(self, clock):
        self.clock = clock                      # callable -> seconds (ROS/sim time)
        self.err = np.zeros(2); self.active = True; self.reloc_on = False
        self.dist = 0.0; self.max_err = 0.0
        self._lock = threading.Lock(); self._s = None; self._recv = None; self.n_updates = 0

    @property
    def has_pose(self):
        return self._s is not None

    def update(self, slam_ne, truth_ne):
        with self._lock:
            self._s = (np.array(slam_ne, float), np.array(truth_ne, float)); self._recv = self.clock()
            self.n_updates += 1

    def step(self, truth, prev, dt, kfs):
        self.dist += float(np.linalg.norm(truth - prev))

    def pos(self, truth):
        with self._lock:
            s = self._s
        p = truth.copy() if s is None else s[0] + (truth - s[1])
        self.err = p - truth
        self.max_err = max(self.max_err, float(np.linalg.norm(self.err)))
        return p

    def stale(self):
        with self._lock:
            r = self._recv
        return 0.0 if r is None else max(0.0, self.clock() - r)


class GpsMonitor:
    """Synthetic GNSS + health monitor. Stores a candidate anchor only when satellites, HDOP and the
    GNSS/estimate consistency test all pass; declares loss after T_D seconds of continuous failure."""

    def __init__(self, rng, naive, frac=0.03):
        self.rng, self.naive, self.frac = rng, naive, frac
        self.deg_t0 = None; self.fail_start = None; self.cand = None; self.last_fix = None
        self.lost = False; self.anchor = None
        a = rng.uniform(0, 2 * math.pi); self.dir = np.array([math.cos(a), math.sin(a)])

    def update(self, t, truth, est, d_den, kf_count):
        if self.deg_t0 is None:
            sats, hdop, bias, avail = 14.0, 0.8, 0.0, True
        else:
            tau = t - self.deg_t0; r = min(tau / 6.0, 1.0)
            sats, hdop, bias, avail = 14 - 10 * r, 0.8 + 7.2 * r, 40.0 * r * r, tau <= 8.0
        ok = False
        if avail:
            meas = truth + bias * self.dir + self.rng.normal(0, 1.0, 2)
            self.last_fix = (meas.copy(), truth.copy(), kf_count)
            var = 1.0 + 0.25 + (self.frac * d_den) ** 2
            stat = float(np.sum((meas - est) ** 2) / var)
            ok = sats >= SATS_MIN and hdop < HDOP_MAX and stat <= CHI2_2D_99
            if ok:
                self.cand = (meas.copy(), truth.copy(), kf_count)
        if ok:
            self.fail_start = None
        else:
            if self.fail_start is None:
                self.fail_start = t
            if not self.lost and t - self.fail_start >= T_D:
                self.lost = True
                self.anchor = self.last_fix if self.naive else self.cand


# ----------------------------------------------------------------------------- drift-aware planner
def _near_kf(p, K, r=RELOC_R):
    return len(K) > 0 and float(np.min(np.linalg.norm(K - p, axis=1))) < r


def plan_return(start, goal, K, kscale, eps, dmin=7.0, res=5.0, reloc_factor=0.1):
    """Resource-constrained shortest path on a 2D grid, solved by Lagrangian weight escalation:
    Dijkstra on cost len*(1 + lam*kappa) for increasing lam until the expected drift fits the budget."""
    n0, n1, e0, e1 = S.BOUNDS
    ns = np.arange(n0, n1 + 1e-6, res); es = np.arange(e0, e1 + 1e-6, res)
    H, W = len(ns), len(es)
    free = np.array([[not S.in_building(n, e, dmin) for e in es] for n in ns])

    def snap(p):
        i = int(np.clip(round((p[0] - n0) / res), 0, H - 1)); j = int(np.clip(round((p[1] - e0) / res), 0, W - 1))
        if free[i, j]:
            return i, j
        best = min(((abs(ii - i) + abs(jj - j), ii, jj) for ii in range(H) for jj in range(W) if free[ii, jj]))
        return best[1], best[2]

    s, g = snap(start), snap(goal)
    K = np.asarray(K).reshape(-1, 2)
    kap = lambda mid: (NOMINAL_KAPPA[1] if S.in_sparse(*mid) else NOMINAL_KAPPA[0]) * kscale * (reloc_factor if _near_kf(mid, K) else 1.0)
    adj = {}
    for i in range(H):
        for j in range(W):
            if not free[i, j]:
                continue
            L = []
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == dj == 0:
                        continue
                    k, l = i + di, j + dj
                    if not (0 <= k < H and 0 <= l < W) or not free[k, l]:
                        continue
                    if di and dj and not (free[i + di, j] and free[i, j + dj]):
                        continue
                    ell = res * math.hypot(di, dj)
                    mid = ((ns[i] + ns[k]) / 2, (es[j] + es[l]) / 2)
                    L.append(((k, l), ell, kap(mid)))
            adj[(i, j)] = L

    def solve(lam):
        dist = {s: 0.0}; prev = {}; pq = [(0.0, s)]
        while pq:
            c, u = heapq.heappop(pq)
            if u == g:
                break
            if c > dist.get(u, 1e18):
                continue
            for v, ell, kp in adj[u]:
                nc = c + ell * (1 + lam * kp)
                if nc < dist.get(v, 1e18):
                    dist[v] = nc; prev[v] = u; heapq.heappush(pq, (nc, v))
        if g not in prev and g != s:
            return None
        path = [g]
        while path[-1] != s:
            path.append(prev[path[-1]])
        path.reverse()
        length = ksum = 0.0
        for a, b in zip(path[:-1], path[1:]):
            for v, ell, kp in adj[a]:
                if v == b:
                    length += ell; ksum += ell * kp
        return path, length, ksum

    cands = []
    for lam in LAMBDAS:
        r = solve(lam)
        if r:
            cands.append((lam,) + r)
            if r[2] <= eps:
                break
    if not cands:
        return dict(ok=False, reason="no path")
    feas = [c for c in cands if c[3] <= eps]
    if not feas:
        best = min(cands, key=lambda c: c[3])
        return dict(ok=False, reason=f"budget infeasible (best expected drift {best[3]:.1f} m > {eps} m)")
    lam, path, length, ksum = feas[0]
    pts = [np.array([ns[i], es[j]]) for i, j in path]
    wps = [pts[0]]
    for a, b, c in zip(pts[:-1], pts[1:], pts[2:]):     # drop collinear points
        if abs((b - a)[0] * (c - b)[1] - (b - a)[1] * (c - b)[0]) > 1e-6:
            wps.append(b)
    wps.append(pts[-1]); wps.append(np.asarray(goal, float))
    return dict(ok=True, wps=wps[1:], lam=lam, length_m=length, expected_drift_m=ksum)


# ----------------------------------------------------------------------------- mission logic
class Sim:
    """Mission state machine. tick(dt, n, e, alt) -> (vn, ve, vd) in NED; the caller moves the vehicle."""

    def __init__(self, a, est=None):
        self.a = a; self.rng = np.random.default_rng(a.seed)
        if est is None:
            if a.method not in EMULATED:
                raise ValueError("methods s2/s4 need a SlamEstimator")
            est = DriftEstimator(a.method, self.rng, a.kappa_scale)
        self.est = est
        self.mon = GpsMonitor(self.rng, a.naive_anchor, a.slam_frac)
        self.kfs = Keyframes()
        self.cruise = a.cruise if a.cruise > 0 else (S.CRUISE_SLAM if est.kind == "slam" else S.CRUISE)
        self.phase = "CLIMB"; self.sub = "UP"; self.t = 0.0; self.tick_n = 0; self.prev = None
        self.wp = 1; self.since_kf = 0.0; self.finished = False
        self.collisions = 0; self.result = None; self.log = []; self.info = {}
        self.tr_out, self.tr_ret, self.es_ret = [], [], []
        self.wps = []; self.wi = 0; self.hold = 0.0; self.ret_len = 0.0; self.t_ret = 0.0
        self.stale_t = 0.0; self.anchor_meas = self.anchor_truth = None; self.denied_leg = 0.0

    def _toward(self, vec, vmax, gain=0.8):
        d = float(np.linalg.norm(vec))
        if d < 1e-6:
            return np.zeros(2)
        return vec / d * min(vmax, gain * d)

    def start_return(self, truth):
        self.phase = "RETURN"; self.est.reloc_on = True; self.t_ret = self.t
        am, at, kfc = self.mon.anchor
        self.anchor_meas, self.anchor_truth = am, at
        self.denied_leg = self.est.dist
        estp = self.est.pos(truth)
        K = np.asarray(self.kfs.est).reshape(-1, 2)

        def retrace():
            ia = max(kfc - 1, 0)
            i = ia + int(np.argmin(np.linalg.norm(K[ia:] - estp, axis=1)))
            return list(K[ia:i + 1][::-1]) + [am]

        m = self.a.method
        if m == "b1":
            self.wps = [am]; self.info["mode"] = "straight"
        elif m in RETRACE:
            self.wps = retrace(); self.info["mode"] = "retrace"
        else:
            p = plan_return(estp, am, K, self.a.kappa_scale, self.a.eps)
            if p["ok"]:
                self.wps = p["wps"]; self.info.update(mode="planned", lam=p["lam"], planned_len_m=p["length_m"],
                                                       expected_drift_m=p["expected_drift_m"])
            else:
                self.wps = retrace(); self.info.update(mode="retrace-fallback", why=p["reason"])
        self.wi = 0; self.hold = 0.0
        self.info["n_waypoints"] = len(self.wps)

    def finish(self, truth, reason):
        a = self.a
        have_anchor = self.phase in ("RETURN", "DONE") and self.mon.anchor is not None
        res = dict(method=a.method, seed=a.seed, naive_anchor=a.naive_anchor, estimator=self.est.kind,
                   reason=reason, collisions=self.collisions, **self.info)
        if have_anchor:
            at, am = self.anchor_truth, self.anchor_meas
            err = float(np.linalg.norm(truth - at))
            res.update(return_err_m=err, anchor_bias_m=float(np.linalg.norm(am - at)),
                       est_err_final_m=float(np.linalg.norm(self.est.err)), est_err_max_m=self.est.max_err,
                       path_ratio=self.ret_len / max(self.denied_leg, 1e-6),
                       drift_pct=100 * float(np.linalg.norm(self.est.err)) / max(self.est.dist, 1e-6),
                       return_time_s=self.t - self.t_ret, return_len_m=self.ret_len, denied_leg_m=self.denied_leg)
            res["success"] = bool(reason == "arrived" and err <= a.tol and self.collisions == 0)
        else:
            res.update(success=False)
        self.result = res
        self.phase = "RECOVER"; self.sub = "UP"

    def tick(self, dt, n, e, alt):
        self.t += dt; self.tick_n += 1
        truth = np.array([n, e])
        if self.prev is None:
            self.prev = truth.copy()
        d = float(np.linalg.norm(truth - self.prev))
        self.est.step(truth, self.prev, dt, self.kfs)
        estp = self.est.pos(truth)
        self.prev = truth.copy()
        vd = float(np.clip(0.8 * (alt - S.ALT), -1.5, 1.5))
        v = np.zeros(2)

        if self.phase in ("OUTBOUND", "RETURN") and S.in_building(n, e, 1.0):
            self.collisions += 1
            self.finish(truth, "collision")

        if self.phase == "CLIMB":
            vd = -1.5
            if alt >= S.ALT - 0.5:
                self.phase = "OUTBOUND"

        elif self.phase == "OUTBOUND":
            self.tr_out.append(truth.copy())
            self.since_kf += d
            if len(self.kfs) == 0 or self.since_kf >= KF_SPACING:
                self.kfs.add(truth, estp, self.est.err); self.since_kf = 0.0
            if self.tick_n % GPS_EVERY == 0 and not self.mon.lost:
                self.mon.update(self.t, truth, estp, self.est.dist, len(self.kfs))
            tgt = np.array(S.ROUTE[self.wp])
            if np.linalg.norm(tgt - truth) < 2.5:
                if self.wp == S.DENIAL_WP and self.mon.deg_t0 is None:
                    self.mon.deg_t0 = self.t; self.est.active = True
                self.wp += 1
                if self.wp >= len(S.ROUTE):
                    if not self.mon.lost:
                        self.mon.update(self.t + T_D + 9, truth, estp, self.est.dist, len(self.kfs))  # force loss decision
                    if self.mon.anchor is None:
                        self.phase = "DONE"; self.result = dict(success=False, reason="GPS loss never declared"); self.finished = True
                    else:
                        self.start_return(truth)
                    return (0.0, 0.0, vd)
                tgt = np.array(S.ROUTE[self.wp])
            v = self._toward(tgt - truth, self.cruise)

        elif self.phase == "RETURN":
            self.tr_ret.append(truth.copy()); self.es_ret.append(estp.copy())
            self.ret_len += d
            tgt = np.asarray(self.wps[self.wi]); last = self.wi == len(self.wps) - 1
            dist = float(np.linalg.norm(tgt - estp))
            if last:
                self.hold = self.hold + dt if dist < 1.2 else 0.0
                if self.hold >= 3.0:
                    self.finish(truth, "arrived"); return (0.0, 0.0, vd)
            elif dist < 2.5:
                self.wi += 1; tgt = np.asarray(self.wps[self.wi])
            if self.t - self.t_ret > self.a.ret_timeout:
                self.finish(truth, "timeout"); return (0.0, 0.0, vd)
            v = self._toward(tgt - estp, self.cruise)
            if self.est.kind == "slam":              # tracking-confidence fallback: hover while the SLAM pose is stale
                if self.est.stale() > self.a.slam_timeout:
                    self.stale_t += dt; v = np.zeros(2)
                    if self.stale_t > self.a.slam_lost_abort:
                        self.finish(truth, "slam_lost"); return (0.0, 0.0, vd)
                else:
                    self.stale_t = 0.0

        elif self.phase == "RECOVER":      # climb above the buildings, go home on PX4's estimate, land
            if self.sub == "UP":
                vd = -2.0
                if alt >= 38:
                    self.sub = "HOME"
            elif self.sub == "HOME":
                vd = float(np.clip(0.8 * (alt - 40), -1.5, 1.5))
                v = self._toward(-truth, self.cruise)
                if np.linalg.norm(truth) < 3.0:
                    self.sub = "LAND"; self.finished = True
        if self.tick_n % 2 == 0:
            self.log.append([round(self.t, 2), self.phase, round(n, 2), round(e, 2), round(alt, 2),
                             round(estp[0], 2), round(estp[1], 2), round(float(np.linalg.norm(self.est.err)), 2)])
        return (float(v[0]), float(v[1]), vd)


# ----------------------------------------------------------------------------- outputs
def save_outputs(sim, outdir):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "result.json"), "w") as f:
        json.dump(sim.result, f, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(os.path.join(outdir, "log.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "phase", "n", "e", "alt", "est_n", "est_e", "est_err"]); w.writerows(sim.log)
    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig, ax = plt.subplots(figsize=(8, 6))
        for (cn, ce, sn, se, _h) in S.BUILDINGS:
            ax.add_patch(Rectangle((ce - se / 2, cn - sn / 2), se, sn, color="gray", alpha=0.7))
        for (n0, n1, e0, e1) in S.SPARSE:
            ax.add_patch(Rectangle((e0, n0), e1 - e0, n1 - n0, fill=False, ls="--", ec="k", label="low-texture zone"))
        if sim.tr_out:
            o = np.array(sim.tr_out); ax.plot(o[:, 1], o[:, 0], "b-", label="outbound (truth)")
        if sim.tr_ret:
            r = np.array(sim.tr_ret); q = np.array(sim.es_ret)
            ax.plot(r[:, 1], r[:, 0], "r-", label="return (truth)"); ax.plot(q[:, 1], q[:, 0], "r:", label="return (estimate)")
        if sim.mon.anchor:
            ax.plot(sim.anchor_truth[1], sim.anchor_truth[0], "g*", ms=14, label="anchor (true fix location)")
            ax.plot(sim.anchor_meas[1], sim.anchor_meas[0], "mx", ms=10, label="anchor (stored)")
        ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]"); ax.set_aspect("equal"); ax.legend(fontsize=7)
        a = sim.a
        ax.set_title(f"{a.method} seed={a.seed}" + (" naive-anchor" if a.naive_anchor else "")
                     + f"  err={sim.result.get('return_err_m', float('nan')):.1f} m")
        fig.savefig(os.path.join(outdir, "plot.png"), dpi=130)
    except Exception as ex:
        print("plot skipped:", ex)


def run_kinematic(sim, dt=0.05, max_ticks=200000):
    """Kinematic stand-in for PX4+Gazebo (first-order velocity lag). Used by tests; no simulator needed."""
    pos = np.zeros(3); vel = np.zeros(3)          # n, e, down
    for _ in range(max_ticks):
        cmd = np.array(sim.tick(dt, pos[0], pos[1], -pos[2]))
        vel += (cmd - vel) * min(1.0, dt / 0.4)
        pos += vel * dt
        if sim.finished:
            break
    return sim
