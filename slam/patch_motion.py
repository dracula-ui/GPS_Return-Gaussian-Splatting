#!/usr/bin/env python3
"""Idempotent patch of MonoGS utils/slam_frontend.py: Training.motion_prior and Training.force_kf_every."""
import sys
p = sys.argv[1]; s = open(p).read()
if "motion_prior" in s:
    print("already patched"); sys.exit(0)
anchor = "        opt_params = []\n        opt_params.append(\n            {\n                \"params\": [viewpoint.cam_rot_delta],\n                \"lr\": self.config[\"Training\"][\"lr\"][\"cam_rot_delta\"],"
assert s.count(anchor) == 1, "tracking() anchor not found exactly once"
s = s.replace(anchor, '''        if self.config["Training"].get("motion_prior", False) and getattr(self, "_vel", None) is not None:
            _p0 = -viewpoint.R.T @ viewpoint.T
            viewpoint.update_RT(viewpoint.R, -viewpoint.R @ (_p0 + self._vel.to(_p0)))   # constant-velocity guess for this frame

''' + anchor)
end = "        self.median_depth = get_median_depth(depth, opacity)\n        return render_pkg\n"
assert s.count(end) == 1, "tracking() end not found exactly once"
s = s.replace(end, '''        if self.config["Training"].get("motion_prior", False):
            _v = (-viewpoint.R.T @ viewpoint.T) - (-prev.R.T @ prev.T)
            self._vel = _v if getattr(self, "_vel", None) is None else 0.5 * self._vel.to(_v) + 0.5 * _v
''' + end)
old = "        return (point_ratio_2 < kf_overlap and dist_check2) or dist_check\n"
assert s.count(old) == 1, "is_keyframe return not found exactly once"
s = s.replace(old, '''        _force = self.config["Training"].get("force_kf_every", 0)
        if _force and (cur_frame_idx - last_keyframe_idx) >= _force:
            return True
''' + old)
open(p, "w").write(s); print("patched", p)
