#!/usr/bin/env bash
# MonoGS live server. Start it BEFORE run_stack.sh in slam mode. Ctrl-C stops it and saves the map + trajectory
# (results/... under the MonoGS folder, trj_final.json can be analysed with slam/analyze_drift.py).
source "$(dirname "$0")/common.sh"
activate_conda
cd "$MONOGS_DIR"
[ -f configs/sim/sim_live.yaml ] || python "$ROOT/slam/make_configs.py" --monogs "$MONOGS_DIR"
exec python slam.py --config configs/sim/sim_live.yaml
