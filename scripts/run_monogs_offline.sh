#!/usr/bin/env bash
# Run MonoGS offline on a recorded dataset and report its real drift (kappa).
# Usage: scripts/run_monogs_offline.sh run1
source "$(dirname "$0")/common.sh"
NAME="${1:-run1}"; DATA="$HOME/rtlf_data/$NAME"
[ -f "$DATA/rgb.txt" ] || die "no dataset at $DATA (run scripts/record_dataset.sh $NAME)"
activate_conda
cd "$MONOGS_DIR"
mkdir -p datasets/sim; ln -sfn "$DATA" "datasets/sim/$NAME"
python "$ROOT/slam/make_configs.py" --monogs "$MONOGS_DIR" --dataset "datasets/sim/$NAME"
python slam.py --config configs/sim/sim_offline.yaml
LATEST="$(ls -dt results/datasets_sim/*/ | head -1)"
say "analysing $LATEST"
python "$ROOT/slam/analyze_drift.py" "$LATEST/plot/trj_final.json" --out "$LATEST/drift.json"
