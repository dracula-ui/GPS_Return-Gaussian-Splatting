#!/usr/bin/env bash
#   scripts/test_tracking.sh run5 160 150              # dataset, first frame, number of frames
#   LOCK_ROT=0 scripts/test_tracking.sh run5 160 150   # same, MonoGS estimates rotation itself (for comparison)
source "$(dirname "$0")/common.sh"
NAME="${1:?dataset name, e.g. run5}"; START="${2:-0}"; N="${3:-150}"
export LOCK_ROT="${LOCK_ROT:-1}" MOTION="${MOTION:-1}" FORCE_KF="${FORCE_KF:-10}"
python3 "$ROOT/slam/make_subset.py" "$NAME" "$N" "$START" || die "could not build the subset"
SUB="${NAME}_${START}_${N}"
python3 "$ROOT/slam/patch_lock_rot.py" "$MONOGS_DIR/utils/slam_frontend.py" || die "patch failed"
python3 "$ROOT/slam/patch_motion.py" "$MONOGS_DIR/utils/slam_frontend.py" || die "motion patch failed"
say "MonoGS on $SUB (rotation locked: $LOCK_ROT)"
"$ROOT/scripts/run_monogs_offline.sh" "$SUB"
LATEST="$(ls -dt "$MONOGS_DIR"/results/datasets_sim/*/ | head -1)"
say "verdict for $LATEST"
python3 "$ROOT/slam/verdict.py" "$LATEST" --table
