#!/usr/bin/env bash
# Fly the full mission (emulated pose, method b4) while recording a TUM-format RGB-D dataset with ground-truth
# camera poses. Usage: scripts/record_dataset.sh run1 [extra launch args, e.g. cruise:=2.0 seed:=2]
# If the PX4 link drops before take-off (intermittent at low real-time factors) the attempt is repeated, up to 4 times.
source "$(dirname "$0")/common.sh"
NAME="${1:-run1}"; DATA="$HOME/rtlf_data/$NAME"; mkdir -p "$HOME/rtlf_data"
export PYTHONUNBUFFERED=1
for attempt in 1 2 3 4; do
  rm -rf "$DATA"
  LOG="$(mktemp)"
  "$ROOT/scripts/run_stack.sh" method:=b4 pose_source:=emulated record_dir:="$DATA" out:="$ROOT/results" "${@:2}" 2>&1 | tee "$LOG"
  if grep -q "PX4 link failed before take-off" "$LOG" && [ "$attempt" -lt 4 ]; then
    warn "attempt $attempt: the PX4 link dropped before take-off; cleaning up and retrying"
    pkill -9 -f mavsdk_server 2>/dev/null; sleep 5
    rm -f "$LOG"; continue
  fi
  rm -f "$LOG"; break
done
say "dataset in $DATA"
