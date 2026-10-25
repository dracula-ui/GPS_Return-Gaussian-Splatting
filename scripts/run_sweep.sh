#!/usr/bin/env bash
# Batch experiment in Gazebo: emulated baselines b1-b4 (+ naive-anchor ablation) for N seeds, optionally real SLAM.
# A fresh simulation is started for every run. Usage:
#   HEADLESS=1 scripts/run_sweep.sh 10                 # emulated methods only
#   HEADLESS=1 SLAM=1 scripts/run_sweep.sh 5           # also s2/s4 (MonoGS server is started/stopped per run)
source "$(dirname "$0")/common.sh"
N="${1:-10}"
run_one() {  # args: launch args...
  local use_slam=0; [[ " $* " == *"pose_source:=slam"* ]] && use_slam=1
  rm -f "$LOGDIR/px4.log"
  "$ROOT/scripts/run_sim.sh" >"$LOGDIR/sweep_sim.log" 2>&1 &
  SIM=$!
  for i in $(seq 1 240); do grep -q "Ready for takeoff" "$LOGDIR/px4.log" 2>/dev/null && break; sleep 1; done
  sleep 5
  if [ $use_slam = 1 ]; then
    "$ROOT/scripts/run_slam_server.sh" >"$LOGDIR/slam_server.log" 2>&1 &
    SLAMPID=$!
    for i in $(seq 1 180); do grep -q "RosDataset\] waiting" "$LOGDIR/slam_server.log" 2>/dev/null && break; sleep 1; done
  fi
  "$ROOT/scripts/run_stack.sh" "$@" || true
  if [ $use_slam = 1 ]; then kill -INT "$SLAMPID" 2>/dev/null; for i in $(seq 1 120); do kill -0 "$SLAMPID" 2>/dev/null || break; sleep 1; done; pkill -f "slam.py --config" 2>/dev/null; fi
  kill "$SIM" 2>/dev/null; "$ROOT/scripts/stop_all.sh" >/dev/null 2>&1; sleep 3
}
for seed in $(seq 1 "$N"); do
  for m in b1 b2 b3 b4; do say "=== $m seed $seed ==="; run_one method:=$m seed:=$seed pose_source:=emulated; done
  say "=== b4 naive anchor seed $seed ==="; run_one method:=b4 seed:=$seed naive_anchor:=true pose_source:=emulated
  if [ "${SLAM:-0}" = 1 ]; then
    for m in s2 s4; do say "=== $m seed $seed (real SLAM) ==="; run_one method:=$m seed:=$seed pose_source:=slam; done
  fi
done
python3 "$ROOT/tools/aggregate.py" "$ROOT/results"
