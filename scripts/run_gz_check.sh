#!/usr/bin/env bash
# Gazebo/PX4 check of b3 and b4 at the measured drift. Usage: HEADLESS=1 scripts/run_gz_check.sh 3
source "$(dirname "$0")/common.sh"
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe PYTHONUNBUFFERED=1
N="${1:-3}"; mkdir -p "$ROOT/results_gz"
for seed in $(seq 1 "$N"); do
  for m in b3 b4; do
    say "=== $m seed $seed (kappa_scale 4.0) ==="
    rm -f "$LOGDIR/px4.log"
    "$ROOT/scripts/run_sim.sh" >"$LOGDIR/gz_check_sim.log" 2>&1 &
    SIM=$!
    for i in $(seq 1 900); do grep -q "Ready for takeoff" "$LOGDIR/px4.log" 2>/dev/null && break; sleep 1; done
    "$PX4_DIR/build/px4_sitl_default/bin/px4-param" set EKF2_GPS_CHECK 149 >/dev/null 2>&1
    "$PX4_DIR/build/px4_sitl_default/bin/px4-param" set COM_ARM_EKF_YAW 1.0 >/dev/null 2>&1
    sleep 5
    TMPLOG=$(mktemp)
    for attempt in 1 2 3 4; do
      "$ROOT/scripts/run_stack.sh" method:=$m seed:=$seed pose_source:=emulated kappa_scale:=4.0 2>&1 | tee "$TMPLOG" | grep -E "RESULT|PX4 link failed|saved ->"
      if grep -q "PX4 link failed before take-off" "$TMPLOG"; then
        say "arm-time link failure; retry $attempt/4"; pkill -9 -f mavsdk_server; sleep 5
      else break; fi
    done
    rm -f "$TMPLOG"
    kill "$SIM" 2>/dev/null; "$ROOT/scripts/stop_all.sh" >/dev/null 2>&1; sleep 3
    rm -rf "$ROOT/results_gz/${m}_s${seed}"; mv "$ROOT/results/${m}_s${seed}" "$ROOT/results_gz/" 2>/dev/null
  done
done
python3 "$ROOT/tools/aggregate.py" "$ROOT/results_gz"
