#!/usr/bin/env bash
# Starts Gazebo (urban-canyon world with the RGB-D x500) and PX4 SITL attached to it. Leave this terminal open.
# Env: HEADLESS=1 (no GUI window)   HEADLESS_RENDER=1 (EGL rendering, for machines with no display)
#      RTF=0.5 (slow the simulation so a slow SLAM keeps up; regenerates the world)   PX4_DIR=...
source "$(dirname "$0")/common.sh"
"$ROOT/scripts/stop_all.sh" >/dev/null 2>&1 || true
[ -d "$PX4_DIR/Tools/simulation/gz/models/x500" ] || die "PX4 with gz models not found at $PX4_DIR (set PX4_DIR or run 02_install_px4.sh)"
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || die "PX4 not built: run (cd $PX4_DIR && make px4_sitl)"
python3 "$ROOT/sim/make_world.py" --rtf "${RTF:-1.0}" --ground "${GROUND:-mesh}" >/dev/null
export_gz_env
say "Gazebo $(gz_name), world rtlf_urban (RTF ${RTF:-1.0})"
RENDER_FLAG=""; [ "${HEADLESS_RENDER:-0}" = 1 ] && RENDER_FLAG="--headless-rendering"
gz sim -r -s $RENDER_FLAG "$GEN/worlds/rtlf_urban.sdf" >"$LOGDIR/gz_server.log" 2>&1 &
echo $! > "$LOGDIR/gz_server.pid"
if [ -z "${HEADLESS:-}" ] && [ "${HEADLESS_RENDER:-0}" != 1 ]; then
  sleep 3; gz sim -g >"$LOGDIR/gz_gui.log" 2>&1 &
  echo $! > "$LOGDIR/gz_gui.pid"
fi

say "waiting for the camera topics ..."
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q "/rtlf/rgbd/depth_image" && break
  sleep 1
done
if ! gz topic -l 2>/dev/null | grep -q "/rtlf/rgbd/depth_image"; then
  warn "camera topics not found. Gazebo topics containing 'rgbd' or 'image':"
  gz topic -l | grep -iE "rgbd|image|camera" || true
  warn "see $LOGDIR/gz_server.log; if the names differ, pass them to the launch: rgb_gz_topic:=... depth_gz_topic:=..."
fi

say "starting PX4 SITL (standalone, attaching to model $(python3 -c "import sys;sys.path.insert(0,'$WS/src/rtlf_ros');from rtlf_ros import sim_params as P;print(P.MODEL_NAME)"))"
cd "$PX4_DIR"
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=rtlf_urban PX4_GZ_MODEL_NAME="$(python3 -c "import sys;sys.path.insert(0,'$WS/src/rtlf_ros');from rtlf_ros import sim_params as P;print(P.MODEL_NAME)")" \
  make px4_sitl gz_x500 >"$LOGDIR/px4.log" 2>&1 &
echo $! > "$LOGDIR/px4.pid"
for i in $(seq 1 120); do grep -q "Ready for takeoff" "$LOGDIR/px4.log" 2>/dev/null && break; sleep 1; done
if grep -q "Ready for takeoff" "$LOGDIR/px4.log"; then
  "$PX4_DIR/build/px4_sitl_default/bin/px4-param" set EKF2_GPS_CHECK 181 >/dev/null 2>&1 || true
  say "PX4 ready. Simulation is up. Logs in $LOGDIR. Press Ctrl-C to stop everything."
else
  warn "PX4 did not report 'Ready for takeoff' within 2 minutes; tail of $LOGDIR/px4.log:"; tail -20 "$LOGDIR/px4.log"
fi
trap '"$ROOT/scripts/stop_all.sh"; exit 0' INT TERM
wait
