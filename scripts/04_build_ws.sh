#!/usr/bin/env bash
# Generates the Gazebo world + textures and builds the ROS 2 workspace.
source "$(dirname "$0")/common.sh"
python3 "$ROOT/sim/make_world.py" "$@"
source_ros
( cd "$WS" && colcon build --symlink-install --packages-select rtlf_ros )
say "built. Next: scripts/run_sim.sh   (then scripts/run_stack.sh in another terminal)"
