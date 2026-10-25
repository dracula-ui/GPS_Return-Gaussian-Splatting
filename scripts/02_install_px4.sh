#!/usr/bin/env bash
# PX4 v1.15.x + Gazebo toolchain. Skips everything that already exists (you may already have PX4).
source "$(dirname "$0")/common.sh"
TAG="${PX4_TAG:-v1.15.4}"
if [ ! -d "$PX4_DIR" ]; then
  sudo apt-get update && sudo apt-get install -y git
  git clone --branch "$TAG" --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx
fi
[ -d "$PX4_DIR/Tools/simulation/gz/models/x500" ] || ( cd "$PX4_DIR" && git submodule update --init --recursive )
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || ( cd "$PX4_DIR" && make px4_sitl )
echo "$PX4_DIR" > "$ROOT/.px4_dir"
say "PX4 ready at $PX4_DIR"
