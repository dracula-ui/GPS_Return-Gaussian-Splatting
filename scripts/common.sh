#!/usr/bin/env bash
# Sourced by the other scripts. Override anything by exporting it before running.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$(cat "$ROOT/.px4_dir" 2>/dev/null || echo "$HOME/PX4-Autopilot")}"
MONOGS_DIR="${MONOGS_DIR:-$HOME/MonoGS}"
CONDA_ENV="${CONDA_ENV:-monogs}"
GEN="$ROOT/sim/generated"
WS="$ROOT/ros_ws"
LOGDIR="$ROOT/logs"; mkdir -p "$LOGDIR"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"

say()  { printf '\033[1;34m[rtlf]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[rtlf] WARNING:\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[rtlf] ERROR:\033[0m %s\n' "$*" >&2; exit 1; }

source_ros() {
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO_NAME/setup.bash" || die "ROS 2 $ROS_DISTRO_NAME not installed (run scripts/01_install_ros.sh)"
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
  return 0
}

gz_major() { gz sim --versions 2>/dev/null | head -1 | cut -d. -f1; }   # 7 = Garden, 8 = Harmonic
gz_name()  { case "$(gz_major)" in 7) echo garden;; 8) echo harmonic;; 6) echo fortress;; *) echo unknown;; esac; }

export_gz_env() {
  export GZ_SIM_RESOURCE_PATH="$GEN/models:$GEN/worlds:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
  export GZ_SIM_SERVER_CONFIG_PATH="$GEN/server.config"
}

activate_conda() {
  local base
  base="$(conda info --base 2>/dev/null || true)"
  [ -z "$base" ] && for b in "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" /opt/conda; do [ -d "$b" ] && base="$b" && break; done
  [ -n "$base" ] || die "conda not found (run scripts/03_install_monogs.sh)"
  source "$base/etc/profile.d/conda.sh"; conda activate "$CONDA_ENV" || die "conda env '$CONDA_ENV' missing (run scripts/03_install_monogs.sh)"
}
