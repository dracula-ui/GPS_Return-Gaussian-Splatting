#!/usr/bin/env bash
# Reports what is installed and what is missing. Changes nothing.
source "$(dirname "$0")/common.sh"
ok()  { printf '  \033[32mOK\033[0m   %s\n' "$*"; }
bad() { printf '  \033[31mMISS\033[0m %s\n' "$*"; }
say "System"
. /etc/os-release; [ "$VERSION_ID" = "22.04" ] && ok "Ubuntu $VERSION_ID" || warn "Ubuntu $VERSION_ID (tested target: 22.04)"
command -v nvidia-smi >/dev/null && ok "GPU: $(nvidia-smi --query-gpu=name,compute_cap,memory.total --format=csv,noheader | head -1)" || bad "nvidia-smi (needed for MonoGS)"
command -v nvcc >/dev/null && ok "nvcc $(nvcc --version | tail -1)" || warn "system nvcc not found (the MonoGS installer adds CUDA 11.8 inside the conda env)"
say "Gazebo / PX4"
command -v gz >/dev/null && ok "gz sim $(gz sim --versions 2>/dev/null | head -1) ($(gz_name))" || bad "gz (Gazebo)"
[ -d "$PX4_DIR/Tools/simulation/gz/models/x500" ] && ok "PX4 at $PX4_DIR (gz models present)" || bad "PX4 with gz models at $PX4_DIR (set PX4_DIR or run 02_install_px4.sh)"
say "ROS 2"
[ -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ] && ok "ROS 2 $ROS_DISTRO_NAME" || bad "ROS 2 $ROS_DISTRO_NAME"
{ source_ros 2>/dev/null; command -v ros2 >/dev/null && ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 && ok "ros_gz_bridge"; } || bad "ros_gz_bridge"
[ -f "$WS/install/setup.bash" ] && ok "workspace built" || bad "workspace not built (04_build_ws.sh)"
python3 -c "import mavsdk" 2>/dev/null && ok "python mavsdk" || bad "python mavsdk (pip3 install --user 'mavsdk<4')"
python3 -c "import zmq" 2>/dev/null && ok "python pyzmq" || bad "python pyzmq (pip3 install --user pyzmq)"
python3 -c "import cv2" 2>/dev/null && ok "python opencv" || bad "python opencv (apt install python3-opencv)"
say "MonoGS"
[ -d "$MONOGS_DIR/.git" ] && ok "MonoGS at $MONOGS_DIR" || bad "MonoGS at $MONOGS_DIR"
[ -f "$MONOGS_DIR/utils/ros_dataset.py" ] && ok "RosDataset patch installed" || bad "RosDataset patch"
command -v conda >/dev/null && conda env list 2>/dev/null | grep -q "^$CONDA_ENV " && ok "conda env $CONDA_ENV" || bad "conda env $CONDA_ENV"
