#!/usr/bin/env bash
# ROS 2 Humble (Ubuntu 22.04) + ros_gz bridge for the Gazebo you have installed + Python deps.
source "$(dirname "$0")/common.sh"
. /etc/os-release; [ "$VERSION_ID" = "22.04" ] || die "ROS 2 Humble needs Ubuntu 22.04"
command -v gz >/dev/null || die "Install Gazebo first (PX4's Tools/setup/ubuntu.sh does it; see 02_install_px4.sh)"
GZ="$(gz_name)"; say "Detected Gazebo $GZ"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  say "Installing ROS 2 Humble"
  sudo apt-get update && sudo apt-get install -y software-properties-common curl gnupg lsb-release
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y ros-humble-ros-base
fi
sudo apt-get install -y ros-humble-message-filters ros-humble-launch-ros ros-humble-geometry-msgs ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-rosgraph-msgs python3-colcon-common-extensions python3-rosdep python3-opencv python3-matplotlib python3-pil \
  python3-pip git build-essential
pip3 install --user "mavsdk<4" pyzmq

# ros_gz bridge: apt package for Harmonic; build from source for Garden (and as a fallback for Harmonic)
source_ros
if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  say "ros_gz_bridge already available"
else
  if [ "$GZ" = harmonic ] && sudo apt-get install -y ros-humble-ros-gzharmonic; then
    say "installed ros-humble-ros-gzharmonic from apt"
  else
    say "Building ros_gz (humble branch) from source for Gazebo $GZ -> $WS/src/ros_gz  (takes a while)"
    export GZ_VERSION="$GZ"
    mkdir -p "$WS/src"
    [ -d "$WS/src/ros_gz" ] || git clone -b humble --depth 1 https://github.com/gazebosim/ros_gz.git "$WS/src/ros_gz"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    ( cd "$WS" && rosdep install -r --from-paths src -i -y --rosdistro humble --skip-keys "rtlf_ros" ) || warn "rosdep reported problems; continuing to build"
    ( cd "$WS" && colcon build --symlink-install --packages-up-to ros_gz_bridge --cmake-args -DBUILD_TESTING=OFF ) \
      || die "ros_gz build failed. Consider installing Gazebo Harmonic instead (PX4 v1.15 supports it) and re-running."
  fi
fi
say "ROS 2 setup done. Next: scripts/04_build_ws.sh"
