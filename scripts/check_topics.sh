#!/usr/bin/env bash
# Shows the camera topics on the Gazebo side and (if the stack is running) the ROS side.
source "$(dirname "$0")/common.sh"
say "Gazebo topics matching rgbd/camera/image:"; gz topic -l | grep -iE "rgbd|camera|image|clock" || echo "(none: is run_sim.sh running?)"
source_ros
say "ROS topics:"; ros2 topic list | grep -E "rtlf|clock" || echo "(none)"
say "image rate (3 s):"; timeout 3 ros2 topic hz /rtlf/rgbd/image 2>/dev/null | tail -2 || true
