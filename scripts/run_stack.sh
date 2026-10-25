#!/usr/bin/env bash
# ROS side. Examples:
#   scripts/run_stack.sh method:=b4 pose_source:=emulated seed:=1
#   scripts/run_stack.sh method:=s4 pose_source:=slam              (needs run_slam_server.sh or run_fake_slam.sh)
#   scripts/run_stack.sh method:=b4 record_dir:=$HOME/rtlf_data/run1
source "$(dirname "$0")/common.sh"
source_ros
exec ros2 launch rtlf_ros stack.launch.py out:="$ROOT/results" "$@"
