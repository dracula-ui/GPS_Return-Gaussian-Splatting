#!/usr/bin/env bash
# GPU-free stand-in for the SLAM server (truth + drift). Tests the ROS pipeline only. Not a SLAM system.
source "$(dirname "$0")/common.sh"
exec python3 "$ROOT/slam/fake_slam_server.py" "$@"
