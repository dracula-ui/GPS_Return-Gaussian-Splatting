#!/usr/bin/env bash
for p in px4 gz_server gz_gui; do
  f="$(dirname "$0")/../logs/$p.pid"; [ -f "$f" ] && kill "$(cat "$f")" 2>/dev/null; rm -f "$f"
done
PATS=("bin/px4" "gz sim" "ruby.*gz" "mavsdk_server" "rtlf_ros/mission_node" "slam_bridge_node" "parameter_bridge")
for pat in "${PATS[@]}"; do pkill -f "$pat" 2>/dev/null; done
sleep 2
for pat in "${PATS[@]}"; do pkill -9 -f "$pat" 2>/dev/null; done
sleep 1
left="$(pgrep -fa 'bin/px4|gz sim|mavsdk_server' | grep -v pgrep)"
if [ -n "$left" ]; then echo "WARNING: still running:"; echo "$left"; else echo "stopped (nothing left running)"; fi
