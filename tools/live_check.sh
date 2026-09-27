#!/bin/bash
# Live end-to-end check: start the node, replay a bag, report rate/latency.
set -x
source /opt/ros/humble/setup.bash
source /home/ros/tram_ws/install/setup.bash
PKG=/home/ros/tram_ws/install/tram_backup_odometry/share/tram_backup_odometry
setsid nohup ros2 run tram_backup_odometry tram_backup_odometry_node \
  --ros-args --params-file $PKG/config/tram_params.yaml "$@" \
  > /tmp/live_node.log 2>&1 < /dev/null &
sleep 6
setsid nohup ros2 bag play /home/ros/Downloads/dataset/data/30618_01f73500 --rate 2.0 \
  > /tmp/live_play.log 2>&1 < /dev/null &
sleep 4
python3 /home/ros/tram_ws/tools/watch_node.py "${WATCH:-200}"
pkill -f tram_backup_odometry_node
pkill -f "ros2 bag play"
sleep 1
echo "=== node log ==="
cat /tmp/live_node.log
