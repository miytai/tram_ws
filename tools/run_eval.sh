#!/bin/bash
# Run the full-dataset evaluation in the background.
#   tools/run_eval.sh <tag> [extra args...]
set -e
source /opt/ros/humble/setup.bash
source /home/ros/tram_ws/install/setup.bash
TAG=$1
shift
cd /home/ros/tram_ws
exec python3 src/tram_backup_odometry/scripts/evaluate_bag \
    --all /home/ros/Downloads/dataset/data \
    --csv "/home/ros/tram_ws/calib/s_${TAG}.csv" "$@" \
    > "/home/ros/tram_ws/calib/e_${TAG}.log" 2>&1
