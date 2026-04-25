#!/bin/bash
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$PROJECT_DIR"

pkill -f robot_state_publisher
pkill -f rviz2
sleep 1 

trap "pkill -f robot_state_publisher; pkill -f rviz2; kill %1 2>/dev/null; exit" SIGINT SIGTERM EXIT

rm -rf build install log
colcon build --packages-select arm_teleop
source install/setup.bash

ros2 launch arm_teleop arm_teleop.launch.py &
LAUNCH_PID=$!
sleep 2 

ros2 run arm_teleop tracker_ros_node

wait $LAUNCH_PID