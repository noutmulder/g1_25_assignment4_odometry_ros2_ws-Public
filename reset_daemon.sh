#!/bin/bash
# reset_daemon.sh
# Cleanly reset ROS 2 by killing all related processes and restarting the daemon.

set -e

echo "=== Restarting ROS 2 environment ==="

# Stop ROS 2 daemon (this clears cached node info)
echo "Stopping ROS 2 daemon..."
ros2 daemon stop || true
sleep 1

# Kill all ROS-related processes
echo "Killing all ROS 2 processes..."
pkill -9 -f "ros2 launch" || true
pkill -9 -f "ros2 run" || true
pkill -9 -f "joint_state_publisher" || true
pkill -9 -f "joint_state_publisher_gui" || true
pkill -9 -f "robot_state_publisher" || true
pkill -9 -f "rviz2" || true
pkill -9 -f "static_tf_node" || true
pkill -9 -f "scene_marker_node" || true
pkill -9 -f "coin_marker_node" || true
pkill -9 -f "gz" || true   # Gazebo (if running)
pkill -9 -f "foxglove" || true  # Foxglove (if open)
sleep 1

# Restart the daemon
echo "Restarting ROS 2 daemon..."
ros2 daemon start
sleep 1

echo "=== ROS 2 reset complete ==="
ros2 node list || echo "No nodes active."
