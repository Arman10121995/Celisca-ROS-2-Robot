#!/bin/bash
source /opt/ros/humble/setup.bash
# Kill Gazebo Sim and related Ruby processes
pkill -9 ruby
pkill -9 -f "gz sim"
pkill -9 -f "ign gazebo"
pkill -9 -f "gz"

# Kill ROS 2 nodes and daemon
pkill -9 -f "ros2"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "controller_manager"
pkill -9 -f "amcl"
pkill -9 -f "map_server"
pkill -9 -f "nav2"
pkill -9 -f "rviz2"
pkill -9 -f "bt_navigator"
pkill -9 -f "planner_server"
pkill -9 -f "controller_server"
pkill -9 -f "recoveries_server"
pkill -9 -f "lifecycle_manager"

# Shutdown ROS 2 daemon
ros2 daemon stop

echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo "---------------"
echo "Killed"
echo "---------------"