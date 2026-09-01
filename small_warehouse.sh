cd "$(dirname "$0")"
colcon build
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py map_name:=small_warehouse mode:=slam