cd ~/bumperbot_ws
colcon build
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py map_name:=small_warehouse mode:=slam