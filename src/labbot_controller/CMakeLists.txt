cmake_minimum_required(VERSION 3.8)
project(labbot_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  ${PROJECT_NAME}/manual_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
