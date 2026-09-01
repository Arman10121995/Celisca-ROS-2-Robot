cmake_minimum_required(VERSION 3.8)
project(robot_lab_adapter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

# Install Python package
install(
  DIRECTORY robot_lab_adapter
  DESTINATION lib/python3.10/site-packages
)

# Install resource file
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  rclpy
  std_msgs
  robot_lab_registry
)

ament_package()
