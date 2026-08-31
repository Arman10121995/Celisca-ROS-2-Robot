cmake_minimum_required(VERSION 3.8)
project(robot_lab_registry)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install config directory
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install resource directory
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files if any
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_dependencies()
ament_package()
