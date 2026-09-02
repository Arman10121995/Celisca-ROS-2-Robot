cmake_minimum_required(VERSION 3.8)
project(robot_lab_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install all map data under share/maps/maps/<map_name>/...
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install P4.6 navigation/seed metadata under share/maps/config/...
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install MJCF (MuJoCo) world files under share/maps/mjcf/...
install(
  DIRECTORY mjcf/
  DESTINATION share/${PROJECT_NAME}/mjcf
)

ament_package()
