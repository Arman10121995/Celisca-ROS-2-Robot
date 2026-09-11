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

# Install the world generators/validators so they can be re-run from an install.
install(
  DIRECTORY tools/
  DESTINATION share/${PROJECT_NAME}/tools
)

# Install MJCF (MuJoCo) world files under share/maps/mjcf/...
install(
  DIRECTORY mjcf/
  DESTINATION share/${PROJECT_NAME}/mjcf
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(mjcf_worlds test/test_mjcf_worlds.py TIMEOUT 180)
endif()

ament_package()
