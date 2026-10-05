cmake_minimum_required(VERSION 3.8)
project(robot_lab_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The pinned occupancy generator is a Fortress system plugin, independent of
# the active robot backend. Keep data-only builds usable without that SDK.
find_package(ignition-gazebo6 QUIET)
if(ignition-gazebo6_FOUND)
  find_package(ignition-plugin1 REQUIRED COMPONENTS register)
  find_package(ignition-transport11 REQUIRED)
  find_package(ignition-msgs8 REQUIRED)
  add_library(robot_lab_2dmap_system SHARED vendor/gazebo_2dmap/gz_2Dmap_system.cc)
  target_compile_features(robot_lab_2dmap_system PRIVATE cxx_std_17)
  target_include_directories(robot_lab_2dmap_system PRIVATE vendor/gazebo_2dmap)
  target_link_libraries(robot_lab_2dmap_system
    ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1
    ignition-transport11::ignition-transport11 ignition-msgs8::ignition-msgs8)
  install(TARGETS robot_lab_2dmap_system LIBRARY DESTINATION lib)
endif()
install(DIRECTORY vendor/gazebo_2dmap/ DESTINATION share/${PROJECT_NAME}/vendor/gazebo_2dmap)
install(PROGRAMS tools/generate_occupancy_map.py DESTINATION lib/${PROJECT_NAME})

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
  ament_add_pytest_test(arena_clearance test/test_arena_clearance.py TIMEOUT 60)
endif()

ament_package()
