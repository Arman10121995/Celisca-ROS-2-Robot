cmake_minimum_required(VERSION 3.8)
project(robot_lab_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The goal relay that restamps RViz goals with the run's clock.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY behavior_tree config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/nav2_goal_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
