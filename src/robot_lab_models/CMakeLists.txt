cmake_minimum_required(VERSION 3.5)
project(robot_lab_models)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
