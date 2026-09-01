cmake_minimum_required(VERSION 3.8)
project(robot_lab_robots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install every robot/config folder so adding a new robot does not require
# editing CMake. Keep upstream source checkouts out of the installed package.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION share/${PROJECT_NAME}
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "_upstream" EXCLUDE
)

ament_package()
