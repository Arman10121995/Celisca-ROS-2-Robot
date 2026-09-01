cmake_minimum_required(VERSION 3.5)
project(robot_lab_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(noisy_controller src/noisy_controller.cpp)
ament_target_dependencies(noisy_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2)

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_controller
  noisy_controller
  twist_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/cleaning_controller.py
  ${PROJECT_NAME}/mapping_controller.py
  ${PROJECT_NAME}/manual_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(map_saver test/test_map_saver.py TIMEOUT 60)
endif()

ament_package()
