cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib/${PROJECT_NAME}:/usr/local/lib:/usr/lib/aarch64-linux-gnu")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

#/opt/ros/humble/lib/python3.10/site-packages
#/workspace/molar/ros_ws/bumperbot_ws/.venv/lib/python3.10/site-packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Prefer installs from source in /usr/local (Pangolin, Sophus, etc.)
list(PREPEND CMAKE_PREFIX_PATH /usr/local)
# Provide explicit hints in case the install layout is non-standard
set(Pangolin_DIR "/usr/local/lib/cmake/Pangolin" CACHE PATH "Path to PangolinConfig.cmake")
set(Sophus_DIR "/usr/local/share/sophus/cmake" CACHE PATH "Path to SophusConfig.cmake")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV 4.8.0 REQUIRED)


# Use both the include paths reported by FindORB_SLAM3 and direct access to vendored headers
include_directories(
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge)
target_include_directories(mono PRIVATE ${ORB_SLAM3_INCLUDE_DIRS})
target_link_libraries(mono ${ORB_SLAM3_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters)
target_include_directories(rgbd PRIVATE ${ORB_SLAM3_INCLUDE_DIRS})
target_link_libraries(rgbd ${ORB_SLAM3_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters)
target_include_directories(stereo PRIVATE ${ORB_SLAM3_INCLUDE_DIRS})
target_link_libraries(stereo ${ORB_SLAM3_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge)
target_include_directories(stereo-inertial PRIVATE ${ORB_SLAM3_INCLUDE_DIRS})
target_link_libraries(stereo-inertial ${ORB_SLAM3_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS mono rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()