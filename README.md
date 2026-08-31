# BumperBot Project

## Overview

The BumperBot project is a comprehensive robotics framework designed for mobile robot navigation, mapping, and control. Built using ROS (Robot Operating System), this project provides a modular architecture that enables researchers and developers to experiment with various robotic algorithms and functionalities.

## Quick Start

```bash
cd ~/bumperbot_ws
colcon build
source install/setup.bash

# If you have a Python venv active (.venv), colcon may pick its python3 which lacks
# ROS modules like catkin_pkg. Use ./build.bash (recommended) or ensure system python:
#   colcon build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3

# Launch simulation with a specific map
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=nav map_name:=small_house

# Or with the room vacuum controller
ros2 launch bumperbot_bringup simulated_robot.launch.py \
  mode:=nav map_name:=small_house start_room_vacuum:=true
```

**Note:** After the restructure, make sure you build the supporting packages (`gazebo_models`, the `map_*` you use, `robot_bumperbot`, etc.). See the "Important: Building After the Package Restructure" section below.

See the **Launching Simulations** section below for choosing different maps and robots.

## Important: Building After the Package Restructure

The workspace was reorganized so that maps and robots live in their own packages (`map_*`, `robot_*`, `gazebo_models`, `robot_description_common`).

If you see `PackageNotFoundError: "package 'gazebo_models' not found"` (or similar for a `map_*` package), it means the required packages are not built (or not present) in the workspace you are sourcing.

**Steps:**

1. Make sure the packages exist in `src/` of **the workspace you are building**:
   - `src/gazebo_models`
   - `src/robot_bumperbot`
   - `src/robot_description_common` (enhanced common descriptions)
   - `src/map_<whatever>` for every map you want to use (e.g. `map_simple_box`)

2. Build + source **in the workspace you are using**:

   ```bash
   cd /workspace/molar/ros_ws/bumperbot_ws     # or wherever you are working
   # If a venv is active this can pick wrong python; prefer using the workspace build.bash
   # or force system python:
   colcon build --packages-select \
     gazebo_models robot_bumperbot robot_description_common \
     map_simple_box map_small_house map_celisca_floor_1 \
     bumperbot_description bumperbot_bringup \
     --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
   source install/setup.bash
   ```

3. **Critical when you have two workspaces** (`/workspace/...` vs `/home/molar1/...`):
   - Source **only** the install of the workspace whose `src/` you just built.
   - Best: use a fresh terminal for each workspace.
   - Verify the right one is active:

     ```bash
     ros2 pkg prefix map_simple_box
     # Should print something like /workspace/molar/ros_ws/bumperbot_ws/install/map_simple_box
     # NOT a path under /home/molar1
     ```

4. See what maps are actually available right now:

   ```bash
   ros2 launch bumperbot_bringup simulated_robot.launch.py --show-arguments
   # Look at the map_name choices / description
   ```

## Architecture

The BumperBot system follows a modular architecture organized into multiple packages, each serving specific functions in the robotics pipeline:

```
bumperbot_ws/
├── src/
│   ├── bumperbot_bringup            # Main launch files (simulated + real robot bringup)
│   ├── bumperbot_controller         # Controllers, teleop, room vacuum controller
│   ├── bumperbot_description        # Generic launch files + remaining assets (URDFs moved to robot packages)
│   ├── bumperbot_firmware           # Hardware/firmware interfaces
│   ├── bumperbot_localization       # Localization (AMCL, EKF, etc.)
│   ├── bumperbot_mapping            # SLAM
│   ├── bumperbot_motion             # Motion planners
│   ├── bumperbot_msgs
│   ├── bumperbot_navigation
│   ├── bumperbot_planning
│   ├── bumperbot_py_examples
│   ├── bumperbot_utils
│   │
│   ├── robot_bumperbot              # Bumperbot robot description (URDF + meshes)
│   ├── robot_description_common     # Common reusable description macros/sensors (e.g. OAK-D)
│   ├── gazebo_models                # Shared Gazebo environment models (model:// URIs)
│   │
│   ├── map_small_house
│   ├── map_small_warehouse
│   ├── map_celisca_floor_1
│   ├── map_celisca_floor_2
│   ├── map_celisca_f1_actor
│   ├── map_celisca_f2_actor
│   ├── map_simple_box
│   └── map_empty                    # One package per environment (maps + worlds)
│
├── build.bash
└── kill.bash
```


## Package Descriptions

### bumperbot_bringup
Contains launch files and system initialization configurations for starting up the entire robot system.

### bumperbot_controller
The core control package with various controller implementations:
- `automatic_teleop_1.py`: Basic automatic teleoperation
- `automatic_teleop_follow_wall.py`: Wall-following navigation algorithm
- `keyboard_teleop.py`: Keyboard-based teleoperation interface
- `lidar_point_monitor.py`: LIDAR data monitoring and processing
- `map_coverage_controller.py`: Map coverage planning
- `map_saver.py`: Map saving functionality
- `noisy_controller.py`: Controller with noise simulation
- `room_cleaner.py`: Room cleaning algorithms
- `room_vacuum_controller.py`: Vacuum cleaning controller
- `simple_controller.py`: Basic simple controller implementation
- `test.py`: Testing utilities
- `twist_relay.py`: Twist command relay functionality

### bumperbot_description
Hosts generic simulation and visualization launch files (`gazebo.launch.py`, `display.launch.py`). Robot-specific URDFs and environment assets have been moved into dedicated `robot_*` and `map_*` packages. It still provides some shared resources during the transition.

### bumperbot_firmware
Handles communication with the robot's hardware components and firmware interfaces.

### bumperbot_localization
Implements localization algorithms for determining robot position and orientation in the environment.

### bumperbot_mapping
Provides SLAM (slam_toolbox) launch and tools. Actual map data now lives in dedicated `map_*` packages.

### bumperbot_motion
Contains motion planning and trajectory execution functionalities.

### bumperbot_msgs
Custom ROS message definitions used throughout the system.

### bumperbot_navigation
Integration with ROS navigation stack for path planning and obstacle avoidance.

### bumperbot_planning
Advanced planning algorithms for decision making and route optimization.

### bumperbot_py_examples
Python examples demonstrating various functionalities of the robot system.

### bumperbot_utils
Utility functions and helper classes used across multiple packages.

### robot_bumperbot
The description package for the Bumperbot robot. Contains:
- `urdf/bumperbot.urdf.xacro` (and supporting xacros)
- Robot-specific meshes

This package (plus `robot_description_common`) replaces the old monolithic robot description that lived inside `bumperbot_description`.

### robot_description_common
Common, reusable description components intended to be shared across robots:
- Sensor macros (e.g. OAK-D camera)
- Common meshes for sensors
- Dummy inertial macros, etc.

Robot packages should include from this package instead of duplicating common elements.

### gazebo_models
Contains Gazebo models referenced by worlds via `model://` URIs (AWS RoboMaker residential/warehouse models, etc.). These are added to `GZ_SIM_RESOURCE_PATH` at launch time.

### map_* (map_small_house, map_celisca_floor_1, etc.)
Dedicated packages for each environment. Each typically contains:
- `maps/map.yaml` + `map.pgm` (for localization / navigation)
- `worlds/<name>.world` (SDF world for Gazebo)
- `meshes/` (optional, for worlds that use relative `../meshes/` URIs for floor models)

The launch system automatically looks for a package named `map_<map_name>` when you pass `map_name:=...`.

## Launching Simulations

The primary entry point is `simulated_robot.launch.py`.

### Basic usage

```bash
# From the workspace root after building
source install/setup.bash

# Launch with a specific map in navigation mode
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=nav map_name:=small_house

# SLAM mode (build a new map)
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=slam map_name:=simple_box

# Localization only (uses a pre-existing map)
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=loc map_name:=celisca_floor_1

# Just visualize the robot (no Gazebo)
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=display
```

### Choosing a specific map

Use the `map_name` argument. The launch file constructs the package name as `map_<map_name>`:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=nav map_name:=small_warehouse
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=nav map_name:=celisca_f1_actor
```

Supported map names (as of now) include:
`small_house`, `small_warehouse`, `celisca_floor_1`, `celisca_floor_2`, `celisca_f1_actor`, `celisca_f2_actor`, `simple_box`

(There must be a corresponding `map_<name>` package in your `src/` tree.)

**Note:** The world file inside the map package must be named `<map_name>.world` (e.g. `small_house.world`).

### Real robot

For hardware bringup (no Gazebo):

```bash
ros2 launch bumperbot_bringup real_robot.launch.py use_slam:=false
```

See `real_robot.launch.py` for SLAM vs localization options and sensor drivers.

### Choosing a specific robot

You can launch with a different robot description:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py \
  mode:=nav \
  map_name:=small_house \
  robot_package:=robot_bumperbot \
  robot_xacro:=bumperbot.urdf.xacro \
  robot_name:=bumperbot
```

- `robot_package`: The ROS package that provides the robot (must be built and contain `urdf/<robot_xacro>` + meshes).
- `robot_xacro`: The main xacro file inside `<robot_package>/urdf/`.
- `robot_name`: The name used for the Gazebo model instance and for constructing sensor bridge topics (e.g. oakd camera).

If you create a new robot package `robot_mycoolbot` with `urdf/mycoolbot.urdf.xacro`, you would run:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py \
  mode:=nav map_name:=small_house \
  robot_package:=robot_mycoolbot \
  robot_xacro:=mycoolbot.urdf.xacro \
  robot_name:=mycoolbot
```

**Current limitations with alternate robots:**
- The low-level controller stack (odom, cmd_vel topics) is still namespaced under `bumperbot_controller` by default.
- You may need to adjust controller configs or launch your own controller stack for a truly different robot.

### Other useful arguments

```bash
# Start the LiDAR room vacuum / cleaning controller after mapping
ros2 launch bumperbot_bringup simulated_robot.launch.py \
  mode:=nav map_name:=small_house start_room_vacuum:=true

# Tune the cleaning grid (used by room_vacuum_controller.py)
ros2 launch ... cleaning_grid_spacing_m:=0.25 robot_obstacle_clearance_m:=0.25

# Use simulation time (usually true for Gazebo)
use_sim_time:=true
```

To see all available arguments and their current defaults:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py --show-arguments
```

## Creating a New Map

1. Create a new package directory:
   ```bash
   mkdir -p src/map_myenv/{maps,worlds,meshes}
   ```

2. Add standard package files (`package.xml` + `CMakeLists.txt`):
   ```xml
   <!-- package.xml -->
   <package format="3">
     <name>map_myenv</name>
     ...
     <buildtool_depend>ament_cmake</buildtool_depend>
     <export><build_type>ament_cmake</build_type></export>
   </package>
   ```
   ```cmake
   # CMakeLists.txt
   cmake_minimum_required(VERSION 3.5)
   project(map_myenv)
   find_package(ament_cmake REQUIRED)
   install(DIRECTORY maps worlds meshes DESTINATION share/${PROJECT_NAME})
   ament_package()
   ```

3. Add map data:
   - `maps/map.yaml` + `maps/map.pgm` (standard 2D occupancy grid)

4. Add a Gazebo world:
   - `worlds/myenv.world`
   - The filename **must match** the `map_name` you intend to use because the launcher does `f"{map_name}.world"`.
   - Use `model://` URIs for shared models (they will be found via `gazebo_models`).
   - For custom floor meshes referenced relatively, use `../meshes/foo.stl` and place the STL in the package's `meshes/` folder.

5. Build and launch:
   ```bash
   colcon build --packages-select map_myenv
   source install/setup.bash
   ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=nav map_name:=myenv
   ```

## Creating a New Robot

1. Create the package:
   ```bash
   mkdir -p src/robot_mycoolbot/{urdf,meshes}
   ```

2. `package.xml` (example):
   ```xml
   <package format="3">
     <name>robot_mycoolbot</name>
     <buildtool_depend>ament_cmake</buildtool_depend>
     <exec_depend>robot_state_publisher</exec_depend>
     <exec_depend>xacro</exec_depend>
     <exec_depend>robot_description_common</exec_depend>
     ...
     <export><build_type>ament_cmake</build_type></export>
   </package>
   ```

3. `CMakeLists.txt`:
   ```cmake
   install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
   ```

4. Create the URDF:
   - `urdf/mycoolbot.urdf.xacro` (main file)
   - Include robot-specific gazebo and ros2_control xacros as needed
   - Prefer including sensors from `robot_description_common`:
     ```xml
     <xacro:include filename="$(find robot_description_common)/urdf/oakd_camera.xacro" />
     ```
   - Reference your own meshes with `package://robot_mycoolbot/meshes/...`

5. Launch it:
   ```bash
   ros2 launch bumperbot_bringup simulated_robot.launch.py \
     mode:=nav map_name:=small_house \
     robot_package:=robot_mycoolbot \
     robot_xacro:=mycoolbot.urdf.xacro \
     robot_name:=mycoolbot
   ```

**Tip:** Put robot-agnostic things (common sensors, materials, macros) in `robot_description_common` so multiple robots can reuse them.

## What Has Been Done

1. **Core Architecture**: Established a modular ROS-based architecture with clear separation of concerns
2. **Controller Implementations**: Implemented multiple controller algorithms including automatic teleoperation, wall following, and room cleaning
3. **System Integration**: Integrated various robotics components (mapping, localization, navigation) into a cohesive system
4. **Teleoperation Interface**: Developed keyboard-based and automatic teleoperation capabilities
5. **Map Management**: Implemented map saving and coverage planning functionalities

## Future Development

### Planned Features
1. **Advanced Navigation Algorithms**:
   - Implementation of A* and Dijkstra path planning algorithms
   - Integration with more sophisticated SLAM algorithms (like Hector SLAM or RTAB-Map)
   - Multi-robot coordination capabilities

2. **Enhanced Localization**:
   - Improved sensor fusion for better localization accuracy
   - Implementation of particle filter-based localization
   - Integration with GPS and IMU data

3. **Machine Learning Integration**:
   - Reinforcement learning-based navigation algorithms
   - Computer vision integration for object recognition
   - Adaptive control systems using neural networks

4. **Improved User Interface**:
   - Web-based visualization dashboard
   - Mobile application for remote control
   - Enhanced logging and monitoring capabilities

5. **Hardware Expansion**:
   - Support for additional sensor types (cameras, ultrasonic sensors)
   - Integration with different robot platforms
   - Improved firmware communication protocols

6. **Performance Optimization**:
   - Real-time performance improvements
   - Resource utilization optimization
   - Cloud integration for data processing

### Technical Improvements
1. **Code Quality**: 
   - Comprehensive unit testing for all components
   - Documentation improvements across all packages
   - Code refactoring for better maintainability

2. **Scalability**:
   - Support for larger-scale robotic systems
   - Distributed computing capabilities
   - Modular design for easy extension

This project provides a solid foundation for robotics research and development, with clear pathways for future enhancements and extensions.

## Credits

This workspace includes the following upstream robot description assets (bundled directly):

| Repository | Original URL | License |
|---|---|---|
| Awesome-URDFs | https://github.com/code-name-57/Awesome-URDFs | MIT |
| Berkeley-Humanoid-Lite | https://github.com/HybridRobotics/Berkeley-Humanoid-Lite | BSD-3-Clause |
| Berkeley-Humanoid-Lite-Assets | https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets | BSD-3-Clause |