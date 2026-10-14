cmake_minimum_required(VERSION 3.8)
project(robot_lab_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/export_3d_map.py scripts/native_arm_scene.py
    scripts/wait_native_sensors.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/native_scene_objects.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # This host's user site carries an anyio pytest plugin that needs a newer
  # pytest than Humble's, so `python3 -m pytest --version` failed and ament
  # silently registered none of these tests.  None of them uses an
  # autoloaded plugin, so autoload is off for the check and for each run.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_profiles test/test_sim_profiles.py TIMEOUT 180 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(simulator_backends test/test_simulator_backends.py TIMEOUT 180 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(algorithm_dispatch test/test_algorithm_dispatch.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(backend_assets test/test_backend_assets.py TIMEOUT 300 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(isaac_runtime_shutdown test/test_isaac_runtime_shutdown.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(sim_frames test/test_sim_frames.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(mujoco_reset test/test_mujoco_reset.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(mujoco_effort test/test_mujoco_effort.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(mujoco_runtime test/test_mujoco_runtime.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(mujoco_world_collision test/test_mujoco_world_collision.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(description_resources test/test_description_resources.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(world_mesh_staging test/test_world_mesh_staging.py TIMEOUT 120 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(process_lifetime test/test_process_lifetime.py TIMEOUT 60 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(mode_capability test/test_mode_capability.py TIMEOUT 60 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(mobile_task_and_takeoff test/test_mobile_task_and_takeoff.py TIMEOUT 60 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(native_planning_description test/test_native_planning_description.py TIMEOUT 60 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(native_hand test/test_native_hand.py TIMEOUT 60 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(native_hand_object test/test_native_hand_object.py TIMEOUT 60 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(fleet_contract test/test_fleet_contract.py TIMEOUT 60 ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
