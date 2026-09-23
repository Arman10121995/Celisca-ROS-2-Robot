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
  PROGRAMS scripts/export_3d_map.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_profiles test/test_sim_profiles.py TIMEOUT 180)
  ament_add_pytest_test(simulator_backends test/test_simulator_backends.py TIMEOUT 180)
  ament_add_pytest_test(algorithm_dispatch test/test_algorithm_dispatch.py TIMEOUT 120)
  ament_add_pytest_test(backend_assets test/test_backend_assets.py TIMEOUT 300)
  ament_add_pytest_test(isaac_runtime_shutdown test/test_isaac_runtime_shutdown.py TIMEOUT 120)
  ament_add_pytest_test(sim_frames test/test_sim_frames.py TIMEOUT 120)
  ament_add_pytest_test(mujoco_reset test/test_mujoco_reset.py TIMEOUT 120)
  ament_add_pytest_test(mujoco_effort test/test_mujoco_effort.py TIMEOUT 120)
  ament_add_pytest_test(mujoco_runtime test/test_mujoco_runtime.py TIMEOUT 120)
  ament_add_pytest_test(mujoco_world_collision test/test_mujoco_world_collision.py TIMEOUT 120)
  ament_add_pytest_test(description_resources test/test_description_resources.py TIMEOUT 120)
  ament_add_pytest_test(world_mesh_staging test/test_world_mesh_staging.py TIMEOUT 120)
  ament_add_pytest_test(process_lifetime test/test_process_lifetime.py TIMEOUT 60)
endif()

ament_package()
