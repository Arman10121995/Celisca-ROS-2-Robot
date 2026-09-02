# Robot Lab Support Matrix (P7.6)

Measured evidence and known limits for all supported configurations.

Last updated: 2026-09-02

## Platform Support

| Component | Status | Evidence |
|-----------|--------|----------|
| Ubuntu 22.04 | ✅ Supported | CI runs on ubuntu-22.04 |
| ROS 2 Humble | ✅ Supported | Primary target distribution |
| arm64 | ✅ Supported | Platform tested |
| Gazebo (Harmonic) | ✅ Supported | `gz sim` headless verified: nav_maze + celisca_floor_1 worlds load with zero errors; furniture STL (165 MB, LFS-restored) resolves via GZ_SIM_RESOURCE_PATH set in gazebo.launch.py |
| Isaac Sim | 🚧 Image Built | `isaac-sim-docker:latest` (26.1 GB) source-built on aarch64 via `tools/docker/` pipeline; stored on 1 TB SSD. Remaining: nvidia runtime registration + container boot test + `robot_lab_isaac` smoke |
| PyBullet | ✅ Qualified | 3.2.7 rebuilt from source against NumPy 2.2.6 (venv + user site); launch verified live on Jetson: spawner runs clean, publishes /clock /odom /scan /imu/out /joint_states |
| MuJoCo | ✅ Qualified | 3.12.0 (C lib source-built on SSD + pip bindings); launch verified live on Jetson: spawner runs clean, publishes /clock /odom /scan /imu/out /joint_states; viewer segfault fix (no .close() at shutdown on ARM) |
| ORB-SLAM3 | ⚠️ Optional | OpenCV ABI warning if versions mismatch |

## Robot Support

| Robot | Type | Status | Algorithms | Evidence |
|-------|------|--------|------------|----------|
| Bumperbot | Differential drive | ✅ Integrated | Full stack (perception, planning, localization, control) | 28 qualification tests, smoke test passes |
| Labbot | Differential drive | ✅ Integrated | Full stack | 23 qualification tests, xacro validated |
| Go2 | Quadruped | ✅ Simulated | 12-DOF leg control, IMU/camera/odometry | 23 qualification tests, xacro validated |
| Berkeley Humanoid Lite | Humanoid | ✅ Simulated | 22-DOF position control | Qualification suite, xacro validated |
| Quadrotor SITL | Aerial | ✅ Simulated | MAVLink attitude/position control | Qualification suite |

## Algorithm Coverage

| Category | Count | Status | Algorithms |
|----------|-------|--------|------------|
| Perception | 8 | ✅ | obstacle_detector, scan_clusterer, pointcloud_segmenter + 5 legacy |
| Localization | 6 | ✅ | dead_reckoning + 5 legacy |
| State Estimation | 5 | ✅ | ekf_3d_estimator, motion_model_estimator, pose_graph_estimator + 2 legacy |
| Sensor Fusion | 5 | ✅ | wheel_imu_fusion, gps_odom_fusion, complementary_imu + 2 legacy |
| Global Planning | 5 | ✅ | rrt_planner, voronoi_planner + 3 legacy |
| Local Planning | 5 | ✅ | follow_the_gap + 4 legacy |
| Control | 9 | ✅ | mavros_offboard_controller, joint_effort_commander + 7 legacy |

## Environment Support

| Environment | Type | Status | Evidence |
|-------------|------|--------|----------|
| small_office | Indoor | ✅ | Primary test environment |
| small_house | Indoor | ✅ | Qualified |
| small_warehouse | Indoor | ✅ | Qualified |
| warehouse_demo | Indoor | ✅ | Qualified |
| nav_empty | Nav arena | ✅ | Deterministic, seeded |
| nav_obstacle | Nav arena | ✅ | Deterministic, seeded |
| nav_maze | Nav arena | ✅ | Deterministic, seeded |
| nav_narrow_passage | Nav arena | ✅ | Deterministic, seeded |
| nav_warehouse | Nav arena | ✅ | Deterministic, seeded |
| nav_dynamic | Dynamic | ✅ | Scripted moving actors |
| nav_sensor_degraded | Sensor | ✅ | Blind-corner occluding walls |
| terrain_rough | Terrain | ✅ | Rough outdoor terrain |
| terrain_stairs | Terrain | ✅ | Stair climbing |
| terrain_stepping_stones | Terrain | ✅ | Stepping stone navigation |
| aerial_course | Aerial | ✅ | 3D slalom course |
| aerial_indoor | Aerial | ✅ | Multi-level indoor course |

## Test Evidence

| Metric | Value |
|--------|-------|
| Registry unit tests | 257 |
| Passing | 257 |
| Failing | 0 |
| Errors | 0 |
| Bringup profile tests | 206 (incl. 7 simulator dispatch + 6 backend smoke) |
| CI status | ✅ Passing |

## Known Limits

| Limit | Description |
|-------|-------------|
| Hardware HIL | Physical Bumperbot validation pending (P7.5 blocked) |
| ORB-SLAM3 | OpenCV ABI must match ROS cv_bridge |
| Simulator backends | PyBullet and MuJoCo both implement complete ROS2 spawner (+GUI, +laser scan via mj_ray). ASCII STL meshes in robot URDF exceed MuJoCo ushort mesh limit -> fallback basic-primitive MJCF used (spawner falls back gracefully). Isaac Sim docker image built (26.1 GB on SSD); GPU wiring + container smoke pending. All large artifacts (docker data-root, containerd store, packman cache, Omniverse ext cache, Isaac Sim source) live on the 1 TB SSD — internal eMMC is ~87% full and must never receive build artifacts (recovery: scripts/fix_containerd_disk.sh) |
| Real-time | Real-time performance not benchmarked on hardware |
| Multi-robot | No multi-robot scenarios tested |

## Benchmark Infrastructure

| Component | Status | Evidence |
|-----------|--------|----------|
| Result schema | ✅ | BenchmarkResult with versioned fields |
| Launch orchestration | ✅ | LaunchOrchestrator with rosbag capture |
| Ground-truth extraction | ✅ | GroundTruthAdapter |
| Metric normalization | ✅ | MetricNormalizer with composite scoring |
| Output generation | ✅ | JSON, CSV, Markdown, HTML, plots |
| Reference baselines | ✅ | Seeded results for bumperbot smoke test |
| Regression checking | ✅ | Per-metric threshold validation |
