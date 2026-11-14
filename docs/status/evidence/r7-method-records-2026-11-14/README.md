# Algorithm method records

Generated 2026-11-14 by `scripts/algorithm_method_records.py` from the catalog, the dispatch, the R7 method subrecords and the test files. Installed and tested are facts about files on this host; they do not qualify a method.

Summary: methods 78, installed 72, not_installed 0, unavailable 6, with_subrecord 42, with_tests 57, name_math_consistent 59, name_math_corrected 9, name_math_mismatch 8, name_math_not_a_method 2.


## control

| Method | Dispatch | Installed | Tests | Mathematics | Name/math |
|---|---|---|---|---|---|
| `backstepping_controller` | node robot_lab_algorithms/nav2_local_planner planner=backstepping | yes | 3 | Lyapunov/backstepping tracking law for the unicycle | consistent |
| `bicycle_mpc` | node robot_lab_algorithms/nav2_bicycle planner=bicycle_mpc | yes | 1 | Linearised kinematic bicycle, steering-rate MPC over a 3-20 step horizon, warm-started, with a solve deadline | consistent |
| `bicycle_pure_pursuit` | node robot_lab_algorithms/nav2_bicycle planner=bicycle_pure_pursuit | yes | 0 | Kinematic bicycle at the fixed axle; curvature 2 y / L_d^2 to a lookahead point, kappa = tan(delta) / L, steering and slew limits from the source | consistent |
| `bicycle_stanley` | node robot_lab_algorithms/nav2_bicycle planner=bicycle_stanley | yes | 1 | Stanley law delta = heading error + atan(k e / v) at the fixed axle, with a rear-steer feedback adaptation and source steering limits | consistent |
| `cleaning_controller` | node robot_lab_controller/cleaning_controller.py | yes | 0 | Boustrophedon-style coverage lanes over the cleanable grid cells, tracking cleaned cells | consistent |
| `feedback_linearization` | node robot_lab_algorithms/nav2_local_planner planner=feedback_linearization | yes | 3 | Static input-output linearisation of a point ahead of the wheel axle | consistent |
| `humanoid_policy_controller` | node robot_lab_adapter/humanoid-policy-controller | yes | 1 | Learned ONNX policy: joint/IMU observation to joint position targets | consistent |
| `humanoid_standing_controller` | node robot_lab_adapter/humanoid-standing-controller | yes | 2 | Joint PD to a standing pose (legs Kp 120, Kd 4 N m/rad) plus an IMU ankle balance strategy and a latched tilt monitor | consistent |
| `joint_effort_commander` | node robot_lab_adapter/joint-effort-commander | yes | 0 | Fixed-rate effort command relay to a forward_command_controller (no feedback law) | consistent |
| `lqr_controller` | node robot_lab_algorithms/nav2_local_planner planner=lqr | yes | 3 | Discrete LQR on the lateral/heading error linearised about the path | consistent |
| `map_coverage_controller` | node robot_lab_controller/mapping_controller.py | yes | 0 | Frontier exploration of the occupancy grid, then wall following to confirm cells | consistent |
| `mavros_offboard_controller` | unavailable retired - the zero-thrust AttitudeTarget demonstration does not hover; fly px4_x500 in mode:=flight | no | 2 | Retired: constant zero-thrust AttitudeTarget (no control law) | **corrected**: Retired zero-thrust AttitudeTarget stub whose node raises on start, yet its dispatch still started it. Now unavailable, status blocked. |
| `mpc_controller` | node robot_lab_algorithms/nav2_local_planner planner=linear_mpc | yes | 2 | Linear time-varying MPC on the lateral error model with hard turn-rate bounds | consistent |
| `native_holonomic_controller` | stack controller | yes | 1 | Mecanum / four-wheel-steer inverse kinematics from body twist to wheel commands | consistent |
| `native_tracked_controller` | stack native_tracked | yes | 0 | Bounded command gate to Gazebo TrackController belt contact (skid steering) | consistent |
| `noisy_controller` | node robot_lab_controller/noisy_controller | yes | 0 | Differential-drive wheel odometry with Gaussian encoder noise (sigma 0.005 rad) | **corrected**: Was catalogued as PID; it is wheel odometry with Gaussian encoder noise (sigma 0.005 rad), with no feedback law. |
| `nonlinear_mpc` | node robot_lab_algorithms/nav2_local_planner planner=nonlinear_mpc | yes | 2 | Nonlinear MPC on the unicycle kinematics, single shooting with an adjoint gradient | consistent |
| `pid_controller` | node robot_lab_algorithms/nav2_local_planner planner=pid | yes | 3 | PID on a heading error that folds in the lateral offset, with curvature feedforward | consistent |
| `simple_controller` | stack controller | yes | 3 | Differential-drive inverse kinematics v_l,r = (v -/+ w b/2) / r and wheel odometry | **corrected**: Was catalogued as PID; it is differential-drive inverse kinematics plus wheel odometry, with no feedback law. |
| `twist_relay` | node robot_lab_controller/twist_relay | yes | 0 | Twist / TwistStamped relay | **not_a_method**: Message relay, not a control law. |

## global_planning

| Method | Dispatch | Installed | Tests | Mathematics | Name/math |
|---|---|---|---|---|---|
| `a_star_planner` | plugin nav2_smac_planner/SmacPlanner2D | yes | 6 | A* on the costmap grid with cost-aware edges | **corrected**: Runs Nav2 Smac 2D (A* with a cost-aware heuristic); the catalog named the undispatched robot_lab_planning::AStarPlanner. |
| `dijkstra_planner` | plugin nav2_navfn_planner/NavfnPlanner | yes | 6 | Dijkstra wavefront navigation function on the costmap (NavFn with use_astar false) | **mismatch**: Dispatches the same NavFn plugin as navfn_planner (use_astar false, i.e. Dijkstra), so the two ids are one method and not separate comparison arms; robot_lab_planning::DijkstraPlanner is built but not dispatched. same dispatch entry as navfn_planner |
| `hybrid_a_star_planner` | plugin nav2_smac_planner/SmacPlannerHybrid | yes | 0 | A* over (x, y, heading) with Dubins/Reeds-Shepp motion primitives | consistent |
| `navfn_planner` | plugin nav2_navfn_planner/NavfnPlanner | yes | 7 | Dijkstra potential field propagated from the goal | **corrected**: Was catalogued as a potential field; NavFn is a Dijkstra wavefront navigation function (A* with use_astar) descended by gradient, i.e. grid search. same dispatch entry as dijkstra_planner |
| `prm_planner` | node robot_lab_algorithms/nav2_global_planner planner=prm_planner | yes | 2 | Probabilistic roadmap: 600 free samples, k = 12 nearest-neighbour edges, Dijkstra | consistent |
| `rrt_planner` | node robot_lab_algorithms/nav2_global_planner planner=rrt_planner | yes | 5 | Goal-biased rapidly-exploring random tree with collision-checked edges | consistent |
| `rrt_star_planner` | node robot_lab_algorithms/nav2_global_planner planner=rrt_star_planner | yes | 3 | RRT with best-parent choice and rewiring, anytime within 1 s | consistent |
| `state_lattice_planner` | plugin nav2_smac_planner/SmacPlannerLattice | yes | 0 | Search over precomputed kinematically feasible motion primitives | consistent |
| `voronoi_planner` | node robot_lab_algorithms/nav2_global_planner planner=voronoi_planner | yes | 4 | Path along the generalized Voronoi diagram (maximum clearance) | consistent |

## local_planning

| Method | Dispatch | Installed | Tests | Mathematics | Name/math |
|---|---|---|---|---|---|
| `dwb_local_planner` | plugin dwb_core::DWBLocalPlanner | yes | 4 | Trajectory rollout: dynamic window velocity samples, rollouts scored with critics | consistent |
| `follow_the_gap` | node robot_lab_algorithms/nav2_local_planner planner=follow_the_gap | yes | 3 | Reactive: steer toward the free scan gap closest to the path direction | consistent |
| `mppi_controller` | plugin nav2_mppi_controller::MPPIController | yes | 1 | Trajectory optimization: model predictive path integral over 2000 noisy control sequences (2.8 s), weighted average | consistent |
| `mppi_local_planner` | unavailable placeholder - it scores random velocities by a constant goal distance, with no horizon rollout or obstacle cost; use mppi_controller (Nav2 MPPI) | no | 2 | Retired placeholder: random velocities scored by goal distance (no rollout) | consistent |
| `pd_motion_planner` | node robot_lab_motion/pd_motion_planner.py | yes | 0 | PD law on distance and heading error to a lookahead point on the path | **mismatch**: Catalogued as a local planner; it is a PD law on pose error toward a lookahead point, with no obstacle term or trajectory search. |
| `pure_pursuit` | plugin nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController | yes | 1 | Pure pursuit on a lookahead point with curvature and cost speed regulation | **corrected**: Runs Nav2 Regulated Pure Pursuit; the catalog named it plain pure pursuit in robot_lab_motion. The id regulated_pure_pursuit runs robot_lab's own copy of the same law. |
| `regulated_pure_pursuit` | node robot_lab_algorithms/nav2_local_planner planner=regulated_pure_pursuit | yes | 2 | Pure pursuit with curvature slowdown below a 0.9 m turn radius, served as Nav2 FollowPath | consistent |
| `teb_local_planner` | unavailable cataloged only - nav2_teb_planner is not built in this workspace | no | 5 | Timed elastic band: sparse nonlinear least squares over poses and time intervals (upstream; not installed) | **corrected**: The catalog named nav2_teb_planner, which does not exist, and said integrated; upstream is teb_local_planner (TebLocalPlannerROS), not installed. Now cataloged; follow_the_gap is the R7.7 replacement. |

## localization

| Method | Dispatch | Installed | Tests | Mathematics | Name/math |
|---|---|---|---|---|---|
| `amcl` | stack global_localization | yes | 13 | Adaptive Monte Carlo localization: particle filter with KLD sampling and a likelihood-field laser model | consistent |
| `correlative_localization` | node robot_lab_algorithms/correlative_localization | yes | 0 | Exhaustive coarse-to-fine search on a Gaussian likelihood field | consistent |
| `dead_reckoning` | node robot_lab_algorithms/dead_reckoning | yes | 3 | Composition of odometry increments | consistent |
| `hector_slam` | unavailable cataloged only - hector_slam is not built in this workspace | no | 1 | Gauss-Newton scan-to-map matching on a multi-resolution grid (upstream; no Humble release) | consistent |
| `icp_localization` | node robot_lab_algorithms/icp_localization | yes | 1 | Iterative closest point to the map surface cells, closed-form 2D rigid alignment | consistent |
| `ndt_localization` | node robot_lab_algorithms/ndt_localization | yes | 1 | Normal distributions transform on 0.5 m cells, reweighted Gauss-Newton with line search | consistent |
| `odometry_motion_model` | node robot_lab_localization/odometry_motion_model | yes | 0 | Sampling odometry motion model (rotation-translation-rotation, noise alpha1-alpha4) | consistent |
| `plicp_localization` | node robot_lab_algorithms/plicp_localization | yes | 0 | Gauss-Newton on point-to-line distances with map normals and robust weights | consistent |
| `rgbd_slam_localization` | unavailable retired 2026-10-13 - the node was a simplified stand-in; RGB-D localization runs as rtabmap_localization | no | 1 | Retired stand-in; RGB-D localization runs as rtabmap_localization | consistent |
| `robot_localization_ekf` | stack local_localization | yes | 0 | robot_localization EKF: 15-state omnidirectional model, odometry and IMU updates | **mismatch**: Same robot_localization ekf_node as ekf_localization_node (one method, two ids); the catalog executable was nav2_ekf_node, which is not a binary (corrected to ekf_node). |
| `rtabmap_localization` | stack rtabmap | yes | 1 | RGB-D appearance-based localization: bag-of-words loop closure and graph optimization | consistent |
| `slam_toolbox` | stack slam | yes | 0 | Pose-graph SLAM: correlative scan matching, loop closure, sparse pose-graph optimization | consistent |

## perception

| Method | Dispatch | Installed | Tests | Mathematics | Name/math |
|---|---|---|---|---|---|
| `camera_calibration` | node robot_lab_algorithms/camera_calibration | yes | 0 | Zhang planar chessboard calibration (cv2.calibrateCamera) with sub-pixel corners | consistent |
| `costmap_2d_observation` | stack navigation_costmap | yes | 3 | Nav2 ObstacleLayer: marks sensor hits and ray-traces free space into the costmap | **corrected**: The catalog named nav2_costmap_2d::ObservationLayer, which does not exist; the Nav2 ObstacleLayer (observation buffers, ray-traced clearing) is what runs. |
| `dbscan_clusterer` | node robot_lab_algorithms/dbscan_clusterer | yes | 2 | Density-based clustering with core, border and noise points | consistent |
| `depth_image_to_pointcloud` | node robot_lab_algorithms/depth_image_to_pointcloud | yes | 0 | Pinhole back-projection p = d K^-1 [u v 1]^T | consistent |
| `euclidean_clusterer` | node robot_lab_algorithms/euclidean_clusterer | yes | 2 | Region growing on a KD-tree (PCL EuclideanClusterExtraction) | consistent |
| `feature_detection` | node robot_lab_algorithms/feature_detection | yes | 0 | Feature matching to a keyframe (Lowe ratio test), depth lift to 3D, PnP with RANSAC | consistent |
| `laser_scan_to_pointcloud` | node pointcloud_to_laserscan/laserscan_to_pointcloud_node | yes | 2 | Polar-to-Cartesian projection of scan rays (laserscan_to_pointcloud_node) | **corrected**: Its dispatch and launch fragment started pointcloud_to_laserscan_node, the reverse conversion; now laserscan_to_pointcloud_node. Runtime availability is determined from the current package index below. |
| `obstacle_detector` | node robot_lab_algorithms/obstacle_detector | yes | 3 | Connected components of scan points under a distance threshold | consistent |
| `pointcloud_segmenter` | node robot_lab_algorithms/pointcloud_segmenter | yes | 3 | Ground / non-ground split by height threshold in the upright target frame | consistent |
| `ransac_ground_removal` | node robot_lab_algorithms/ransac_ground_removal | yes | 2 | RANSAC plane hypotheses restricted to near-vertical normals, least-squares refinement on the inliers | consistent |
| `scan_clusterer` | node robot_lab_algorithms/scan_clusterer | yes | 3 | Sequential breakpoint detection along the scan, fixed or adaptive (range-dependent) threshold | consistent |

## sensor_fusion

| Method | Dispatch | Installed | Tests | Mathematics | Name/math |
|---|---|---|---|---|---|
| `attitude_eskf` | node robot_lab_algorithms/attitude_eskf | yes | 0 | Error-state Kalman filter on attitude and gyro bias | consistent |
| `complementary_imu` | node robot_lab_algorithms/complementary_imu | yes | 2 | Gyro integration with roll and pitch pulled toward the accelerometer tilt | consistent |
| `gps_odom_fusion` | node robot_lab_algorithms/gps_odom_fusion | yes | 2 | The wheel/IMU EKF with GNSS position updates | consistent |
| `imu_complementary_filter` | node imu_filter_madgwick/imu_filter_madgwick_node | yes | 0 | Madgwick gradient-descent attitude filter (imu_tools, upstream C++) | **mismatch**: Id says complementary filter; it runs imu_tools' Madgwick gradient-descent filter (imu_filter_madgwick_node); the imu_tools complementary filter package is not installed. Kept as the upstream Madgwick reference beside madgwick_filter; family corrected to gradient_descent. |
| `imu_republisher` | stack local_localization | yes | 1 | IMU frame/topic relay | **not_a_method**: Frame/topic relay, not an estimator. |
| `madgwick_filter` | node robot_lab_algorithms/madgwick_filter | yes | 1 | One normalised gradient step per sample on the gravity objective | consistent |
| `mahony_filter` | node robot_lab_algorithms/mahony_filter | yes | 1 | PI feedback of the gravity cross error into the gyro | consistent |
| `wheel_imu_fusion` | node robot_lab_algorithms/wheel_imu_fusion | yes | 2 | EKF on [x, y, theta, v, b_g] with a zero angular rate update at standstill | consistent |
| `wheel_imu_gnss_ukf` | node robot_lab_algorithms/wheel_imu_gnss_ukf | yes | 1 | Unscented Kalman filter on the same model and measurements | consistent |

## state_estimation

| Method | Dispatch | Installed | Tests | Mathematics | Name/math |
|---|---|---|---|---|---|
| `ekf_3d_estimator` | node robot_lab_algorithms/ekf_3d_estimator | yes | 2 | Extended Kalman filter on the constant turn rate and velocity model | **mismatch**: Id says 3D; the node runs a planar constant turn-rate and velocity (CTRV) EKF (tracking_filters.CtrvEkf). state_estimation.EKF3DEstimator (per-axis scalar gains, not an EKF) is unused legacy code. |
| `ekf_localization_node` | stack local_localization | yes | 2 | robot_localization EKF: 15-state omnidirectional model, odometry and IMU updates | **mismatch**: Same robot_localization ekf_node as robot_localization_ekf (one method, two ids). |
| `error_state_ekf` | unavailable retired 2026-10-13 - its node filtered noiseless ground truth; the error-state formulation runs for attitude as sensor_fusion attitude_eskf | no | 1 | Retired: filtered noiseless truth; the error-state formulation runs as attitude_eskf | consistent |
| `kalman_filter_1d` | node robot_lab_localization/kalman_filter | yes | 0 | Scalar Kalman filter on yaw rate fusing wheel odometry and IMU | consistent |
| `linear_kalman_filter` | node robot_lab_algorithms/linear_kalman_filter | yes | 1 | Kalman filter on [x, y, vx, vy] | consistent |
| `motion_model_estimator` | node robot_lab_algorithms/motion_model_estimator | yes | 2 | IMM of a straight-line and a turning CTRV EKF | **mismatch**: Generic id; the node runs an interacting multiple model filter (constant velocity + coordinated turn). The fixed-gain MotionModelEstimator class is unused legacy code. |
| `particle_filter` | node robot_lab_algorithms/particle_filter | yes | 1 | Sequential importance resampling on the CTRV model | consistent |
| `pose_graph_estimator` | node robot_lab_algorithms/pose_graph_estimator | yes | 2 | Fixed-lag least squares over the last 20 fixes with constant-velocity factors | **mismatch**: Id says pose graph; the node runs a 2 s sliding-window least-squares smoother over GNSS fixes with motion constraints (a small pose graph without loop closures). The exponential-decay PoseGraphEstimator class is unused legacy code. |
| `ukf_estimator` | node robot_lab_algorithms/ukf_estimator | yes | 1 | Unscented Kalman filter on the CTRV model | consistent |
