from setuptools import setup, find_packages

package_name = 'robot_lab_algorithms'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Lab Team',
    maintainer_email='robot-lab@example.com',
    description='Bumperbot algorithm breadth: perception, localization, state estimation, sensor fusion, planning (P5)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Perception
            'obstacle_detector = robot_lab_algorithms.perception:obstacle_detector_main',
            'scan_clusterer = robot_lab_algorithms.perception:scan_clusterer_main',
            'pointcloud_segmenter = robot_lab_algorithms.perception:pointcloud_segmenter_main',
            'euclidean_clusterer = robot_lab_algorithms.perception:euclidean_clusterer_main',
            'dbscan_clusterer = robot_lab_algorithms.perception:dbscan_clusterer_main',
            'ransac_ground_removal = robot_lab_algorithms.perception:ransac_ground_removal_main',
            # Localization
            'dead_reckoning = robot_lab_algorithms.localization:dead_reckoning_main',
            'amcl = robot_lab_algorithms.localization:amcl_main',
            'icp_localization = robot_lab_algorithms.localization:icp_localization_main',
            'ndt_localization = robot_lab_algorithms.localization:ndt_localization_main',
            'rgbd_slam_localization = robot_lab_algorithms.localization:rgbd_slam_localization_main',
            # State estimation
            'ekf_3d_estimator = robot_lab_algorithms.state_estimation:ekf_3d_estimator_main',
            'motion_model_estimator = robot_lab_algorithms.state_estimation:motion_model_estimator_main',
            'pose_graph_estimator = robot_lab_algorithms.state_estimation:pose_graph_estimator_main',
            'linear_kalman_filter = robot_lab_algorithms.state_estimation:linear_kalman_filter_main',
            'ukf_estimator = robot_lab_algorithms.state_estimation:ukf_estimator_main',
            'particle_filter = robot_lab_algorithms.state_estimation:particle_filter_main',
            'error_state_ekf = robot_lab_algorithms.state_estimation:error_state_ekf_main',
            # Sensor fusion
            'wheel_imu_fusion = robot_lab_algorithms.sensor_fusion:wheel_imu_fusion_main',
            'gps_odom_fusion = robot_lab_algorithms.sensor_fusion:gps_odom_fusion_main',
            'complementary_imu = robot_lab_algorithms.sensor_fusion:complementary_imu_main',
            'mahony_filter = robot_lab_algorithms.sensor_fusion:mahony_filter_main',
            'madgwick_filter = robot_lab_algorithms.sensor_fusion:madgwick_filter_main',
            'wheel_imu_gnss_ukf = robot_lab_algorithms.sensor_fusion:wheel_imu_gnss_ukf_main',
            # Global planning
            'rrt_planner = robot_lab_algorithms.global_planning:rrt_planner_main',
            'voronoi_planner = robot_lab_algorithms.global_planning:voronoi_planner_main',
            'dijkstra_planner = robot_lab_algorithms.global_planning:dijkstra_planner_main',
            'a_star_planner = robot_lab_algorithms.global_planning:a_star_planner_main',
            'prm_planner = robot_lab_algorithms.global_planning:prm_planner_main',
            'rrt_star_planner = robot_lab_algorithms.global_planning:rrt_star_planner_main',
            # Local planning
            'follow_the_gap = robot_lab_algorithms.local_planning:follow_the_gap_main',
            'dwb_local_planner = robot_lab_algorithms.local_planning:dwb_local_planner_main',
            'regulated_pure_pursuit = robot_lab_algorithms.local_planning:regulated_pure_pursuit_main',
            'teb_local_planner = robot_lab_algorithms.local_planning:teb_local_planner_main',
            'mppi_local_planner = robot_lab_algorithms.local_planning:mppi_local_planner_main',
            # Control
            'pid_controller = robot_lab_algorithms.control:pid_controller_main',
            'lqr_controller = robot_lab_algorithms.control:lqr_controller_main',
            'mpc_controller = robot_lab_algorithms.control:mpc_controller_main',
            'nonlinear_mpc = robot_lab_algorithms.control:nonlinear_mpc_main',
            'feedback_linearization = robot_lab_algorithms.control:feedback_linearization_main',
            'backstepping_controller = robot_lab_algorithms.control:backstepping_main',
        ],
    },
)
