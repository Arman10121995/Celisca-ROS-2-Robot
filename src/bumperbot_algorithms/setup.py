from setuptools import setup, find_packages

package_name = 'bumperbot_algorithms'

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
            'obstacle_detector = bumperbot_algorithms.perception:obstacle_detector_main',
            'scan_clusterer = bumperbot_algorithms.perception:scan_clusterer_main',
            'pointcloud_segmenter = bumperbot_algorithms.perception:pointcloud_segmenter_main',
            # Localization
            'dead_reckoning = bumperbot_algorithms.localization:dead_reckoning_main',
            # State estimation
            'ekf_3d_estimator = bumperbot_algorithms.state_estimation:ekf_3d_estimator_main',
            'motion_model_estimator = bumperbot_algorithms.state_estimation:motion_model_estimator_main',
            'pose_graph_estimator = bumperbot_algorithms.state_estimation:pose_graph_estimator_main',
            # Sensor fusion
            'wheel_imu_fusion = bumperbot_algorithms.sensor_fusion:wheel_imu_fusion_main',
            'gps_odom_fusion = bumperbot_algorithms.sensor_fusion:gps_odom_fusion_main',
            'complementary_imu = bumperbot_algorithms.sensor_fusion:complementary_imu_main',
            # Global planning
            'rrt_planner = bumperbot_algorithms.global_planning:rrt_planner_main',
            'voronoi_planner = bumperbot_algorithms.global_planning:voronoi_planner_main',
            # Local planning
            'follow_the_gap = bumperbot_algorithms.local_planning:follow_the_gap_main',
        ],
    },
)
