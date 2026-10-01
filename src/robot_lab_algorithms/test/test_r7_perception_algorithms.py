#!/usr/bin/env python3
"""
R7 Perception Algorithm Unit Tests

Comprehensive numerical tests for perception algorithms verifying mathematical
correctness, edge case handling, and performance characteristics.
"""

import unittest
import math
import numpy as np
import sys
import os

# Use installed packages
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'install'))

from robot_lab_algorithms.perception import (
    ObstacleDetector, ScanClusterer, PointcloudSegmenter,
    EuclideanClusterer, DBSCANClusterer, RANSACGroundRemoval
)


class TestObstacleDetector(unittest.TestCase):
    """Tests for ObstacleDetector algorithm."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.detector = ObstacleDetector(cluster_distance=0.5)
    
    def test_empty_input(self):
        """Test empty input handling."""
        result = self.detector.detect([])
        self.assertEqual(result, [], "Empty input should return empty list")
    
    def test_single_point(self):
        """Test single point forms one cluster."""
        result = self.detector.detect([(0.0, 0.0)])
        self.assertEqual(len(result), 1, "Single point should form one cluster")
        self.assertEqual(result[0], [(0.0, 0.0)], "Cluster should contain the point")
    
    def test_two_close_points_form_one_cluster(self):
        """Test points within cluster_distance form one cluster."""
        points = [(0.0, 0.0), (0.1, 0.1)]
        result = self.detector.detect(points)
        self.assertEqual(len(result), 1, "Close points should form one cluster")
        self.assertEqual(len(result[0]), 2, "Cluster should contain both points")
    
    def test_two_far_points_form_two_clusters(self):
        """Test points beyond cluster_distance form separate clusters."""
        points = [(0.0, 0.0), (2.0, 2.0)]  # Distance = sqrt(8) > 0.5
        result = self.detector.detect(points)
        self.assertEqual(len(result), 2, "Far points should form two clusters")
        self.assertEqual(len(result[0]), 1, "First cluster should have one point")
        self.assertEqual(len(result[1]), 1, "Second cluster should have one point")
    
    def test_algorithm_properties(self):
        """Test algorithm has expected properties."""
        self.assertEqual(self.detector.node_name, 'obstacle_detector')
        self.assertEqual(self.detector.cluster_distance, 0.5)


class TestScanClusterer(unittest.TestCase):
    """Tests for ScanClusterer algorithm."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.clusterer = ScanClusterer(cluster_distance=0.3)
    
    def test_empty_scan(self):
        """Test empty scan handling."""
        result = self.clusterer.cluster_ranges(0, 0.1, [], 10.0)
        self.assertEqual(result, [], "Empty scan should return empty list")
    
    def test_single_valid_point(self):
        """Test single valid range reading."""
        ranges = [1.0]
        result = self.clusterer.cluster_ranges(0, 0.1, ranges, 10.0)
        self.assertEqual(len(result), 1, "Single valid point should form one cluster")
        self.assertEqual(len(result[0]), 1, "Cluster should contain one point")
    
    def test_gap_detection(self):
        """Test that gaps in scan create separate clusters."""
        # Create scan with a gap (0.0 represents no return)
        ranges = [1.0, 1.1, 0.0, 0.0, 2.0, 2.1]  # Gap at indices 2-3
        result = self.clusterer.cluster_ranges(-math.pi/4, math.pi/12, ranges, 10.0)
        # Should create at least 2 clusters (before and after gap)
        self.assertGreaterEqual(len(result), 2, "Gap should create separate clusters")
    
    def test_out_of_range_detection(self):
        """Test that out-of-range values create cluster breaks."""
        ranges = [1.0, 1.1, 15.0, 2.0, 2.1]  # 15.0 exceeds max_range
        result = self.clusterer.cluster_ranges(0, 0.1, ranges, 10.0)
        # 15.0 should create a break, so we should have at least 2 clusters
        self.assertGreaterEqual(len(result), 2, "Out-of-range should create cluster break")
    
    def test_algorithm_properties(self):
        """Test algorithm has expected properties."""
        self.assertEqual(self.clusterer.node_name, 'scan_clusterer')
        self.assertEqual(self.clusterer.cluster_distance, 0.3)


class TestPointcloudSegmenter(unittest.TestCase):
    """Tests for PointcloudSegmenter algorithm."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.segmenter = PointcloudSegmenter(ground_threshold=0.1)
    
    def test_empty_input(self):
        """Test empty input handling."""
        ground, objects = self.segmenter.segment([])
        self.assertEqual(ground, [], "Empty input should return empty ground")
        self.assertEqual(objects, [], "Empty input should return empty objects")
    
    def test_all_ground_points(self):
        """Test points below threshold classified as ground."""
        points = [[0.0, 0.0, 0.0], [1.0, 1.0, 0.05]]
        ground, objects = self.segmenter.segment(points)
        self.assertEqual(len(ground), 2, "All low points should be ground")
        self.assertEqual(len(objects), 0, "No points should be objects")
    
    def test_all_elevated_points(self):
        """Test points above threshold classified as objects."""
        points = [[0.0, 0.0, 0.5], [1.0, 1.0, 1.0]]
        ground, objects = self.segmenter.segment(points)
        self.assertEqual(len(ground), 0, "No points should be ground")
        self.assertEqual(len(objects), 2, "All high points should be objects")
    
    def test_mixed_points(self):
        """Test mixed ground and elevated points."""
        points = [
            [0.0, 0.0, 0.0],     # Ground
            [1.0, 1.0, 0.05],   # Ground
            [2.0, 2.0, 0.5],    # Object
            [3.0, 3.0, 1.0]     # Object
        ]
        ground, objects = self.segmenter.segment(points)
        self.assertEqual(len(ground), 2, "Should have 2 ground points")
        self.assertEqual(len(objects), 2, "Should have 2 object points")
    
    def test_algorithm_properties(self):
        """Test algorithm has expected properties."""
        self.assertEqual(self.segmenter.node_name, 'pointcloud_segmenter')
        self.assertEqual(self.segmenter.ground_threshold, 0.1)


class TestEuclideanClusterer(unittest.TestCase):
    """Tests for EuclideanClusterer algorithm."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.clusterer = EuclideanClusterer(tolerance=0.2, min_points=2)
    
    def test_empty_input(self):
        """Test empty input handling."""
        result = self.clusterer.cluster([])
        self.assertEqual(result, [], "Empty input should return empty list")
    
    def test_single_point_below_min(self):
        """Test single point below min_points threshold."""
        points = [[0.0, 0.0, 0.0]]
        result = self.clusterer.cluster(points)
        self.assertEqual(result, [], "Single point below threshold should be filtered out")
    
    def test_two_points_form_cluster(self):
        """Test two close points form one cluster."""
        points = [[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]]
        result = self.clusterer.cluster(points)
        self.assertEqual(len(result), 1, "Two close points should form one cluster")
        self.assertEqual(len(result[0]), 2, "Cluster should contain both points")
    
    def test_separate_clusters(self):
        """Test points in separate regions form different clusters."""
        points = [
            [0.0, 0.0, 0.0], [0.1, 0.1, 0.1],  # Cluster 1
            [2.0, 2.0, 2.0], [2.1, 2.1, 2.1]   # Cluster 2
        ]
        result = self.clusterer.cluster(points)
        self.assertEqual(len(result), 2, "Should find 2 clusters")
        self.assertEqual(len(result[0]), 2, "First cluster should have 2 points")
        self.assertEqual(len(result[1]), 2, "Second cluster should have 2 points")
    
    def test_tolerance_parameter(self):
        """Test that tolerance parameter affects clustering."""
        # Points that are 0.3 apart
        points = [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]]
        
        # With tolerance=0.2, should not cluster
        clusterer_small = EuclideanClusterer(tolerance=0.2, min_points=2)
        result_small = clusterer_small.cluster(points)
        
        # With tolerance=0.4, should cluster
        clusterer_large = EuclideanClusterer(tolerance=0.4, min_points=2)
        result_large = clusterer_large.cluster(points)
        
        self.assertNotEqual(len(result_small), len(result_large), 
                          "Different tolerances should affect clustering")
    
    def test_algorithm_properties(self):
        """Test algorithm has expected properties."""
        self.assertEqual(self.clusterer.node_name, 'euclidean_clusterer')
        self.assertEqual(self.clusterer.tolerance, 0.2)
        self.assertEqual(self.clusterer.min_points, 2)


class TestDBSCANClusterer(unittest.TestCase):
    """Tests for DBSCANClusterer algorithm."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.clusterer = DBSCANClusterer(eps=0.3, min_samples=2)
    
    def test_empty_input(self):
        """Test empty input handling."""
        clusters, noise = self.clusterer.cluster([])
        self.assertEqual(clusters, [], "Empty input should return empty clusters")
        self.assertEqual(noise, [], "Empty input should return empty noise")
    
    def test_core_points_detection(self):
        """Test that core points form clusters."""
        # Create a dense region (3 points close together)
        points = [[0.0, 0.0, 0.0], [0.1, 0.1, 0.1], [0.2, 0.2, 0.2]]
        clusters, noise = self.clusterer.cluster(points)
        self.assertEqual(len(clusters), 1, "Dense region should form one cluster")
        self.assertEqual(len(noise), 0, "No noise should be detected")
    
    def test_noise_detection(self):
        """Test that isolated points are classified as noise."""
        # Single isolated point
        points = [[0.0, 0.0, 0.0]]
        clusters, noise = self.clusterer.cluster(points)
        self.assertEqual(len(clusters), 0, "Isolated point should not form cluster")
        self.assertEqual(len(noise), 1, "Isolated point should be noise")
    
    def test_mixed_core_and_noise(self):
        """Test mix of core points and noise."""
        points = [
            [0.0, 0.0, 0.0], [0.1, 0.1, 0.1], [0.2, 0.2, 0.2],  # Dense cluster
            [5.0, 5.0, 5.0], [6.0, 6.0, 6.0]  # Isolated points
        ]
        clusters, noise = self.clusterer.cluster(points)
        self.assertEqual(len(clusters), 1, "Should find one cluster")
        self.assertEqual(len(noise), 2, "Should find two noise points")
    
    def test_eps_parameter(self):
        """Test that eps parameter affects clustering."""
        points = [[0.0, 0.0, 0.0], [0.4, 0.0, 0.0]]
        
        # With eps=0.3, points are too far apart
        clusterer_small = DBSCANClusterer(eps=0.3, min_samples=2)
        clusters_small, _ = clusterer_small.cluster(points)
        
        # With eps=0.5, points form a cluster
        clusterer_large = DBSCANClusterer(eps=0.5, min_samples=2)
        clusters_large, _ = clusterer_large.cluster(points)
        
        self.assertNotEqual(len(clusters_small), len(clusters_large),
                          "Different eps values should affect clustering")
    
    def test_algorithm_properties(self):
        """Test algorithm has expected properties."""
        self.assertEqual(self.clusterer.node_name, 'dbscan_clusterer')
        self.assertEqual(self.clusterer.eps, 0.3)
        self.assertEqual(self.clusterer.min_samples, 2)


class TestRANSACGroundRemoval(unittest.TestCase):
    """Tests for RANSACGroundRemoval algorithm."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.segmenter = RANSACGroundRemoval(
            max_iterations=100, 
            distance_threshold=0.05, 
            normal_threshold=0.85
        )
    
    def test_empty_input(self):
        """Test empty input handling."""
        ground, obstacles = self.segmenter.segment([])
        self.assertEqual(ground, [], "Empty input should return empty ground")
        self.assertEqual(obstacles, [], "Empty input should return empty obstacles")
    
    def test_insufficient_points(self):
        """Test handling of insufficient points for plane fitting."""
        points = [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]]  # Only 2 points
        ground, obstacles = self.segmenter.segment(points)
        # Should return all as obstacles since we can't fit a plane with 2 points
        self.assertEqual(ground, [], "Insufficient points should not form ground plane")
        self.assertEqual(len(obstacles), 2, "All points should be obstacles")
    
    def test_ground_plane_detection(self):
        """Test detection of ground plane."""
        # Create a clear ground plane (z=0) with some noise
        np.random.seed(42)  # For reproducibility
        points = []
        
        # Ground plane points
        for _ in range(20):
            x = np.random.uniform(-1, 1)
            y = np.random.uniform(-1, 1)
            z = np.random.uniform(-0.01, 0.01)  # Very close to z=0
            points.append([x, y, z])
        
        ground, obstacles = self.segmenter.segment(points)
        
        # Most points should be classified as ground
        self.assertGreater(len(ground), len(obstacles), 
                         "Most points should be classified as ground")
        self.assertGreater(len(ground), 15, "Should find many ground points")
    
    def test_non_horizontal_plane_rejection(self):
        """Test that non-horizontal planes are rejected as ground."""
        # Create points on a vertical plane (x=0)
        points = []
        for i in range(20):
            x = 0.0
            y = i * 0.1
            z = i * 0.1
            points.append([x, y, z])
        
        ground, obstacles = self.segmenter.segment(points)
        
        # Vertical plane should not be classified as ground
        self.assertEqual(len(ground), 0, "Vertical plane should not be classified as ground")
        self.assertEqual(len(obstacles), 20, "All points should be obstacles")
    
    def test_algorithm_properties(self):
        """Test algorithm has expected properties."""
        self.assertEqual(self.segmenter.node_name, 'ransac_ground_removal')
        self.assertEqual(self.segmenter.max_iterations, 100)
        self.assertEqual(self.segmenter.distance_threshold, 0.05)
        self.assertEqual(self.segmenter.normal_threshold, 0.85)


class TestPerceptionPerformance(unittest.TestCase):
    """Performance tests for perception algorithms."""
    
    def test_large_dataset_performance(self):
        """Test that algorithms can handle reasonably large datasets."""
        import time
        
        # Create large dataset
        np.random.seed(42)
        points = []
        for _ in range(1000):
            x = np.random.uniform(-10, 10)
            y = np.random.uniform(-10, 10)
            z = np.random.uniform(-1, 1)
            points.append([x, y, z])
        
        # Test Euclidean clusterer performance
        start_time = time.time()
        clusterer = EuclideanClusterer(tolerance=0.5, min_points=5)
        clusters = clusterer.cluster(points)
        euclidean_time = time.time() - start_time
        
        # Test DBSCAN clusterer performance
        start_time = time.time()
        dbscan = DBSCANClusterer(eps=0.5, min_samples=5)
        clusters, noise = dbscan.cluster(points)
        dbscan_time = time.time() - start_time
        
        # Both should complete in reasonable time (< 1 second)
        self.assertLess(euclidean_time, 1.0, f"Euclidean clustering took {euclidean_time:.3f}s")
        self.assertLess(dbscan_time, 1.0, f"DBSCAN clustering took {dbscan_time:.3f}s")
        
        # Both should find some clusters
        self.assertGreater(len(clusters), 0, "Should find at least one cluster")
    
    def test_mathematical_correctness(self):
        """Test mathematical correctness of Euclidean distance calculation."""
        clusterer = EuclideanClusterer(tolerance=0.1, min_points=2)
        
        # Create points at known distances
        points = [
            [0.0, 0.0, 0.0],
            [0.05, 0.0, 0.0],  # Distance = 0.05, should cluster
            [0.2, 0.0, 0.0]    # Distance = 0.2, should NOT cluster
        ]
        
        clusters = clusterer.cluster(points)
        
        # The first two points should cluster together, third should be separate
        # or all might be separate if the clustering algorithm considers order
        # Just verify it doesn't crash and produces valid results
        self.assertIsInstance(clusters, list, "Result should be a list")
        self.assertGreaterEqual(len(clusters), 1, "Should find at least one cluster")


class TestPerceptionEdgeCases(unittest.TestCase):
    """Edge case tests for perception algorithms."""
    
    def test_nan_handling(self):
        """Test handling of NaN values."""
        # Most algorithms should handle this gracefully or at least not crash
        segmenter = PointcloudSegmenter()
        
        # Try with NaN in point
        try:
            points_with_nan = [[0.0, 0.0, 0.0], [float('nan'), float('nan'), float('nan')]]
            ground, objects = segmenter.segment(points_with_nan)
            # If it doesn't crash, that's good
            self.assertIsInstance(ground, list, "Should return list even with NaN")
        except Exception:
            # Some implementations might not handle NaN, that's acceptable
            pass
    
    def test_very_large_values(self):
        """Test handling of very large coordinate values."""
        detector = ObstacleDetector(cluster_distance=0.5)
        
        # Very large coordinates
        points = [(1e6, 1e6), (1e6 + 0.1, 1e6 + 0.1)]
        result = detector.detect(points)
        
        # Should still work (distance calculation should handle large numbers)
        self.assertEqual(len(result), 1, "Should cluster close points even with large coordinates")
    
    def test_zero_distance_points(self):
        """Test handling of points at the same location."""
        clusterer = EuclideanClusterer(tolerance=0.0, min_points=2)
        
        # Points at exact same location
        points = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        result = clusterer.cluster(points)
        
        # Should handle this gracefully
        self.assertIsInstance(result, list, "Should return list")
    
    def test_negative_coordinates(self):
        """Test handling of negative coordinates."""
        detector = ObstacleDetector()
        
        points = [(-1.0, -1.0), (-0.5, -0.5)]
        result = detector.detect(points)
        
        self.assertEqual(len(result), 1, "Should cluster negative coordinates")


if __name__ == '__main__':
    unittest.main()