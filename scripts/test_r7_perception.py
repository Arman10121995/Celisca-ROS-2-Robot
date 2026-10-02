#!/usr/bin/env python3
"""
R7 Perception Algorithm Unit Tests

Comprehensive numerical tests for all perception algorithms to verify
they work correctly, produce expected outputs, and handle edge cases.
"""

import unittest
import math
import numpy as np
import sys
import os

# Add the src directory to Python path
workspace_path = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(workspace_path, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

# Test fixtures
class TestFixtures:
    """Common test fixtures for perception algorithms."""
    
    @staticmethod
    def create_test_scan(ranges=None, angles=None, num_points=100):
        """Create a test LaserScan-like point cloud."""
        if ranges is None:
            # Create a semi-circle of points
            ranges = [2.0 + 0.1 * math.sin(i * 2 * math.pi / num_points) for i in range(num_points)]
        if angles is None:
            angles = [i * 2 * math.pi / num_points - math.pi for i in range(num_points)]
        return ranges, angles
    
    @staticmethod
    def create_test_pointcloud(num_points=50):
        """Create a 3D point cloud for testing."""
        points = []
        for i in range(num_points):
            theta = i * 2 * math.pi / num_points
            r = 1.0 + 0.2 * math.sin(3 * theta)  # Some variation
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            z = 0.1 * math.sin(2 * theta)  # Some height variation
            points.append([x, y, z])
        return points
    
    @staticmethod
    def create_clustered_pointcloud(num_clusters=3, points_per_cluster=20):
        """Create a point cloud with distinct clusters."""
        points = []
        for cluster_idx in range(num_clusters):
            center_x = cluster_idx * 2.0 - 2.0
            center_y = cluster_idx * 1.0 - 1.0
            center_z = cluster_idx * 0.5
            
            for j in range(points_per_cluster):
                # Add some noise around the center
                x = center_x + 0.1 * (np.random.random() - 0.5)
                y = center_y + 0.1 * (np.random.random() - 0.5)
                z = center_z + 0.05 * (np.random.random() - 0.5)
                points.append([x, y, z])
        return points


class TestPerceptionAlgorithms(unittest.TestCase):
    """Test all perception algorithm implementations."""
    
    def setUp(self):
        """Set up test environment."""
        # Import modules with error handling
        try:
            from robot_lab_algorithms.robot_lab_algorithms.perception import (
                ObstacleDetector, ScanClusterer, PointcloudSegmenter,
                EuclideanClusterer, DBSCANClusterer, RANSACGroundRemoval
            )
            self.ObstacleDetector = ObstacleDetector
            self.ScanClusterer = ScanClusterer
            self.PointcloudSegmenter = PointcloudSegmenter
            self.EuclideanClusterer = EuclideanClusterer
            self.DBSCANClusterer = DBSCANClusterer
            self.RANSACGroundRemoval = RANSACGroundRemoval
        except ImportError as e:
            self.skipTest(f"Failed to import perception algorithms: {e}")
    
    def test_obstacle_detector_instantiation(self):
        """Test that ObstacleDetector can be instantiated."""
        detector = self.ObstacleDetector()
        self.assertIsNotNone(detector)
        self.assertEqual(detector.node_name, 'obstacle_detector')
    
    def test_obstacle_detector_functionality(self):
        """Test that ObstacleDetector detects clusters correctly."""
        detector = self.ObstacleDetector(cluster_distance=0.5)
        
        # Create test points: two obvious clusters
        points = [
            (0.0, 0.0), (0.1, 0.1), (0.2, 0.2),  # Cluster 1
            (2.0, 2.0), (2.1, 2.1), (2.2, 2.2)   # Cluster 2
        ]
        
        clusters = detector.detect(points)
        self.assertEqual(len(clusters), 2, "Should detect 2 clusters")
        self.assertEqual(len(clusters[0]), 3, "First cluster should have 3 points")
        self.assertEqual(len(clusters[1]), 3, "Second cluster should have 3 points")
    
    def test_scan_clusterer_instantiation(self):
        """Test that ScanClusterer can be instantiated."""
        clusterer = self.ScanClusterer()
        self.assertIsNotNone(clusterer)
    
    def test_scan_clusterer_functionality(self):
        """Test that ScanClusterer clusters scan ranges correctly."""
        clusterer = self.ScanClusterer(cluster_distance=0.3)
        
        # Create scan with two clusters separated by a gap
        ranges = [1.0, 1.1, 1.2, 0.0, 0.0, 0.0, 2.0, 2.1, 2.2]  # Gap at indices 3-5
        angle_min = -math.pi / 4
        angle_increment = math.pi / 16
        max_range = 10.0
        
        clusters = clusterer.cluster_ranges(angle_min, angle_increment, ranges, max_range)
        self.assertEqual(len(clusters), 2, "Should detect 2 clusters")
    
    def test_pointcloud_segmenter_instantiation(self):
        """Test that PointcloudSegmenter can be instantiated."""
        segmenter = self.PointcloudSegmenter()
        self.assertIsNotNone(segmenter)
    
    def test_pointcloud_segmenter_functionality(self):
        """Test that PointcloudSegmenter separates ground and obstacles."""
        segmenter = self.PointcloudSegmenter(ground_threshold=0.1)
        
        # Create point cloud with ground and elevated points
        points = [
            [0.0, 0.0, 0.0],   # Ground
            [0.1, 0.1, 0.05], # Ground
            [1.0, 1.0, 1.0],   # Elevated
            [1.1, 1.1, 1.2]    # Elevated
        ]
        
        ground, objects = segmenter.segment(points)
        self.assertEqual(len(ground), 2, "Should have 2 ground points")
        self.assertEqual(len(objects), 2, "Should have 2 object points")
    
    def test_euclidean_clusterer_instantiation(self):
        """Test that EuclideanClusterer can be instantiated."""
        clusterer = self.EuclideanClusterer()
        self.assertIsNotNone(clusterer)
    
    def test_euclidean_clusterer_functionality(self):
        """Test that EuclideanClusterer finds clusters in 3D space."""
        clusterer = self.EuclideanClusterer(tolerance=0.2, min_points=2)
        
        # Create test points with two clusters
        points = [
            [0.0, 0.0, 0.0], [0.1, 0.1, 0.1],  # Cluster 1
            [2.0, 2.0, 2.0], [2.1, 2.1, 2.1]   # Cluster 2
        ]
        
        clusters = clusterer.cluster(points)
        self.assertEqual(len(clusters), 2, "Should find 2 clusters")
    
    def test_dbscan_clusterer_instantiation(self):
        """Test that DBSCANClusterer can be instantiated."""
        clusterer = self.DBSCANClusterer()
        self.assertIsNotNone(clusterer)
    
    def test_dbscan_clusterer_functionality(self):
        """Test that DBSCANClusterer correctly identifies core points, border points, and noise."""
        clusterer = self.DBSCANClusterer(eps=0.3, min_samples=2)
        
        # Create a dense cluster and some noise points
        points = [
            [0.0, 0.0, 0.0], [0.1, 0.1, 0.1], [0.2, 0.2, 0.2],  # Dense cluster
            [5.0, 5.0, 5.0]  # Isolated point (noise)
        ]
        
        clusters, noise = clusterer.cluster(points)
        self.assertTrue(len(clusters) >= 1, "Should find at least 1 cluster")
        self.assertEqual(len(noise), 1, "Should identify 1 noise point")
    
    def test_ransac_ground_removal_instantiation(self):
        """Test that RANSACGroundRemoval can be instantiated."""
        segmenter = self.RANSACGroundRemoval()
        self.assertIsNotNone(segmenter)
    
    def test_ransac_ground_removal_functionality(self):
        """Test that RANSACGroundRemoval separates ground plane from obstacles."""
        segmenter = self.RANSACGroundRemoval(
            max_iterations=100, 
            distance_threshold=0.05, 
            normal_threshold=0.85
        )
        
        # Create point cloud with obvious ground plane
        points = []
        
        # Ground plane points (z ≈ 0)
        for i in range(10):
            x = (i - 5) * 0.1
            y = (i % 3 - 1) * 0.1
            z = 0.0 + 0.01 * np.random.random()  # Small variations
            points.append([x, y, z])
        
        # Some elevated points
        for i in range(5):
            x = (i - 2) * 0.1
            y = (i % 2 - 0.5) * 0.1  
            z = 0.5 + 0.01 * np.random.random()  # Elevated
            points.append([x, y, z])
        
        ground, obstacles = segmenter.segment(points)
        
        # Should find most points as ground and few as obstacles
        self.assertGreater(len(ground), 0, "Should find some ground points")
        self.assertGreater(len(obstacles), 0, "Should find some obstacle points")
    
    def test_empty_input_handling(self):
        """Test that all algorithms handle empty input gracefully."""
        detector = self.ObstacleDetector()
        clusterer = self.ScanClusterer()
        segmenter = self.PointcloudSegmenter()
        euclidean = self.EuclideanClusterer()
        dbscan = self.DBSCANClusterer()
        ransac = self.RANSACGroundRemoval()
        
        # Test empty inputs
        self.assertEqual(detector.detect([]), [], "Empty input should return empty list")
        self.assertEqual(clusterer.cluster_ranges(0, 0.1, [], 10), [], "Empty scan should return empty list")
        self.assertEqual(segmenter.segment([]), ([], []), "Empty point cloud should return empty ground and obstacles")
        self.assertEqual(euclidean.cluster([]), [], "Empty input should return empty list")
        
        # DBSCAN returns (clusters, noise) tuple
        clusters, noise = dbscan.cluster([])
        self.assertEqual(clusters, [], "Empty input should return empty clusters")
        self.assertEqual(noise, [], "Empty input should return empty noise")
        
        # RANSAC
        ground, obstacles = ransac.segment([])
        self.assertEqual(ground, [], "Empty input should return empty ground")
        self.assertEqual(obstacles, [], "Empty input should return empty obstacles")


class TestPerceptionEdgeCases(unittest.TestCase):
    """Test edge cases and error conditions for perception algorithms."""
    
    def setUp(self):
        """Set up test environment."""
        try:
            from robot_lab_algorithms.robot_lab_algorithms.perception import (
                ObstacleDetector, ScanClusterer, PointcloudSegmenter,
                EuclideanClusterer, DBSCANClusterer, RANSACGroundRemoval
            )
            self.ObstacleDetector = ObstacleDetector
            self.ScanClusterer = ScanClusterer
            self.PointcloudSegmenter = PointcloudSegmenter
            self.EuclideanClusterer = EuclideanClusterer
            self.DBSCANClusterer = DBSCANClusterer
            self.RANSACGroundRemoval = RANSACGroundRemoval
        except ImportError as e:
            self.skipTest(f"Failed to import perception algorithms: {e}")
    
    def test_single_point_handling(self):
        """Test algorithms with single point inputs."""
        detector = self.ObstacleDetector()
        
        # Single point should form one cluster
        clusters = detector.detect([(0.0, 0.0)])
        self.assertEqual(len(clusters), 1, "Single point should form one cluster")
        
        segmenter = self.PointcloudSegmenter()
        ground, objects = segmenter.segment([[0.0, 0.0, 0.0]])
        self.assertEqual(len(ground), 1, "Single ground point should be classified as ground")
    
    def test_noise_handling(self):
        """Test algorithms with noisy input data."""
        clusterer = self.EuclideanClusterer(tolerance=0.1, min_points=3)
        
        # Create noisy data around a single point
        points = []
        for _ in range(10):
            x = 0.0 + 0.05 * np.random.random()
            y = 0.0 + 0.05 * np.random.random()
            z = 0.0 + 0.05 * np.random.random()
            points.append([x, y, z])
        
        clusters = clusterer.cluster(points)
        self.assertEqual(len(clusters), 1, "Noisy points around same location should form one cluster")
    
    def test_different_densities(self):
        """Test DBSCAN with different density clusters."""
        clusterer = self.DBSCANClusterer(eps=0.2, min_samples=3)
        
        # Create dense and sparse clusters
        points = []
        
        # Dense cluster (6 points close together)
        for _ in range(6):
            points.append([0.0 + 0.05 * np.random.random(), 
                          0.0 + 0.05 * np.random.random(), 
                          0.0 + 0.05 * np.random.random()])
        
        # Sparse cluster (2 points close together)
        for _ in range(2):
            points.append([2.0 + 0.01 * np.random.random(),
                          2.0 + 0.01 * np.random.random(),
                          2.0 + 0.01 * np.random.random()])
        
        clusters, noise = clusterer.cluster(points)
        # Dense cluster should be found, sparse might be noise depending on min_samples
        self.assertTrue(len(clusters) >= 1, "Should find at least the dense cluster")


class TestPerceptionPerformance(unittest.TestCase):
    """Test performance characteristics of perception algorithms."""
    
    def setUp(self):
        """Set up test environment."""
        try:
            from robot_lab_algorithms.robot_lab_algorithms.perception import (
                EuclideanClusterer, DBSCANClusterer
            )
            self.EuclideanClusterer = EuclideanClusterer
            self.DBSCANClusterer = DBSCANClusterer
        except ImportError as e:
            self.skipTest(f"Failed to import perception algorithms: {e}")
    
    def test_clusterer_performance_with_large_data(self):
        """Test that clusterers can handle reasonably large datasets."""
        import time
        
        # Create large dataset
        np.random.seed(42)  # Reproducible
        points = []
        for _ in range(500):  # 500 points
            x = 5.0 * np.random.random()
            y = 5.0 * np.random.random()
            z = 0.5 * np.random.random()
            points.append([x, y, z])
        
        # Test Euclidean clusterer
        start_time = time.time()
        clusterer = self.EuclideanClusterer(tolerance=0.5, min_points=2)
        clusters = clusterer.cluster(points)
        euclidean_time = time.time() - start_time
        
        # Test DBSCAN clusterer
        start_time = time.time()
        dbscan = self.DBSCANClusterer(eps=0.5, min_samples=2)
        clusters, noise = dbscan.cluster(points)
        dbscan_time = time.time() - start_time
        
        # Should complete in reasonable time
        self.assertLess(euclidean_time, 1.0, "Euclidean clustering should be fast")
        self.assertLess(dbscan_time, 1.0, "DBSCAN clustering should be fast")
        
        print(f"Performance: Euclidean={euclidean_time:.3f}s, DBSCAN={dbscan_time:.3f}s")


if __name__ == '__main__':
    unittest.main()