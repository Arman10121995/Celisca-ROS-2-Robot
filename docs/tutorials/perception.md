# Perception Pipelines Tutorial

Documentation reviewed October 7, 2026 against runtime checkpoint `091d388`.
Read [current status](../status/CURRENT_STATUS.md) for available workflows and
remaining qualification; evidence below retains its named source stages.

## Overview

These are perception kernel and adapter notes for scan detection, clustering, ground removal and segmentation. R7.2 real input/output, robot missions and fair multi-method comparison remain partial.

## R7.2 Target Methods

| Method | Description | Implementation | Status |
|--------|-------------|----------------|---------|
| Scan Obstacle Clustering | Segments occupancy grid/scan ranges into obstacle clusters | `obstacle_detector` | ✅ Integrated |
| Euclidean Clustering | Distance-based clustering for 3D point clouds | `euclidean_clusterer` | ✅ Integrated |
| DBSCAN Clustering | Density-based clustering with noise removal | `dbscan_clusterer` | ✅ Integrated |
| RANSAC Ground Removal | Plane fitting for ground/obstacle segmentation | `ransac_ground_removal` | ✅ Integrated |
| Voxel Occupancy | Grid-based occupancy segmentation | `pointcloud_segmenter` | ✅ Integrated |

## Implementation Details

### 1. Scan Obstacle Clustering (`obstacle_detector`)

- **Algorithm**: Proximity-based clustering of 2D points
- **Input**: `/scan` (LaserScan)
- **Output**: `/perception/obstacles` (MarkerArray)
- **Parameters**: `cluster_distance` (default: 0.5m)
- **Mathematical Basis**: Euclidean distance grouping

**Equation**:
```
clusters = group_points_by_distance(points, threshold=cluster_distance)
```

### 2. Euclidean Point-Cloud Clustering (`euclidean_clusterer`)

- **Algorithm**: Euclidean distance-based clustering in 3D space
- **Input**: PointCloud2 on `/oakd/points`
- **Output**: `/perception/euclidean_clusters` (MarkerArray)
- **Parameters**: `tolerance` (default: 0.1m), `min_points` (default: 5)
- **Mathematical Basis**: ||p_i - p_j||_2 ≤ tolerance

**Equation**:
```
Cluster(p) = {q ∈ P | ||p - q||_2 ≤ tolerance}
```

### 3. DBSCAN Clustering (`dbscan_clusterer`)

- **Algorithm**: Density-Based Spatial Clustering of Applications with Noise
- **Input**: PointCloud2 on `/oakd/points`
- **Output**: `/perception/dbscan_clusters` (MarkerArray)
- **Parameters**: `eps` (default: 0.1m), `min_samples` (default: 5)
- **Mathematical Basis**: Core point expansion with ε-neighborhoods

**Algorithm**:
```
1. Find all ε-neighbors for each point
2. Classify points as core, border, or noise
3. Expand clusters from core points
4. Identify noise points not reachable from core points
```

### 4. RANSAC Ground Removal (`ransac_ground_removal`)

- **Algorithm**: RANdom SAmple Consensus plane fitting
- **Input**: PointCloud2 on `/oakd/points`
- **Output**: `/perception/ransac_ground` and `/perception/ransac_obstacles` (PointCloud2)
- **Parameters**: `max_iterations` (default: 100), `distance_threshold` (default: 0.05m), `normal_threshold` (default: 0.85)
- **Mathematical Basis**: Iterative plane fitting with inlier counting

**Algorithm**:
```
For max_iterations:
    1. Randomly select 3 points to define plane
    2. Calculate plane normal n and distance d
    3. Count inliers within distance_threshold
    4. Keep best plane (most inliers)
5. Check if best plane is horizontal (n_z > normal_threshold)
6. Segment points based on best plane
```

### 5. Voxel Occupancy Pipeline (`pointcloud_segmenter`)

- **Algorithm**: Height-threshold segmentation
- **Input**: PointCloud2 on `/oakd/points`
- **Output**: `/perception/ground` and `/perception/obstacle_cloud` (PointCloud2)
- **Parameters**: `ground_threshold` (default: 0.1m)
- **Mathematical Basis**: Simple height-based classification

**Equation**:
```
ground = {p ∈ P | p_z ≤ ground_threshold}
obstacles = {p ∈ P | p_z > ground_threshold}
```

## Input Strata and Fair Comparison

### Strata Definition

| Stratum | Input Type | Typical Sensors | Comparison Basis |
|---------|------------|----------------|------------------|
| Scan-based | 2D LaserScan | LIDAR | Angular resolution, range limits |
| Point Cloud | 3D PointCloud2 | RGB-D, Stereo | Point density, organized vs unorganized |
| RGB-D Vision | Image + Depth | RGB-D Cameras | Resolution, frame rate |

### Comparison Protocol

1. **Same Sensor Configuration**: Compare algorithms using identical sensor inputs
2. **Common Evaluation Metrics**:
   - Precision/Recall for detection tasks
   - Intersection-over-Union (IoU) for segmentation
   - Latency and throughput measurements
   - Occlusion robustness

3. **Benchmark Scenarios**:
   - Empty arena (baseline)
   - Cluttered environment (multiple obstacles)
   - Dynamic obstacles (moving targets)
   - Occluded scenarios (partial visibility)

## Usage Examples

### Running Individual Pipelines

```bash
# Run obstacle detector
ros2 run robot_lab_algorithms obstacle_detector

# Run Euclidean clustering
ros2 run robot_lab_algorithms euclidean_clusterer

# Run DBSCAN clustering
ros2 run robot_lab_algorithms dbscan_clusterer

# Run RANSAC ground removal
ros2 run robot_lab_algorithms ransac_ground_removal

# Run point cloud segmenter
ros2 run robot_lab_algorithms pointcloud_segmenter
```

### Configuration Files

All pipelines support ROS 2 parameters for tuning:

```yaml
# Example configuration for euclidean_clusterer
euclidean_clusterer:
  ros__parameters:
    points_topic: "/oakd/points"
    tolerance: 0.15
    min_points: 10
    markers_topic: "/perception/euclidean_clusters"
```

## Illustrative performance characteristics

The qualitative timing/memory labels below are teaching estimates, not
measured performance on a named host/input/robot. Actual comparisons remain R7/R9.2.

| Algorithm | Complexity | Memory Usage | Real-time Performance | Best Use Case |
|-----------|------------|---------------|----------------------|---------------|
| Obstacle Detector | O(n²) | Low | Excellent | Simple 2D environments |
| Euclidean Clusterer | O(n²) | Medium | Good | Structured 3D scenes |
| DBSCAN | O(n²) | High | Moderate | Dense point clouds |
| RANSAC Ground Removal | O(n·k) | Medium | Good | Flat ground assumption |
| Voxel Segmenter | O(n) | Low | Excellent | Simple height separation |

## Limitations and Future Work

- **Computational Efficiency**: Some algorithms could benefit from spatial indexing (KD-trees, Octrees)
- **Parameter Tuning**: Default parameters may need adjustment for specific environments
- **3D Integration**: Full 3D perception pipelines would extend beyond current scope
- **Multi-Sensor Fusion**: Integration of multiple sensor modalities is not yet implemented
- **Machine Learning**: Learning-based approaches could complement traditional methods

## References

- [DBSCAN Original Paper](https://www.aaai.org/Papers/KDD/1996/KDD96-037.pdf)
- [RANSAC Original Paper](https://ieeexplore.ieee.org/document/4767408)
- [PCL Documentation](https://pointclouds.org/documentation/)