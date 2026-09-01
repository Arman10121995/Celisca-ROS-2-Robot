# Perception Comparison Tutorial

Compare obstacle detection vs. scan clustering on the same input data.

## Run

```python
from bumperbot_algorithms.perception import ObstacleDetector, ScanClusterer

# Detect obstacle clusters from 2D points
detector = ObstacleDetector()
points = [(0.0, 0.0), (0.1, 0.1), (5.0, 5.0), (5.1, 5.0)]
clusters = detector.detect(points)
print(f"Obstacle clusters: {len(clusters)}")

# Cluster laser scan ranges
clusterer = ScanClusterer()
angle_min, angle_increment = -1.5708, 0.01745
ranges = [10.0] * 60 + [0.5] * 30 + [10.0] * 30
scan_clusters = clusterer.cluster_ranges(angle_min, angle_increment, ranges, 10.0)
print(f"Scan clusters: {len(scan_clusters)}")
```

## Expected Output

```
Obstacle clusters: 2
Scan clusters: 2
```
