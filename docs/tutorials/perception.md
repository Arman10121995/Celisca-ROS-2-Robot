# Perception: numerical clustering example

This example calls the existing `ObstacleDetector` and `ScanClusterer` Python
methods. Neither call starts a ROS node or processes a live sensor stream.
The current perception executable wrappers are not working sensor-to-output
adapters; these numerical results must not be used to qualify those wrappers.

`ObstacleDetector` groups points by distance-connected components.
`ScanClusterer` groups consecutive valid scan returns, breaking groups at an
invalid return or a large point-to-point gap. They have different semantics,
even when their output counts happen to match.

## Run

First follow the workspace-root setup in the [tutorial index](index.md).
The two methods below see equivalent geometry: two groups of three returns.
No optional numerical or ROS dependency is needed for these method calls.

```bash
python3 - <<'PY'
import math
from robot_lab_algorithms.perception import ObstacleDetector, ScanClusterer

angle_min, angle_increment, max_range = -1.0, 0.1, 10.0
ranges = [1.0, 1.0, 1.0] + [max_range] * 7 + [1.0, 1.0, 1.0]
points = [
    (r * math.cos(angle_min + i * angle_increment),
     r * math.sin(angle_min + i * angle_increment))
    for i, r in enumerate(ranges)
    if math.isfinite(r) and 0.0 < r < max_range
]

clusters = ObstacleDetector(cluster_distance=0.3).detect(points)
scan_clusters = ScanClusterer(cluster_distance=0.3).cluster_ranges(
    angle_min, angle_increment, ranges, max_range
)
print('Point-cluster sizes:', sorted(map(len, clusters)))
print('Scan-cluster sizes:', sorted(map(len, scan_clusters)))
PY
```

Expected output, checked against the current source:

```text
Point-cluster sizes: [3, 3]
Scan-cluster sizes: [3, 3]
```

This establishes behavior on one finite synthetic input, not detection quality,
robustness to NaN/Inf, runtime scalability, or suitability for navigation.
The methods do not provide learned classification or object tracking.

## Path to a controlled comparison

1. Define whether the task is scan segmentation, obstacle-instance detection,
   ground segmentation, or semantic perception; compare methods solving the
   same task rather than treating converters as competing detectors.
2. Supply labeled scan/point-cloud datasets with frames, units, timestamps,
   minimum/maximum ranges and validity rules. Preserve the original scans and
   derive common point inputs reproducibly.
3. Specify instance matching and evaluate precision/recall, segmentation
   agreement, latency and memory. Include sparse returns, occlusion, touching
   objects, outliers and non-finite inputs with recorded noise seeds.
4. Implement real ROS subscriptions/publications, then test the full
   input-to-output contract before running simulator missions.

Continue via the [roadmap](../../ROADMAP.md) and
[agent handoff](../AGENT_HANDOFF.md). No comparative benchmark is completed by
this tutorial.
