# Planning: inspect two numerical prototypes

This example calls `RRTPlanner` and the class named `VoronoiPlanner` directly.
It does not launch Nav2 or move a robot. The current executable entry points
create idle ROS nodes rather than consume maps/goals and publish computed paths.

Important implementation limits:

- RRT checks sampled vertices, not entire edges or a robot footprint, and stops
  within a goal tolerance rather than necessarily connecting to the goal.
- `VoronoiPlanner` is a greedy local clearance-score walker. It does not build a
  Voronoi diagram/roadmap or perform a graph search toward the goal. It can cycle;
  its fallback step is not collision checked.

They are not two qualified, interchangeable global planners.

## Run

Follow the workspace-root setup in the [tutorial index](index.md). This test
uses a point robot, a central square obstacle and a fixed RRT seed. The separate
edge sampler checks the returned polyline more carefully than the RRT code,
but finite sampling is still not a continuous collision or footprint guarantee.

```bash
python3 - <<'PY'
import math
from robot_lab_algorithms.global_planning import RRTPlanner, VoronoiPlanner

bounds = (-5.0, -5.0, 5.0, 5.0)
start, goal = (-4.0, -4.0), (4.0, 4.0)

def is_free(x, y):
    return (-5.0 <= x <= 5.0 and -5.0 <= y <= 5.0
            and (abs(x) > 1.0 or abs(y) > 1.0))

def clearance_score(x, y):
    if not is_free(x, y):
        return 0.0
    obstacle_distance = math.hypot(max(abs(x) - 1.0, 0.0),
                                   max(abs(y) - 1.0, 0.0))
    return min(obstacle_distance, x + 5.0, 5.0 - x, y + 5.0, 5.0 - y)

def sampled_edges_free(path, spacing=0.02):
    if not path:
        return False
    for a, b in zip(path, path[1:]):
        n = max(1, math.ceil(math.dist(a, b) / spacing))
        for j in range(n + 1):
            t = j / n
            if not is_free(a[0] + t * (b[0] - a[0]),
                           a[1] + t * (b[1] - a[1])):
                return False
    return True

rrt_path = RRTPlanner(step=1.0, max_iter=2000, goal_tol=0.5).plan(
    start, goal, is_free, bounds, seed=42
)
ridge_path = VoronoiPlanner().plan(
    start, goal, clearance_score, step=0.5, max_steps=100
)
for name, path in [('RRT', rrt_path), ('Greedy ridge prototype', ridge_path)]:
    near_goal = bool(path) and math.dist(path[-1], goal) <= 0.5
    print(f'{name}: waypoints={len(path)}, near_goal={near_goal}, '
          f'sampled_edges_free={sampled_edges_free(path)}')
PY
```

Observed output for the audited implementation; a different implementation or
random-number sequence can change waypoint counts:

```text
RRT: waypoints=15, near_goal=True, sampled_edges_free=True
Greedy ridge prototype: waypoints=101, near_goal=False, sampled_edges_free=True
```

The second result is an expected demonstration of a limitation, not a successful
plan. More waypoints do not imply a longer route or a better algorithm. No
planning-time or success-rate comparison has been measured here.

## Path to a controlled comparison

1. Add complete edge/footprint collision checks, invalid-start/goal handling,
   explicit failure/timeout results, bounds checks and goal acceptance tests.
2. Replace or accurately rename the greedy prototype; if retaining the Voronoi
   name, implement and validate the actual roadmap construction and search.
3. Use common map resolution, inflation, start/goal pairs and robot constraints;
   keep static 2D global planning distinct from dynamic local control and 3D
   flight planning.
4. Run multiple fixed seeds and compare valid-path success rate, path length,
   minimum footprint clearance, planning latency and memory. Validate metrics
   independently of the planner, then test the real ROS/Nav2 adapter.

The [roadmap](../../ROADMAP.md) and [agent handoff](../AGENT_HANDOFF.md) define
the wider implementation work; this example alone is not a benchmark.
