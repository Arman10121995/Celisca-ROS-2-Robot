# Planning Comparison Tutorial

Compare RRT vs. Voronoi global planning on the same occupancy grid.

## Run

```python
from robot_lab_algorithms.global_planning import RRTPlanner, VoronoiPlanner

# Define environment: central obstacle
def is_free(x, y):
    return abs(x) > 1.0 or abs(y) > 1.0

bounds = (-5.0, -5.0, 5.0, 5.0)
start, goal = (-4.0, -4.0), (4.0, 4.0)

# RRT planner
rrt = RRTPlanner(step=1.0, max_iter=2000, goal_tol=0.5)
rrt_path = rrt.plan(start, goal, is_free, bounds, seed=42)
print(f"RRT path: {len(rrt_path)} waypoints")

# Voronoi-like ridge follower
def cost_at(x, y):
    # Higher cost further from obstacles
    return min(abs(x), abs(y), 1.0) + 0.1

voronoi = VoronoiPlanner()
voronoi_path = voronoi.plan(start, goal, cost_at, step=0.5, max_steps=100)
print(f"Voronoi path: {len(voronoi_path)} waypoints")
```

## Expected Output

```
RRT path: 12 waypoints
Voronoi path: 16 waypoints
```
