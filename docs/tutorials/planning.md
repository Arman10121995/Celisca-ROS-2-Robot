# Planning Algorithms

Documentation reviewed October 7, 2026 against runtime checkpoint `091d388`.
Read [current status](../status/CURRENT_STATUS.md) for available workflows and
remaining qualification; evidence below retains its named source stages.

These are educational planning notes and source API examples. Nav2 integration is declared per algorithm in the dispatch configuration; the listed numerical classes do not all become interchangeable ROS planners. Fair multi-method comparison remains R7/R9.2.

## Overview

Planning algorithms generate collision-free paths from start to goal positions. The implemented algorithms span graph-based planners (Dijkstra, A*, PRM), sampling-based planners (RRT, RRT*), and geometric planners (Voronoi).

## Global Planning Algorithms

### 1. Dijkstra's Algorithm

**Purpose**: Find shortest path in a weighted graph with non-negative edge weights

**Mathematical Model**:
- **Graph**: G = (V, E) where V are vertices and E are edges with weights w
- **Cost Function**: g(v) = minimum cost from start to vertex v
- **Algorithm**: Greedy selection of vertex with minimum g(v)

**Usage**:
```python
from robot_lab_algorithms.global_planning import DijkstraPlanner

# Create grid map (1=obstacle, 0=free)
map_data = [[0, 0, 0, 1, 0],
            [0, 1, 0, 1, 0], 
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0]]

# Initialize planner
planner = DijkstraPlanner(map_data, resolution=1.0)

# Set start and goal (x, y)
start = (0, 0)
goal = (4, 3)

# Plan path
path, cost = planner.plan(start, goal)

print(f"Path cost: {cost}")
print(f"Path: {path}")
```

**Applications**: Grid-based path planning, navigation in structured environments

**Advantages**: Complete (finds optimal path if one exists), simple to implement

**Limitations**: Computationally expensive for high-resolution grids, only works with discrete grids

---

### 2. A* Algorithm

**Purpose**: Find optimal path using heuristic to guide search

**Mathematical Model**:
- **Cost-to-Come**: g(v) = actual cost from start to vertex v
- **Heuristic**: h(v) = estimated cost from vertex v to goal (must be admissible)
- **Total Cost**: f(v) = g(v) + h(v)
- **Algorithm**: Expand vertex with minimum f(v)

**Usage**:
```python
from robot_lab_algorithms.global_planning import AStarPlanner

# Create grid map
map_data = [[0, 0, 0, 1, 0],
            [0, 1, 0, 1, 0], 
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0]]

# Initialize with Euclidean heuristic
planner = AStarPlanner(map_data, resolution=1.0)

# Set start and goal
start = (0, 0)
goal = (4, 3)

# Plan path
path, cost = planner.plan(start, goal)

print(f"Path cost: {cost}")
print(f"Path length: {len(path)}")
```

**Applications**: Path planning with heuristics, faster than Dijkstra for large maps

**Advantages**: Optimally efficient (expands fewer nodes than Dijkstra), guaranteed optimal path

**Limitations**: Requires admissible heuristic, performance depends on heuristic quality

---

### 3. Probabilistic Roadmap (PRM)

**Purpose**: Path planning in continuous configuration spaces using sampling

**Mathematical Model**:
- **Sampling**: Randomly sample configurations in free space
- **Connectivity**: Connect samples using local planner
- **Graph Construction**: Create graph where nodes are samples and edges are collision-free paths
- **Query**: Use A* on the pre-computed graph

**Usage**:
```python
from robot_lab_algorithms.global_planning import PRMPlanner
import numpy as np

# Define workspace bounds
bounds = [(0, 10), (0, 10)]  # (x_min, x_max), (y_min, y_max)

# Define obstacle function
polygon_obstacles = [
    [(2, 2), (3, 2), (3, 3), (2, 3)],  # square obstacle
    [(6, 6), (7, 6), (7, 7), (6, 7)]
]

def is_collision_free(point):
    for poly in polygon_obstacles:
        if point_in_polygon(point, poly):
            return False
    return True

# Initialize planner
planner = PRMPlanner(
    bounds=bounds,
    num_samples=1000,
    max_edge_length=2.0,
    collision_checker=is_collision_free
)

# Pre-compute roadmap
planner.precompute_roadmap()

# Query path
start = (1, 1)
goal = (9, 9)
path, cost = planner.plan(start, goal)

print(f"PRM Path found: {path is not None}")
print(f"Number of nodes in roadmap: {len(planner.graph.nodes)}")
```

**Applications**: Path planning in continuous spaces, high-dimensional configuration spaces

**Advantages**: Works in continuous spaces, handles complex obstacles, pre-computation allows multiple queries

**Limitations**: Pre-computation time, memory usage, probabilistic completeness

---

### 4. Rapidly-exploring Random Tree (RRT)

**Purpose**: Sampling-based path planning that grows a tree from start to goal

**Mathematical Model**:
- **Tree Construction**: Grow tree by randomly sampling and connecting to nearest neighbor
- **Biasing**: Bias sampling towards goal to accelerate convergence
- **Termination**: When goal region is reached

**Usage**:
```python
from robot_lab_algorithms.global_planning import RRTPlanner

# Define workspace
bounds = [(0, 10), (0, 10)]

# Initialize planner
planner = RRTPlanner(
    bounds=bounds,
    obstacle_checker=is_collision_free,
    max_iterations=5000,
    goal_bias=0.1,  # 10% of samples biased towards goal
    step_size=0.5
)

# Plan path
start = (1, 1)
goal = (9, 9)
path, cost = planner.plan(start, goal)

print(f"RRT Path found: {path is not None}")
print(f"Tree size: {planner.tree_size}")
```

**Applications**: Path planning in continuous spaces, non-holonomic systems

**Advantages**: Fast in practice, works in high dimensions, handles complex constraints

**Limitations**: Not optimal, probabilistic completeness

---

### 5. RRT* Algorithm

**Purpose**: Optimal sampling-based path planning that refines RRT

**Mathematical Model**:
- **RRT Foundation**: Grow tree like RRT
- **Rewiring**: After adding new node, check if it provides better paths to nearby nodes
- **Convergence**: Asymptotically optimal as number of samples increases

**Usage**:
```python
from robot_lab_algorithms.global_planning import RRTStarPlanner

# Define workspace
bounds = [(0, 10), (0, 10)]

# Initialize planner
planner = RRTStarPlanner(
    bounds=bounds,
    obstacle_checker=is_collision_free,
    max_iterations=5000,
    radius_function="k_nearest",  # or "fixed"
    k_nearest=10,
    step_size=0.5
)

# Plan path
start = (1, 1)
goal = (9, 9)
path, cost = planner.plan(start, goal)

print(f"RRT* Path cost: {cost}")
print(f"Optimal cost estimate: {planner.get_optimal_cost_estimate()}")
```

**Applications**: Optimal path planning in continuous spaces

**Advantages**: Asymptotically optimal, maintains RRT's fast convergence

**Limitations**: Computationally more expensive than RRT, convergence rate depends on parameters

---

### 6. Voronoi Planner

**Purpose**: Path planning using Voronoi diagrams to maximize clearance from obstacles

**Mathematical Model**:
- **Voronoi Diagram**: Partition of plane into regions based on distance to obstacles
- **Path Extraction**: Follow Voronoi edges from start to goal
- **Medial Axis**: Skeleton of free space with maximum clearance

**Usage**:
```python
from robot_lab_algorithms.global_planning import VoronoiPlanner

# Define obstacles
obstacles = [
    [(2, 2), (3, 2), (3, 3), (2, 3)],
    [(6, 6), (7, 6), (7, 7), (6, 7)]
]

# Initialize planner
planner = VoronoiPlanner(
    bounds=bounds,
    obstacles=obstacles,
    resolution=0.1
)

# Build Voronoi diagram
planner.build_diagram()

# Plan path
start = (1, 1)
goal = (9, 9)
path, cost = planner.plan(start, goal)

print(f"Voronoi path found: {path is not None}")
print(f"Path clearance: {planner.get_min_clearance(path)}")
```

**Applications**: Path planning with obstacle avoidance, maximizing clearance

**Advantages**: Naturally maximizes distance from obstacles, works in continuous spaces

**Limitations**: Computationally expensive to build diagram, sensitive to obstacle representation

---

## Local Planning Algorithms

### 1. Dynamic Window Approach (DWA)

**Purpose**: Reactive local planning that considers robot dynamics

**Mathematical Model**:
- **Velocity Space**: Sample (v, ω) in admissible velocity space
- **Trajectory Simulation**: For each velocity, simulate trajectory over prediction horizon
- **Cost Evaluation**: Evaluate cost based on obstacle avoidance, goal progression, and velocity
- **Optimal Control**: Select velocity with minimum cost

**Usage**:
```python
from robot_lab_algorithms.local_planning import DWBLocalPlanner
import numpy as np

# Initialize planner
planner = DWBLocalPlanner(
    max_velocity=0.5,
    min_velocity=-0.2,
    max_angular_velocity=1.0,
    max_angular_acceleration=2.0,
    prediction_horizon=2.0,
    cost_functions=["obstacle", "goal", "velocity"]
)

# Current robot state
current_state = [0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, theta, v, omega

# Global path (sequence of poses)
global_path = [[1, 0, 0], [2, 0, 0], [2, 1, np.pi/2]]

# Costmap data
costmap = np.zeros((10, 10))
costmap[3:7, 3:7] = 100  # obstacle in center

# Get optimal control
v, omega, trajectories = planner.compute_velocity_commands(
    current_state, global_path, costmap
)

print(f"Optimal velocity: {v}, angular velocity: {omega}")
```

---

### 2. Regulated Pure Pursuit

**Purpose**: Smooth path following using pure pursuit with regulation

**Mathematical Model**:
- **Pure Pursuit**: Drive towards lookahead point on path
- **Regulation**: Limit acceleration and curvature
- **Lookahead Distance**: Function of current velocity

**Usage**:
```python
from robot_lab_algorithms.local_planning import RegulatedPurePursuit

# Initialize planner
planner = RegulatedPurePursuit(
    lookahead_min=0.5,
    lookahead_max=2.0,
    max_curvature=1.0,
    max_acceleration=0.5,
    max_deceleration=1.0
)

# Current robot state
current_state = [0.0, 0.0, 0.0]  # x, y, theta

# Path to follow
path = [[0, 0], [1, 0], [2, 1], [3, 2]]

# Get control command
v, omega, lookahead_point = planner.compute_control(
    current_state, path, current_velocity=0.3
)

print(f"Command: v={v}, omega={omega}")
print(f"Lookahead point: {lookahead_point}")
```

---

### 3. Timed Elastic Band (TEB)

**Purpose**: Time-optimized trajectory generation with kinematic constraints

**Mathematical Model**:
- **Trajectory**: Parameterized by time and control points
- **Optimization**: Minimize time while satisfying constraints
- **Constraints**: Obstacle avoidance, kinematic limits, dynamic feasibility

**Usage**:
```python
from robot_lab_algorithms.local_planning import TEBLocalPlanner

# Initialize planner
planner = TEBLocalPlanner(
    max_velocity=0.5,
    max_acceleration=0.2,
    max_angular_velocity=1.0,
    max_angular_acceleration=0.5,
    obstacle_cost_weight=10.0,
    time_cost_weight=1.0
)

# Current robot state
current_state = [0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, theta, v, omega

# Global path
path = [[0, 0, 0], [1, 0, 0], [2, 1, np.pi/4]]

# Obstacles
obstacles = [[1.5, 0.5], [1.8, 0.3]]

# Optimize trajectory
success, trajectory, cost = planner.optimize_trajectory(
    current_state, path, obstacles
)

if success:
    print(f"Trajectory optimized with cost: {cost}")
    print(f"Trajectory duration: {trajectory.duration}")
```

---

### 4. Model Predictive Path Integral (MPPI)

**Purpose**: Sampling-based trajectory optimization using path integral theory

**Mathematical Model**:
- **Trajectory Sampling**: Generate perturbed trajectories around nominal
- **Cost Evaluation**: Compute cost for each trajectory
- **Optimal Control**: Weighted average of sampled trajectories based on cost
- **Iteration**: Repeatedly update nominal trajectory

**Usage**:
```python
from robot_lab_algorithms.local_planning import MPPILocalPlanner

# Initialize planner
planner = MPPILocalPlanner(
    horizon=2.0,           # time horizon
    num_samples=100,      # number of trajectory samples
    temperature=0.1,       # temperature parameter for softmax
    control_cost_weight=0.1,
    obstacle_cost_weight=10.0,
    goal_cost_weight=1.0
)

# Current robot state
current_state = [0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, theta, v, omega

# Goal
goal = [2.0, 1.0, 0.0]  # x, y, theta

# Obstacles
obstacles = [[1.0, 0.5], [1.5, 0.8]]

# Compute optimal trajectory
optimal_controls, optimal_trajectory, cost = planner.compute_optimal_trajectory(
    current_state, goal, obstacles
)

print(f"MPPI optimal cost: {cost}")
```

---

### 5. Follow-the-Gap

**Purpose**: Reactive navigation through gaps in sensor data

**Mathematical Model**:
- **Gap Detection**: Identify gaps in laser scan data
- **Gap Selection**: Choose best gap based on size, direction, and distance
- **Motion Control**: Drive towards selected gap

**Usage**:
```python
from robot_lab_algorithms.local_planning import FollowTheGap

# Initialize planner
planner = FollowTheGap(
    scan_range_max=10.0,
    scan_angle_min=-np.pi,
    scan_angle_max=np.pi,
    min_gap_size=0.5,
    max_gap_angle=0.5
)

# Laser scan data (ranges and angles)
ranges = [1.0, 1.2, 0.8, 1.5, 2.0, 1.8]  # distances
angles = [-np.pi, -np.pi/2, -np.pi/4, 0, np.pi/4, np.pi/2]  # angles

# Current robot pose
current_pose = [0.0, 0.0, 0.0]  # x, y, theta

# Goal direction
goal_direction = np.pi/2  # 90 degrees

# Compute control command
v, omega, selected_gap = planner.compute_control(
    current_pose, ranges, angles, goal_direction
)

print(f"Follow-the-gap command: v={v}, omega={omega}")
```

---

## Performance Metrics

### Path Quality Metrics

1. **Path Length**: Total distance traveled
2. **Path Cost**: Sum of edge weights or other cost function
3. **Clearance**: Minimum distance from obstacles
4. **Smoothness**: Curvature integral or jerk

### Planning Efficiency Metrics

1. **Planning Time**: Time to find solution
2. **Memory Usage**: Memory consumed during planning
3. **Nodes Expanded**: Number of nodes/vertices explored
4. **Success Rate**: Percentage of successful planning attempts

### Execution Metrics

1. **Tracking Error**: Deviation from planned path
2. **Collision Rate**: Frequency of collisions during execution
3. **Time to Goal**: Total time to reach goal
4. **Energy Efficiency**: Control effort or energy consumed

## Input Strata

### Grid-Based Planners
- **Dijkstra**: Works on regular grids, 2D/2.5D/3D
- **A***: Works on regular grids with heuristics

### Sampling-Based Planners  
- **PRM**: Works in continuous configuration spaces, 2D/3D
- **RRT**: Works in continuous spaces, handles non-holonomic constraints
- **RRT***: Works in continuous spaces, asymptotically optimal

### Geometric Planners
- **Voronoi**: Works in 2D planes, maximizes obstacle clearance

### Local Planners
- **DWB**: Works in 2D, considers dynamics, reactive
- **Regulated Pure Pursuit**: Works in 2D, smooth path following
- **TEB**: Works in 2D, time-optimized, considers kinematics
- **MPPI**: Works in 2D/3D, sampling-based optimization
- **Follow-the-Gap**: Works in 2D, reactive, sensor-based

## Usage Patterns

### ROS Integration

The planners integrate with ROS2 and Nav2:

```bash
# Launch Dijkstra planner node
ros2 run robot_lab_algorithms dijkstra_planner

# Launch A* planner node
ros2 run robot_lab_algorithms a_star_planner

# Launch RRT planner node
ros2 run robot_lab_algorithms rrt_planner
```

### Nav2 Plugin Integration

For Nav2 integration, the global planners are available as plugins:

```yaml
# Navigation configuration with A* planner
global_planner_plugin: nav2_navfn_planner/NavfnPlanner
# or
global_planner_plugin: robot_lab_planner/AStarPlanner
```

## Benchmarking Setup

```python
import time
import numpy as np
from robot_lab_algorithms.global_planning import *

def evaluate_planner(planner_class, maps, start_goals, **kwargs):
    results = []
    
    for map_data, (start, goal) in zip(maps, start_goals):
        # Initialize planner
        planner = planner_class(map_data, **kwargs)
        
        # Time planning
        start_time = time.time()
        path, cost = planner.plan(start, goal)
        planning_time = time.time() - start_time
        
        # Calculate metrics
        path_length = calculate_path_length(path) if path else float('inf')
        success = path is not None
        
        results.append({
            'success': success,
            'planning_time': planning_time,
            'path_length': path_length,
            'path_cost': cost,
            'nodes_expanded': getattr(planner, 'nodes_expanded', None)
        })
    
    return results
```

## Historical illustrative benchmark tables

These values have no linked executed trial, seed protocol, measured artifact
or source/host manifest. They remain unmeasured placeholders and cannot rank
methods or qualify R7/R9.2. Use [current status](../status/CURRENT_STATUS.md)
for actual named robot evidence and the workflow for a real comparison.

### Planning Success Rate and Time

| Algorithm | Success Rate | Planning Time (ms) | Nodes Expanded | Memory Usage (MB) |
|-----------|--------------|---------------------|----------------|-------------------|
| Dijkstra | 100% | 15.2 | 245 | 12.1 |
| A* | 100% | 8.7 | 123 | 10.8 |
| PRM | 98% | 25.4 | 89 | 15.6 |
| RRT | 95% | 12.8 | 342 | 8.2 |
| RRT* | 95% | 35.1 | 412 | 14.3 |
| Voronoi | 92% | 45.2 | N/A | 22.1 |

### Path Quality Comparison

| Algorithm | Path Length (m) | Clearance (m) | Smoothness | Obstacle Avoidance |
|-----------|------------------|---------------|------------|-------------------|
| Dijkstra | 8.45 | 0.23 | Medium | Good |
| A* | 8.45 | 0.23 | Medium | Good |
| PRM | 8.62 | 0.18 | Medium | Medium |
| RRT | 8.89 | 0.15 | Low | Medium |
| RRT* | 8.41 | 0.21 | Medium | Good |
| Voronoi | 8.73 | 0.35 | High | Excellent |

### Local Planner Performance

| Algorithm | Tracking Error (m) | Collision Rate | Time to Goal (s) | CPU Usage (%) |
|-----------|---------------------|----------------|------------------|---------------|
| DWB | 0.02 | 1% | 15.2 | 12 |
| Regulated Pure Pursuit | 0.03 | 2% | 14.8 | 8 |
| TEB | 0.01 | 0% | 12.1 | 25 |
| MPPI | 0.02 | 0% | 13.5 | 30 |
| Follow-the-Gap | 0.05 | 3% | 16.7 | 6 |

## Best Practices

1. **Planner Selection**: Choose based on environment characteristics
2. **Parameter Tuning**: Adjust parameters based on robot dynamics and environment
3. **Resolution**: Higher resolution for precise navigation, lower for speed
4. **Obstacle Inflation**: Increase obstacle sizes for safety margin
5. **Cost Functions**: Customize cost functions for specific requirements

## Troubleshooting

### Common Issues

**No Path Found**:
- Check if start/goal are in free space
- Increase resolution or number of samples
- Reduce obstacle sizes or add safety margins
- Verify collision checking function

**Path Too Close to Obstacles**:
- Increase cost for proximity to obstacles
- Use planners that maximize clearance (Voronoi)
- Add inflation to obstacles

**Planning Takes Too Long**:
- Reduce resolution or grid size
- Use sampling-based planners (RRT, PRM) for complex environments
- Implement hierarchical planning
- Use pre-computation where possible

**Jerk/Discomfort in Motion**:
- Use smooth cost functions
- Implement trajectory smoothing post-processing
- Use planners that consider dynamics (TEB, MPPI)

## Advanced Topics

### Anytime Planning

```python
class AnytimePlanner:
    def __init__(self, base_planner):
        self.base_planner = base_planner
        self.solution = None
        self.best_path = None
        self.best_cost = float('inf')
        
    def plan_with_time_limit(self, start, goal, time_limit):
        start_time = time.time()
        
        while time.time() - start_time < time_limit:
            # Run planner with increasing quality
            path, cost = self.base_planner.plan_anytime(
                start, goal, 
                time.time() - start_time
            )
            
            if cost < self.best_cost:
                self.best_cost = cost
                self.best_path = path
                
        return self.best_path, self.best_cost
```

### Hybrid Planning

Combine multiple planners for different phases:

```python
class HybridPlanner:
    def __init__(self):
        self.coarse_planner = AStarPlanner(resolution=0.5)
        self.fine_planner = RRTStarPlanner(step_size=0.1)
        
    def plan(self, start, goal, obstacles):
        # Phase 1: Coarse planning
        coarse_path = self.coarse_planner.plan(start, goal)
        
        if not coarse_path:
            return None
            
        # Phase 2: Fine planning around obstacles
        fine_path = []
        for i in range(len(coarse_path) - 1):
            segment_start = coarse_path[i]
            segment_goal = coarse_path[i + 1]
            
            # Check if direct path is collision-free
            if self.is_segment_collision_free(segment_start, segment_goal, obstacles):
                fine_path.extend([segment_start, segment_goal])
            else:
                # Use fine planner for this segment
                segment_path = self.fine_planner.plan(segment_start, segment_goal)
                if not segment_path:
                    return None
                fine_path.extend(segment_path)
        
        return fine_path
```

## References

- [Planning Algorithms](https://planning.cs.uiuc.edu/)
- [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [RRT Algorithm](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)
- [PRM Algorithm](https://en.wikipedia.org/wiki/Probabilistic_roadmap)
- [Voronoi Diagram](https://en.wikipedia.org/wiki/Voronoi_diagram)
- [Local Planning for Mobile Robots](https://ieeexplore.ieee.org/document/8461140)

## Run

This page documents planned methods or configuration. Consult the
[workflow](../WORKFLOW.md) and [completion audit](../status/audit-2026-10-02.md)
for executable, scoped tests and remaining qualification. No measured comparison
is established by this page alone.
