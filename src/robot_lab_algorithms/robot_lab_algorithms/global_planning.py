"""Global planning algorithms (P5).

Adds two global planners that round out the global_planning category:
  1. rrt_planner     - Rapidly-exploring Random Tree sampling-based planner.
  2. voronoi_planner - grid-based planner following Voronoi-roadmap-like
                       clearance-maximizing corridors (approximated by a
                       distance-transform ridge follower on a cost grid).

Both are pure-Python deterministic implementations that plan a waypoint path
from a start to a goal over an occupancy grid.
"""

from __future__ import annotations

import math
import random
import sys


try:
    import rclpy
    from rclpy.node import Node as _RclpyNode
    Node = _RclpyNode
except Exception:  # pragma: no cover - optional dependency
    rclpy = None

    class Node:
        """Fallback base when rclpy is unavailable (dry/local testing)."""
        def __init__(self, node_name='node'):
            self._node_name = node_name
            self._params = {}
        def declare_parameter(self, name, value=None):
            self._params.setdefault(name, value)
        def get_parameter(self, name):
            class _P:
                value = self._params.get(name, None)
            return _P()
        def get_logger(self):
            name = self._node_name
            class _L:
                def info(self, *a, **k):
                    print(f'[{name}]', *a)
                def warn(self, *a, **k):
                    print(f'[{name}] WARN', *a)
            return _L()
        def get_name(self):
            return self._node_name


def _to_grid(x, y, origin, resolution):
    return int(round((x - origin[0]) / resolution)), int(round((y - origin[1]) / resolution))


def _from_grid(c, r, origin, resolution):
    return origin[0] + c * resolution, origin[1] + r * resolution


class RRTPlanner:
    """Sampling-based RRT planner over a binary occupancy grid."""

    def __init__(self, step=1.0, max_iter=2000, goal_tol=0.6):
        self.step = step
        self.max_iter = max_iter
        self.goal_tol = goal_tol

    def plan(self, start, goal, is_free, bounds, resolution=0.05, seed=0):
        """Return a list of (x, y) waypoints from start to goal.

        is_free(x, y) -> bool; bounds = (min_x, min_y, max_x, max_y).
        """
        rng = random.Random(seed)
        nodes = [start]
        parents = {start: None}
        goal_reached = None
        for _ in range(self.max_iter):
            if rng.random() < 0.2:
                sample = goal
            else:
                sample = (
                    rng.uniform(bounds[0], bounds[2]),
                    rng.uniform(bounds[1], bounds[3]),
                )
            if not is_free(sample[0], sample[1]):
                continue
            nearest = min(nodes, key=lambda n: math.hypot(n[0] - sample[0], n[1] - sample[1]))
            d = math.hypot(sample[0] - nearest[0], sample[1] - nearest[1])
            if d < 1e-6:
                continue
            newx = nearest[0] + (sample[0] - nearest[0]) * min(1.0, self.step / d)
            newy = nearest[1] + (sample[1] - nearest[1]) * min(1.0, self.step / d)
            if not is_free(newx, newy):
                continue
            new = (newx, newy)
            if new in parents:
                continue
            nodes.append(new)
            parents[new] = nearest
            if math.hypot(new[0] - goal[0], new[1] - goal[1]) <= self.goal_tol:
                goal_reached = new
                break
        if goal_reached is None:
            return []
        path = []
        node = goal_reached
        while node is not None:
            path.append(node)
            node = parents[node]
        path.reverse()
        return path


class VoronoiPlanner:
    """Ridge-following global planner that maximizes local clearance."""

    def __init__(self):
        pass

    def plan(self, start, goal, cost_at, step=0.5, max_steps=2000):
        """Greedy walk that ascends cost ridges toward the goal.

        cost_at(x, y) -> float clearance cost (higher is more clear).
        """
        current = start
        path = [current]
        for _ in range(max_steps):
            if math.hypot(current[0] - goal[0], current[1] - goal[1]) < step:
                path.append(goal)
                return path
            best = None
            best_cost = -1.0
            for dx in (-step, 0.0, step):
                for dy in (-step, 0.0, step):
                    if dx == 0.0 and dy == 0.0:
                        continue
                    cand = (current[0] + dx, current[1] + dy)
                    c = cost_at(cand[0], cand[1])
                    if c <= 0.0:
                        continue
                    if c > best_cost:
                        best_cost = c
                        best = cand
            if best is None:
                # no free neighbor; fall back toward goal
                best = (current[0] + math.copysign(step, goal[0] - current[0]),
                        current[1] + math.copysign(step, goal[1] - current[1]))
            path.append(best)
            current = best
        return path


class DijkstraPlanner:
    """Dijkstra's algorithm for shortest path planning on a grid."""

    def __init__(self, resolution=0.1):
        self.resolution = resolution

    def plan(self, start, goal, is_free, bounds):
        """Find shortest path using Dijkstra's algorithm.
        
        is_free(x, y) -> bool: check if point is collision-free
        bounds = (min_x, min_y, max_x, max_y)
        
        Returns list of (x, y) waypoints from start to goal.
        """
        min_x, min_y, max_x, max_y = bounds
        
        # Create grid
        width = int(math.ceil((max_x - min_x) / self.resolution))
        height = int(math.ceil((max_y - min_y) / self.resolution))
        
        # Convert start and goal to grid coordinates
        start_gx = int(round((start[0] - min_x) / self.resolution))
        start_gy = int(round((start[1] - min_y) / self.resolution))
        goal_gx = int(round((goal[0] - min_x) / self.resolution))
        goal_gy = int(round((goal[1] - min_y) / self.resolution))
        
        # Check bounds
        if (start_gx < 0 or start_gy < 0 or start_gx >= width or start_gy >= height or
            goal_gx < 0 or goal_gy < 0 or goal_gx >= width or goal_gy >= height):
            return []
        
        # Dijkstra's algorithm
        distances = [[float('inf')] * width for _ in range(height)]
        previous = [[None] * width for _ in range(height)]
        visited = [[False] * width for _ in range(height)]
        
        distances[start_gy][start_gx] = 0.0
        
        # Priority queue (simple implementation using list)
        queue = [(0.0, start_gx, start_gy)]
        
        while queue:
            # Get node with minimum distance
            queue.sort()
            dist, x, y = queue.pop(0)
            
            if visited[y][x]:
                continue
            visited[y][x] = True
            
            # Check if we reached the goal
            if x == goal_gx and y == goal_gy:
                break
            
            # Check neighbors
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and not visited[ny][nx]:
                    # Convert grid coordinates to world coordinates
                    wx = min_x + nx * self.resolution
                    wy = min_y + ny * self.resolution
                    
                    if is_free(wx, wy):
                        # Euclidean distance (1 for cardinal, sqrt(2) for diagonal)
                        move_cost = math.sqrt(dx*dx + dy*dy)
                        new_dist = distances[y][x] + move_cost
                        
                        if new_dist < distances[ny][nx]:
                            distances[ny][nx] = new_dist
                            previous[ny][nx] = (x, y)
                            queue.append((new_dist, nx, ny))
        
        # Reconstruct path
        path = []
        current = (goal_gx, goal_gy)
        
        while current is not None:
            x, y = current
            # Convert back to world coordinates
            wx = min_x + x * self.resolution
            wy = min_y + y * self.resolution
            path.append((wx, wy))
            current = previous[y][x]
        
        path.reverse()
        
        # Remove goal if it's not exactly reachable
        if len(path) > 0 and math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) > self.resolution * 2:
            path = []
        
        return path


class AStarPlanner:
    """A* algorithm for path planning with heuristic."""

    def __init__(self, resolution=0.1, heuristic_weight=1.0):
        self.resolution = resolution
        self.heuristic_weight = heuristic_weight

    def plan(self, start, goal, is_free, bounds):
        """Find path using A* algorithm.
        
        is_free(x, y) -> bool: check if point is collision-free
        bounds = (min_x, min_y, max_x, max_y)
        
        Returns list of (x, y) waypoints from start to goal.
        """
        min_x, min_y, max_x, max_y = bounds
        
        # Create grid
        width = int(math.ceil((max_x - min_x) / self.resolution))
        height = int(math.ceil((max_y - min_y) / self.resolution))
        
        # Convert start and goal to grid coordinates
        start_gx = int(round((start[0] - min_x) / self.resolution))
        start_gy = int(round((start[1] - min_y) / self.resolution))
        goal_gx = int(round((goal[0] - min_x) / self.resolution))
        goal_gy = int(round((goal[1] - min_y) / self.resolution))
        
        # Check bounds
        if (start_gx < 0 or start_gy < 0 or start_gx >= width or start_gy >= height or
            goal_gx < 0 or goal_gy < 0 or goal_gx >= width or goal_gy >= height):
            return []
        
        # A* algorithm
        g_scores = [[float('inf')] * width for _ in range(height)]
        f_scores = [[float('inf')] * width for _ in range(height)]
        previous = [[None] * width for _ in range(height)]
        visited = [[False] * width for _ in range(height)]
        
        g_scores[start_gy][start_gx] = 0.0
        f_scores[start_gy][start_gx] = self._heuristic(start_gx, start_gy, goal_gx, goal_gy)
        
        # Priority queue
        queue = [(f_scores[start_gy][start_gx], start_gx, start_gy)]
        
        while queue:
            # Get node with minimum f_score
            queue.sort()
            f_score, x, y = queue.pop(0)
            
            if visited[y][x]:
                continue
            visited[y][x] = True
            
            # Check if we reached the goal
            if x == goal_gx and y == goal_gy:
                break
            
            # Check neighbors
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and not visited[ny][nx]:
                    # Convert grid coordinates to world coordinates
                    wx = min_x + nx * self.resolution
                    wy = min_y + ny * self.resolution
                    
                    if is_free(wx, wy):
                        # Euclidean distance (1 for cardinal, sqrt(2) for diagonal)
                        move_cost = math.sqrt(dx*dx + dy*dy)
                        new_g_score = g_scores[y][x] + move_cost
                        
                        if new_g_score < g_scores[ny][nx]:
                            g_scores[ny][nx] = new_g_score
                            f_scores[ny][nx] = new_g_score + self.heuristic_weight * self._heuristic(nx, ny, goal_gx, goal_gy)
                            previous[ny][nx] = (x, y)
                            queue.append((f_scores[ny][nx], nx, ny))
        
        # Reconstruct path
        path = []
        current = (goal_gx, goal_gy)
        
        while current is not None:
            x, y = current
            # Convert back to world coordinates
            wx = min_x + x * self.resolution
            wy = min_y + y * self.resolution
            path.append((wx, wy))
            current = previous[y][x]
        
        path.reverse()
        
        return path

    def _heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


class PRMPlanner:
    """Probabilistic Roadmap (PRM) planner for sampling-based path planning."""

    def __init__(self, num_samples=100, max_neighbors=10, max_connection_distance=2.0):
        self.num_samples = num_samples
        self.max_neighbors = max_neighbors
        self.max_connection_distance = max_connection_distance

    def plan(self, start, goal, is_free, bounds):
        """Find path using PRM algorithm.
        
        is_free(x, y) -> bool: check if point is collision-free
        bounds = (min_x, min_y, max_x, max_y)
        
        Returns list of (x, y) waypoints from start to goal.
        """
        min_x, min_y, max_x, max_y = bounds
        
        # Generate random samples
        import random
        samples = [start, goal]  # Always include start and goal
        
        rng = random.Random(42)  # Fixed seed for reproducibility
        for _ in range(self.num_samples):
            x = rng.uniform(min_x, max_x)
            y = rng.uniform(min_y, max_y)
            if is_free(x, y):
                samples.append((x, y))
        
        # Build roadmap (connect samples within max_connection_distance)
        roadmap = {}
        for i, sample in enumerate(samples):
            roadmap[i] = []
            for j, other in enumerate(samples):
                if i != j:
                    distance = math.hypot(sample[0] - other[0], sample[1] - other[1])
                    if distance <= self.max_connection_distance:
                        # Check if the connection is collision-free
                        if self._is_path_free(sample, other, is_free):
                            roadmap[i].append(j)
        
        # Find path from start (index 0) to goal (index 1) using BFS
        path_indices = self._bfs(roadmap, 0, 1)
        
        if not path_indices:
            return []
        
        # Convert indices to points
        path = [samples[i] for i in path_indices]
        return path

    def _is_path_free(self, start, end, is_free):
        """Check if the direct path between two points is collision-free."""
        # Discretize the path and check points along it
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            if not is_free(x, y):
                return False
        return True

    def _bfs(self, graph, start, goal):
        """Breadth-first search to find path in roadmap."""
        from collections import deque
        
        visited = set()
        previous = {}
        queue = deque([start])
        visited.add(start)
        
        while queue:
            current = queue.popleft()
            
            if current == goal:
                break
            
            for neighbor in graph[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    previous[neighbor] = current
                    queue.append(neighbor)
        
        # Reconstruct path
        if goal not in previous:
            return []
        
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = previous[current]
        path.append(start)
        path.reverse()
        
        return path


class RRTStarPlanner:
    """RRT* algorithm - asymptotically optimal RRT variant."""

    def __init__(self, step=1.0, max_iter=2000, goal_tol=0.6, radius=1.5):
        self.step = step
        self.max_iter = max_iter
        self.goal_tol = goal_tol
        self.radius = radius  # Neighborhood radius for rewiring

    def plan(self, start, goal, is_free, bounds):
        """Find path using RRT* algorithm.
        
        is_free(x, y) -> bool: check if point is collision-free
        bounds = (min_x, min_y, max_x, max_y)
        
        Returns list of (x, y) waypoints from start to goal.
        """
        import random
        rng = random.Random(42)  # Fixed seed for reproducibility
        
        nodes = [start]
        parents = {start: None}
        costs = {start: 0.0}
        goal_reached = None
        
        for _ in range(self.max_iter):
            # Sample a random point (with bias toward goal)
            if rng.random() < 0.2:
                sample = goal
            else:
                sample = (
                    rng.uniform(bounds[0], bounds[2]),
                    rng.uniform(bounds[1], bounds[3]),
                )
            
            if not is_free(sample[0], sample[1]):
                continue
            
            # Find nearest node
            nearest = min(nodes, key=lambda n: math.hypot(n[0] - sample[0], n[1] - sample[1]))
            
            # Steer toward sample
            d = math.hypot(sample[0] - nearest[0], sample[1] - nearest[1])
            if d < 1e-6:
                continue
            
            # Find the new node in the direction of the sample
            new_x = nearest[0] + (sample[0] - nearest[0]) * min(1.0, self.step / d)
            new_y = nearest[1] + (sample[1] - nearest[1]) * min(1.0, self.step / d)
            new_node = (new_x, new_y)
            
            # Check collision
            if not self._is_path_free(nearest, new_node, is_free):
                continue
            
            # Find the nearest node for connection (not just the initial nearest)
            near_nodes = [n for n in nodes if math.hypot(n[0] - new_x, n[1] - new_y) <= self.radius]
            
            if not near_nodes:
                # No nearby nodes, connect to nearest
                cost_to_new = costs[nearest] + math.hypot(new_x - nearest[0], new_y - nearest[1])
                parent = nearest
            else:
                # Find the best parent among nearby nodes
                min_cost = float('inf')
                parent = None
                cost_to_new = float('inf')
                
                for near_node in near_nodes:
                    cost = costs[near_node] + math.hypot(new_x - near_node[0], new_y - near_node[1])
                    if cost < min_cost:
                        min_cost = cost
                        parent = near_node
                        cost_to_new = cost
            
            # Add the new node
            nodes.append(new_node)
            parents[new_node] = parent
            costs[new_node] = cost_to_new
            
            # Rewire nearby nodes to improve the tree
            for near_node in near_nodes:
                if near_node == new_node:
                    continue
                
                # Cost from new node to near node
                new_cost = cost_to_new + math.hypot(new_x - near_node[0], new_y - near_node[1])
                if new_cost < costs[near_node]:
                    # Check collision-free path
                    if self._is_path_free(new_node, near_node, is_free):
                        parents[near_node] = new_node
                        costs[near_node] = new_cost
            
            # Check if we reached the goal
            if math.hypot(new_x - goal[0], new_y - goal[1]) <= self.goal_tol:
                goal_reached = new_node
                break
        
        if goal_reached is None:
            return []
        
        # Reconstruct path
        path = []
        node = goal_reached
        while node is not None:
            path.append(node)
            node = parents[node]
        path.reverse()
        
        return path

    def _is_path_free(self, start, end, is_free):
        """Check if the direct path between two points is collision-free."""
        steps = 5
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            if not is_free(x, y):
                return False
        return True


# ---------------------------------------------------------------------------
# ROS 2 node wrappers (what the console entry points run)
# ---------------------------------------------------------------------------

from ._runtime import run as _run, spin_node as _spin_node  # noqa: E402


class _GridPlannerNode(Node):
    """Shared plumbing for grid planners: /map + /goal_pose -> nav_msgs/Path.

    These planners run alongside the Nav2 stack rather than as Nav2 plugins:
    they consume the same occupancy map and goal, and publish their own plan
    so it can be compared against the plugin planner's output.
    """

    def __init__(self, node_name, default_output):
        super().__init__(node_name)
        from nav_msgs.msg import OccupancyGrid, Path
        from geometry_msgs.msg import PoseStamped
        self._path_type = Path
        self._pose_type = PoseStamped
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('path_topic', default_output)
        self.declare_parameter('occupied_threshold', 50)
        self._pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        from nav_msgs.msg import Odometry
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)
        self._map = None
        self._start = (0.0, 0.0)
        self.path = []
        self.get_logger().info('%s ready (waiting for /map and a goal)' % node_name)

    def _on_map(self, msg):
        self._map = msg

    def _on_odom(self, msg):
        self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _world_is_free(self, x, y):
        grid = self._map
        if grid is None:
            return True
        column = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if column < 0 or row < 0 or column >= grid.info.width \
                or row >= grid.info.height:
            return False
        value = grid.data[row * grid.info.width + column]
        threshold = int(self.get_parameter('occupied_threshold').value)
        return value >= 0 and value < threshold

    def _bounds(self):
        grid = self._map
        if grid is None:
            return (-10.0, -10.0, 10.0, 10.0)
        return (
            grid.info.origin.position.x,
            grid.info.origin.position.y,
            grid.info.origin.position.x + grid.info.width * grid.info.resolution,
            grid.info.origin.position.y + grid.info.height * grid.info.resolution,
        )

    def _publish_path(self, waypoints, frame_id):
        path = self._path_type()
        path.header.frame_id = frame_id or 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = self._pose_type()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path = list(waypoints)
        self._pub.publish(path)

    def _on_goal(self, msg):  # pragma: no cover - overridden
        raise NotImplementedError


class RRTPlannerNode(_GridPlannerNode):
    """Sampling-based RRT global planner publishing /plan/rrt."""

    def __init__(self, node_name='rrt_planner'):
        super().__init__(node_name, '/plan/rrt')
        self.declare_parameter('step', 1.0)
        self.declare_parameter('max_iterations', 2000)
        self.declare_parameter('goal_tolerance', 0.6)
        self.declare_parameter('seed', 0)
        self.planner = RRTPlanner(
            step=float(self.get_parameter('step').value),
            max_iter=int(self.get_parameter('max_iterations').value),
            goal_tol=float(self.get_parameter('goal_tolerance').value))

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._world_is_free, self._bounds(),
            seed=int(self.get_parameter('seed').value))
        if not waypoints:
            self.get_logger().warn('RRT found no path to (%.2f, %.2f)' % goal)
            return
        self._publish_path(waypoints, msg.header.frame_id)


class VoronoiPlannerNode(_GridPlannerNode):
    """Clearance-maximizing global planner publishing /plan/voronoi."""

    def __init__(self, node_name='voronoi_planner'):
        super().__init__(node_name, '/plan/voronoi')
        self.declare_parameter('step', 0.5)
        self.declare_parameter('max_steps', 2000)
        self.planner = VoronoiPlanner()

    def _clearance_at(self, x, y):
        """Distance-to-obstacle proxy: how far a free ring stays free."""
        if not self._world_is_free(x, y):
            return 0.0
        clearance = 0.0
        for radius in (0.2, 0.4, 0.6, 0.8):
            blocked = any(
                not self._world_is_free(x + radius * math.cos(a),
                                        y + radius * math.sin(a))
                for a in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2))
            if blocked:
                break
            clearance = radius
        return clearance + 0.1

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._clearance_at,
            step=float(self.get_parameter('step').value),
            max_steps=int(self.get_parameter('max_steps').value))
        self._publish_path(waypoints, msg.header.frame_id)


def rrt_planner_main(args=None):
    return _run(RRTPlannerNode, 'rrt_planner', args=args)


def voronoi_planner_main(args=None):
    return _run(VoronoiPlannerNode, 'voronoi_planner', args=args)


class DijkstraPlannerNode(_GridPlannerNode):
    """Dijkstra global planner node."""

    def __init__(self, node_name='dijkstra_planner'):
        super().__init__(node_name, '/plan/dijkstra')
        self.declare_parameter('resolution', 0.1)
        self.planner = DijkstraPlanner(
            resolution=float(self.get_parameter('resolution').value))

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._world_is_free, self._bounds(),
            resolution=float(self.get_parameter('resolution').value))
        if not waypoints:
            self.get_logger().warn('Dijkstra found no path to (%.2f, %.2f)' % goal)
            return
        self._publish_path(waypoints, msg.header.frame_id)


class AStarPlannerNode(_GridPlannerNode):
    """A* global planner node."""

    def __init__(self, node_name='a_star_planner'):
        super().__init__(node_name, '/plan/a_star')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('heuristic_weight', 1.0)
        self.planner = AStarPlanner(
            resolution=float(self.get_parameter('resolution').value),
            heuristic_weight=float(self.get_parameter('heuristic_weight').value))

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._world_is_free, self._bounds())
        if not waypoints:
            self.get_logger().warn('A* found no path to (%.2f, %.2f)' % goal)
            return
        self._publish_path(waypoints, msg.header.frame_id)


class PRMPlannerNode(Node):
    """PRM global planner node."""

    def __init__(self, node_name='prm_planner'):
        super().__init__(node_name)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('path_topic', '/plan/prm')
        self.declare_parameter('num_samples', 100)
        self.declare_parameter('max_neighbors', 10)
        self.declare_parameter('max_connection_distance', 2.0)
        self.declare_parameter('occupied_threshold', 50)
        
        from nav_msgs.msg import OccupancyGrid, Path
        from geometry_msgs.msg import PoseStamped
        self._path_type = Path
        self._pose_type = PoseStamped
        self._pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        from nav_msgs.msg import Odometry
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)
        
        self.planner = PRMPlanner(
            num_samples=int(self.get_parameter('num_samples').value),
            max_neighbors=int(self.get_parameter('max_neighbors').value),
            max_connection_distance=float(self.get_parameter('max_connection_distance').value))
        
        self._map = None
        self._start = (0.0, 0.0)
        self.get_logger().info('PRM Planner ready (waiting for /map and a goal)')

    def _on_map(self, msg):
        self._map = msg

    def _on_odom(self, msg):
        self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _world_is_free(self, x, y):
        grid = self._map
        if grid is None:
            return True
        column = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if column < 0 or row < 0 or column >= grid.info.width \
                or row >= grid.info.height:
            return False
        value = grid.data[row * grid.info.width + column]
        threshold = int(self.get_parameter('occupied_threshold').value)
        return value >= 0 and value < threshold

    def _bounds(self):
        grid = self._map
        if grid is None:
            return (-10.0, -10.0, 10.0, 10.0)
        return (
            grid.info.origin.position.x,
            grid.info.origin.position.y,
            grid.info.origin.position.x + grid.info.width * grid.info.resolution,
            grid.info.origin.position.y + grid.info.height * grid.info.resolution,
        )

    def _publish_path(self, waypoints, frame_id):
        path = self._path_type()
        path.header.frame_id = frame_id or 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = self._pose_type()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._pub.publish(path)

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._world_is_free, self._bounds())
        if not waypoints:
            self.get_logger().warn('PRM found no path to (%.2f, %.2f)' % goal)
            return
        self._publish_path(waypoints, msg.header.frame_id)


class RRTStarPlannerNode(_GridPlannerNode):
    """RRT* global planner node."""

    def __init__(self, node_name='rrt_star_planner'):
        super().__init__(node_name, '/plan/rrt_star')
        self.declare_parameter('step', 1.0)
        self.declare_parameter('max_iterations', 2000)
        self.declare_parameter('goal_tolerance', 0.6)
        self.declare_parameter('radius', 1.5)
        self.declare_parameter('seed', 0)
        self.planner = RRTStarPlanner(
            step=float(self.get_parameter('step').value),
            max_iter=int(self.get_parameter('max_iterations').value),
            goal_tol=float(self.get_parameter('goal_tolerance').value),
            radius=float(self.get_parameter('radius').value))

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._world_is_free, self._bounds(),
            seed=int(self.get_parameter('seed').value))
        if not waypoints:
            self.get_logger().warn('RRT* found no path to (%.2f, %.2f)' % goal)
            return
        self._publish_path(waypoints, msg.header.frame_id)


def dijkstra_planner_main(args=None):
    return _run(DijkstraPlannerNode, 'dijkstra_planner', args=args)


def a_star_planner_main(args=None):
    return _run(AStarPlannerNode, 'a_star_planner', args=args)


def prm_planner_main(args=None):
    return _run(PRMPlannerNode, 'prm_planner', args=args)


def rrt_star_planner_main(args=None):
    return _run(RRTStarPlannerNode, 'rrt_star_planner', args=args)


if __name__ == '__main__':
    sys.exit(rrt_planner_main())
