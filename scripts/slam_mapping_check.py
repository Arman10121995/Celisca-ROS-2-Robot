#!/usr/bin/env python3
"""Drive a robot through a known world in slam mode and score its map.

Runs against a launched ``mode:=slam`` stack.  The robot is driven along an
A* route through the world's reference occupancy map (inflated by
``--radius``), visiting ``--waypoints`` and returning to the start.  Only
the drive harness looks at simulator truth (/odom/ground_truth); slam_toolbox
sees the robot's own scan and odometry as in any slam run.  The drive steers
with forward speed and turn rate; with ``--holonomic`` a mecanum or crab /
in-phase steered base instead translates toward the route at a fixed heading.

Scores, all against simulator truth and the reference map:

* trajectory: the slam pose (TF map -> base) against truth after aligning
  the two frames at the start (RMSE, maximum, final);
* map precision: the share of the slam map's occupied cells that lie within
  0.10 m (two cells) of a reference occupied cell, after the same alignment
  (no invented walls);
* map recall: the share of reference wall-surface cells (occupied cells
  touching free space) within 3 m of the driven path that have a slam
  occupied cell within 0.10 m;
* the mean distance from slam occupied cells to the nearest reference wall
  surface.

Prints one JSON object; ``--map-out`` also saves the final slam map as PGM.
"""
import argparse
import heapq
import json
import math
import os
import time

import numpy as np
import yaml
from scipy import ndimage
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class ReferenceMap:
    def __init__(self, name):
        folder = os.path.join(ROOT, 'src', 'robot_lab_maps', 'maps', name, 'maps')
        meta = yaml.safe_load(open(os.path.join(folder, 'map.yaml')))
        with open(os.path.join(folder, meta['image']), 'rb') as handle:
            assert handle.readline().strip() == b'P5'
            line = handle.readline()
            while line.startswith(b'#'):
                line = handle.readline()
            width, height = (int(v) for v in line.split())
            int(handle.readline())
            image = np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width)
        occ = image.astype(float) / 255.0 if meta.get('negate') else (255 - image.astype(float)) / 255.0
        self.occupied = np.flipud(occ > meta['occupied_thresh'])
        self.free = np.flipud(occ < meta['free_thresh'])
        self.resolution = float(meta['resolution'])
        self.origin = np.array(meta['origin'][:2], dtype=float)
        # Wall surfaces only (occupied cells touching free space): the interior
        # of a thick wall is never seen, so it must not count against recall.
        touches_free = np.zeros_like(self.occupied)
        touches_free[1:, :] |= self.free[:-1, :]
        touches_free[:-1, :] |= self.free[1:, :]
        touches_free[:, 1:] |= self.free[:, :-1]
        touches_free[:, :-1] |= self.free[:, 1:]
        rows, cols = np.nonzero(self.occupied & touches_free)
        self.wall_points = np.column_stack([cols, rows]) * self.resolution + self.origin + self.resolution / 2
        self.tree = cKDTree(self.wall_points)
        rows, cols = np.nonzero(self.occupied)
        self.occupied_tree = cKDTree(np.column_stack([cols, rows]) * self.resolution + self.origin
                                     + self.resolution / 2)

    def cell(self, x, y):
        return (int((y - self.origin[1]) / self.resolution), int((x - self.origin[0]) / self.resolution))

    def point(self, row, col):
        return (self.origin[0] + (col + 0.5) * self.resolution, self.origin[1] + (row + 0.5) * self.resolution)

    def open_point(self, x, y, clearance):
        """The nearest point at least ``clearance`` from any wall."""
        free = ndimage.distance_transform_edt(~self.occupied) * self.resolution
        r, c = self.cell(x, y)
        if free[r, c] >= clearance:
            return (x, y)
        rows, cols = np.nonzero((free >= clearance) & self.free)
        k = int(np.argmin((rows - r) ** 2 + (cols - c) ** 2))
        return self.point(rows[k], cols[k])

    def route(self, start, goal, radius):
        """A* over cells farther than ``radius`` from any wall (8-connected)."""
        clearance = ndimage.distance_transform_edt(~self.occupied) * self.resolution
        passable = (clearance > radius) & self.free
        s, g = self.cell(*start), self.cell(*goal)
        for name, c in (('start', s), ('goal', g)):
            if not passable[c]:
                # Snap to the nearest passable cell (the robot is there physically).
                rows, cols = np.nonzero(passable)
                k = int(np.argmin((rows - c[0]) ** 2 + (cols - c[1]) ** 2))
                if name == 'start':
                    s = (rows[k], cols[k])
                else:
                    g = (rows[k], cols[k])
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        cost = {s: 0.0}
        parent = {s: None}
        frontier = [(0.0, s)]
        while frontier:
            _, c = heapq.heappop(frontier)
            if c == g:
                break
            for dr, dc, w in steps:
                n = (c[0] + dr, c[1] + dc)
                if not (0 <= n[0] < passable.shape[0] and 0 <= n[1] < passable.shape[1]) or not passable[n]:
                    continue
                # Prefer the middle of corridors: penalise low clearance.
                new = cost[c] + w * (1.0 + 0.5 / max(clearance[n], 0.05))
                if new < cost.get(n, float('inf')):
                    cost[n] = new
                    parent[n] = c
                    heapq.heappush(frontier, (new + math.hypot(n[0] - g[0], n[1] - g[1]), n))
        if g not in parent:
            raise RuntimeError('no route from %s to %s' % (start, goal))
        cells, c = [], g
        while c is not None:
            cells.append(c)
            c = parent[c]
        cells.reverse()
        return [self.point(r, c) for r, c in cells[::4]] + [self.point(*cells[-1])]


def score_map(reference, world, path):
    """Map scores for slam occupied cells ``world`` (N x 2, world frame).

    precision: share of slam occupied cells within 0.10 m of a reference
    occupied cell (no invented walls); wall distance: from slam cells to the
    nearest reference wall *surface*; recall: share of reference wall
    surfaces within 3 m of the driven ``path`` with a slam cell within
    0.10 m.
    """
    if not len(world):
        return {'slam_occupied_cells': 0}
    near_any, _ = reference.occupied_tree.query(world)
    near_surface, _ = reference.tree.query(world)
    ref_dist_to_path, _ = cKDTree(path).query(reference.wall_points)
    seen_walls = reference.wall_points[ref_dist_to_path < 3.0]
    recall_hits = cKDTree(world).query(seen_walls)[0] <= 0.10 if len(seen_walls) else np.zeros(0)
    return {
        'slam_occupied_cells': int(len(world)),
        'precision_0p10': round(float(np.mean(near_any <= 0.10)), 3),
        'mean_surface_distance_m': round(float(np.mean(near_surface)), 3),
        'p95_surface_distance_m': round(float(np.percentile(near_surface, 95)), 3),
        'reference_walls_near_path': int(len(seen_walls)),
        'recall_0p10': round(float(np.mean(recall_hits)), 3) if len(recall_hits) else None}


def rescore(map_name, pgm, bag):
    """Score a saved slam map (PGM + YAML with world_from_map) against the
    reference, with the driven path read from the run's bag."""
    import glob
    import sqlite3
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    reference = ReferenceMap(map_name)
    meta = yaml.safe_load(open(os.path.splitext(pgm)[0] + '.yaml'))
    with open(pgm, 'rb') as handle:
        handle.readline()
        width, height = (int(v) for v in handle.readline().split())
        handle.readline()
        image = np.flipud(np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width))
    rows, cols = np.nonzero(image == 0)
    world = np.array([compose(meta['world_from_map'], compose(
        meta['origin'], ((c + 0.5) * meta['resolution'], (r + 0.5) * meta['resolution'], 0.0)))[:2]
        for r, c in zip(rows, cols)]) if len(rows) else np.zeros((0, 2))
    db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])
    topic = dict(db.execute('select name, id from topics'))['/odom/ground_truth']
    path = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in (
        deserialize_message(d, Odometry) for (d,) in db.execute(
            'select data from messages where topic_id=? order by timestamp', (topic,)))])
    return score_map(reference, world, path[::10])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--map', default='nav_obstacle', help='reference map (robot_lab_maps)')
    parser.add_argument('--waypoints', default='-3.2,-3.8;3.3,-3.4',
                        help='x,y;x,y visited in order before returning to the start')
    parser.add_argument('--radius', type=float, default=0.45, help='route clearance from walls (m)')
    parser.add_argument('--speed', type=float, default=0.35)
    parser.add_argument('--min-speed', type=float, default=0.2,
                        help='slowest forward command the gait walks at')
    parser.add_argument('--turn-in-place', action='store_true',
                        help='rotate on the spot when the heading error exceeds 60 deg')
    parser.add_argument('--holonomic', action='store_true',
                        help='translate toward the route without turning (mecanum strafe, 4WS crab or '
                             'in_phase): the velocity points at the carrot and the start heading is held')
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--timeout', type=float, default=900.0, help='wall seconds for the whole drive')
    parser.add_argument('--warmup', type=float, default=10.0)
    parser.add_argument('--map-out')
    parser.add_argument('--rescore', nargs=2, metavar=('PGM', 'BAG'),
                        help='score a saved map (with its YAML) and the run bag; no robot')
    args = parser.parse_args()
    if args.rescore:
        print(json.dumps(rescore(args.map, *args.rescore), indent=1))
        return 0

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    reference = ReferenceMap(args.map)
    rclpy.init()
    node = Node('slam_mapping_check')
    state = {'truth': None, 'map': None}
    node.create_subscription(Odometry, '/odom/ground_truth',
                             lambda m: state.__setitem__('truth', m), qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', lambda m: state.__setitem__('map', m), latched)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    tf = Buffer()
    TransformListener(tf, node)

    def truth_pose():
        m = state['truth']
        p = m.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def slam_pose():
        try:
            t = tf.lookup_transform('map', args.base, Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    report = {'world': args.map, 'base': args.base, 'holonomic': args.holonomic}
    deadline = time.monotonic() + 300
    while time.monotonic() < deadline and (state['truth'] is None or slam_pose() is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if state['truth'] is None or slam_pose() is None:
        report['error'] = 'no truth or no map -> %s transform' % args.base
        print(json.dumps(report))
        return 1
    spin(args.warmup)
    start_truth, start_slam = truth_pose(), slam_pose()
    # World <- slam map, fixed at the start (both frames see the same pose).
    world_from_map = compose(start_truth, inverse(start_slam))

    # Waypoints move into open space: a mapping drive has no reason to brush
    # obstacles (the first Go2 drive got wedged 0.16 m from one).
    targets = [reference.open_point(*(float(v) for v in w.split(',')), args.radius + 0.35)
               for w in args.waypoints.split(';') if w.strip()]
    targets.append(start_truth[:2])
    report['waypoints'] = [[round(v, 2) for v in t] for t in targets]
    route, here = [], start_truth[:2]
    for target in targets:
        route += reference.route(here, target, args.radius)
        here = target
    report['route_points'] = len(route)
    report['route_m'] = round(sum(math.dist(a, b) for a, b in zip(route, route[1:])), 2)

    samples, index = [], 0
    end = time.monotonic() + args.timeout
    last_sample = 0.0
    progress = (time.monotonic(), truth_pose())
    report['backoffs'] = report['skipped_points'] = 0
    stuck_here = 0
    while rclpy.ok() and index < len(route) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        pose = truth_pose()
        # Stuck (pressed against something, or orbiting in place): back off
        # straight; after three tries at the same carrot, skip ahead.
        if time.monotonic() - progress[0] > 6.0:
            if (math.dist(pose[:2], progress[1][:2]) < 0.15
                    and abs(wrap(pose[2] - progress[1][2])) < math.radians(30)):
                stuck_here += 1
                report['backoffs'] += 1
                back = Twist()
                if args.holonomic:
                    carrot = route[index]
                    away = wrap(math.atan2(carrot[1] - pose[1], carrot[0] - pose[0]) - pose[2] + math.pi)
                    back.linear.x, back.linear.y = 0.2 * math.cos(away), 0.2 * math.sin(away)
                else:
                    back.linear.x = -0.2
                for _ in range(40):
                    cmd.publish(back)
                    spin(0.05)
                if stuck_here >= 3:
                    skip = min(10, len(route) - 1 - index)
                    index += skip
                    report['skipped_points'] += skip
                    stuck_here = 0
                pose = truth_pose()
            else:
                stuck_here = 0
            progress = (time.monotonic(), pose)
        # Advance to the first route point at least 0.6 m ahead (pure pursuit carrot).
        while index < len(route) - 1 and math.dist(pose[:2], route[index]) < 0.6:
            index += 1
        if index == len(route) - 1 and math.dist(pose[:2], route[index]) < 0.3:
            index += 1
            break
        target = route[index]
        bearing = math.atan2(target[1] - pose[1], target[0] - pose[0])
        error = wrap(bearing - pose[2])
        twist = Twist()
        if args.holonomic:
            twist.linear.x = args.speed * math.cos(error)
            twist.linear.y = args.speed * math.sin(error)
            twist.angular.z = max(-0.3, min(0.3, wrap(start_truth[2] - pose[2])))
        elif args.turn_in_place and abs(error) > math.radians(60):
            twist.angular.z = math.copysign(0.8, error)
        else:
            twist.linear.x = max(args.min_speed, args.speed * max(0.0, math.cos(error)))
            twist.angular.z = max(-0.8, min(0.8, 1.5 * error))
        cmd.publish(twist)
        now = time.monotonic()
        if now - last_sample >= 0.5:
            estimate = slam_pose()
            if estimate is not None:
                samples.append((pose, compose(world_from_map, estimate)))
            last_sample = now
    cmd.publish(Twist())
    spin(1.0)
    for _ in range(20):
        cmd.publish(Twist())
        spin(0.05)
    report['completed_route'] = index >= len(route)
    report['drive_wall_s'] = round(args.timeout - max(0.0, end - time.monotonic()), 1)
    spin(5.0)  # let slam_toolbox publish its last map update

    if samples:
        errors = np.array([math.dist(t[:2], e[:2]) for t, e in samples])
        heading = np.array([abs(wrap(t[2] - e[2])) for t, e in samples])
        path = np.array([t[:2] for t, _ in samples])
        report['trajectory'] = {
            'samples': len(samples), 'driven_m': round(float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))), 2),
            'rmse_m': round(float(np.sqrt(np.mean(errors ** 2))), 3), 'max_m': round(float(errors.max()), 3),
            'final_m': round(float(errors[-1]), 3),
            'heading_rmse_deg': round(math.degrees(float(np.sqrt(np.mean(heading ** 2)))), 2)}
    grid = state['map']
    if grid is None or not samples:
        report['error'] = 'no slam map received' if grid is None else 'no slam poses sampled'
        print(json.dumps(report))
        return 1
    info = grid.info
    data = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
    oq = info.origin.orientation
    map_origin = (info.origin.position.x, info.origin.position.y, yaw_of(oq))
    rows, cols = np.nonzero(data >= 65)
    local = np.column_stack([(cols + 0.5) * info.resolution, (rows + 0.5) * info.resolution])
    world = []
    for lx, ly in local:
        p = compose(world_from_map, compose(map_origin, (lx, ly, 0.0)))
        world.append(p[:2])
    world = np.array(world) if world else np.zeros((0, 2))
    report['world_from_map'] = [round(v, 4) for v in world_from_map]
    report['map'] = score_map(reference, world, path)
    report['map'].update(slam_free_cells=int(np.sum((data >= 0) & (data < 25))),
                         resolution_m=round(float(info.resolution), 3))
    if args.map_out:
        image = np.full(data.shape, 205, dtype=np.uint8)
        image[(data >= 0) & (data < 25)] = 254
        image[data >= 65] = 0
        with open(args.map_out, 'wb') as handle:
            handle.write(b'P5\n%d %d\n255\n' % (info.width, info.height))
            handle.write(np.flipud(image).tobytes())
        with open(os.path.splitext(args.map_out)[0] + '.yaml', 'w') as handle:
            yaml.safe_dump({'image': os.path.basename(args.map_out), 'mode': 'trinary',
                            'resolution': float(info.resolution),
                            'origin': [float(map_origin[0]), float(map_origin[1]), float(map_origin[2])],
                            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
                            'world_from_map': [float(v) for v in world_from_map]}, handle)
    print(json.dumps(report, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
