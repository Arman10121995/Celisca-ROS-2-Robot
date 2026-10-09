"""Bounded 3D A* over canonical SDF collision surfaces and primitive solids.

Uses upstream trimesh proximity/containment and shared exact world/heightfield
conversion. Clearance is a declared spherical lab planning envelope. Unknown
geometry fails explicitly; this is static planning, not actor prediction.
"""
import heapq
import itertools
import math
from pathlib import Path
import time

import numpy as np


def rotation(q):
    w, x, y, z = np.asarray(q, dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class StaticScene:
    def __init__(self, world):
        import trimesh
        from ament_index_python.packages import get_package_share_directory
        from robot_lab_utils.mesh_assets import load_indexed_mesh
        from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver
        self.resource_stamps = {}
        def remember(path):
            path = Path(path).resolve()
            if path.is_file():
                self.resource_stamps[str(path)] = (path.stat().st_mtime_ns, path.stat().st_size)
        remember(world)
        resolve_upstream = uri_resolver(package_share=get_package_share_directory)
        def resolve(uri, base_dir):
            path = resolve_upstream(uri, base_dir)
            if path:
                remember(path)
                if Path(path).is_dir():
                    for dependency in [*Path(path).glob('*.sdf'), Path(path)/'model.config']:
                        remember(dependency)
            return path
        shapes, skipped = extract_static_shapes(world, resolve, collision_only=True)
        if skipped:
            raise ValueError('Incomplete aerial collision scene: '+'; '.join(skipped))
        self.shapes = []
        self.resources = list(self.resource_stamps)
        for shape in shapes:
            item = dict(shape, rotation=rotation(shape['orientation']))
            if shape['type'] in ('mesh', 'heightmap'):
                if shape['type'] == 'mesh':
                    vertices, faces = load_indexed_mesh(shape['mesh'])
                    vertices = vertices*np.asarray(shape['scale'])
                    self.resources.append(shape['mesh'])
                else:
                    from robot_lab_utils.heightfield import heightfield_grid
                    vertices, faces = heightfield_grid(shape)
                    # Shared heightfield_grid retains shape-local metres;
                    # apply the canonical SDF pose in clearance(), as meshes.
                    self.resources.append(shape['heightmap'])
                item['surface'] = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
                item['solid'] = shape['type'] == 'mesh' and item['surface'].is_watertight
                item['bounds'] = item['surface'].bounds
                # Build the actual triangle index now. Missing rtree is an
                # installation error, never a no-obstacle fallback.
                _ = item['surface'].triangles_tree
            elif shape['type'] not in ('box', 'sphere', 'cylinder', 'plane'):
                raise ValueError('Unsupported aerial collider: '+shape['type'])
            self.shapes.append(item)

    def clearance(self, position):
        distance = math.inf
        for shape in self.shapes:
            p = (np.asarray(position)-shape['position']) @ shape['rotation']
            kind = shape['type']
            if kind == 'sphere':
                value = np.linalg.norm(p)-shape['size'][0]
            elif kind == 'box':
                q = abs(p)-np.asarray(shape['size'])/2
                value = np.linalg.norm(np.maximum(q, 0))+min(float(np.max(q)), 0.)
            elif kind == 'cylinder':
                radius, length = shape['size']
                q = np.array([np.linalg.norm(p[:2])-radius, abs(p[2])-length/2])
                value = np.linalg.norm(np.maximum(q, 0))+min(float(np.max(q)), 0.)
            elif kind == 'plane':
                q = np.maximum(abs(p[:2])-np.asarray(shape['size'])/2, 0)
                value = math.sqrt(float(q @ q)+p[2]**2)
            else:
                lower, upper = shape['bounds']
                bound = np.linalg.norm(np.maximum(np.maximum(lower-p, p-upper), 0))
                if bound >= distance:
                    continue
                _, distances, _ = shape['surface'].nearest.on_surface([p])
                value = float(distances[0])
                if shape['solid'] and shape['surface'].contains([p])[0]:
                    value = -value
                elif kind == 'heightmap' and np.all(p[:2] >= lower[:2]) and np.all(p[:2] <= upper[:2]):
                    # Heightfields are solid below their authored surface.
                    # Use actual local triangles; an underground route must
                    # not appear free merely because terrain is open-sided.
                    hits, _, _ = shape['surface'].ray.intersects_location(
                        [[p[0], p[1], upper[2]+1.]], [[0., 0., -1.]], multiple_hits=False)
                    if len(hits) and p[2] < hits[0, 2]:
                        value = -value
            distance = min(distance, float(value))
        return distance

    def segment(self, start, end, radius, deadline, canceled):
        length = math.dist(start, end)
        a, b = np.asarray(start), np.asarray(end)
        travelled = 0.
        while True:
            if time.monotonic() > deadline or canceled():
                raise TimeoutError('Aerial planning deadline/cancellation')
            p = a if length == 0 else a+(b-a)*(travelled/length)
            clearance = self.clearance(p)-radius
            if clearance <= 1e-5:
                return False
            if travelled >= length:
                return True
            # Distance to a surface is 1-Lipschitz. Advance strictly less
            # than the available clearance so thin walls cannot be skipped.
            travelled = min(length, travelled+min(.25, .8*clearance))


def plan(scene, start, goal, radius=.6, resolution=.35, timeout=25., max_nodes=60000,
         canceled=lambda: False, min_altitude=-math.inf, max_altitude=math.inf):
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if start.shape != (3,) or goal.shape != (3,) or not np.all(np.isfinite([start, goal])):
        raise ValueError('Finite 3D start and goal required')
    if radius <= 0 or resolution <= 0 or timeout <= 0:
        raise ValueError('Positive planning envelope, resolution and budget required')
    if not min_altitude <= start[2] <= max_altitude or not min_altitude <= goal[2] <= max_altitude:
        raise ValueError('Start/goal altitude is outside the configured flight envelope')
    deadline = time.monotonic()+timeout
    if scene.clearance(start) <= radius or scene.clearance(goal) <= radius:
        raise ValueError('Start or goal is inside the spherical collision-clearance envelope')
    if scene.segment(start, goal, radius, deadline, canceled):
        return [start.tolist(), goal.tolist()], dict(expanded=0, method='direct checked segment', radius_m=radius)
    origin = start
    lower = np.minimum(start, goal)-[6., 6., 2.]
    upper = np.maximum(start, goal)+[6., 6., 3.]
    lower[2], upper[2] = max(lower[2], min_altitude), min(upper[2], max_altitude)
    moves = [np.array(v, dtype=int) for v in itertools.product((-1, 0, 1), repeat=3) if any(v)]
    first = (0, 0, 0)
    costs, parents, counter = {first: 0.}, {}, itertools.count()
    heap = [(math.dist(start, goal), next(counter), 0., first)]
    expanded = 0
    while heap:
        if time.monotonic() > deadline or canceled():
            raise TimeoutError('Aerial A* budget/cancellation')
        _, _, cost, cell = heapq.heappop(heap)
        if cost != costs[cell]:
            continue
        expanded += 1
        if expanded > max_nodes:
            raise TimeoutError('Aerial A* node budget exceeded')
        point = origin+resolution*np.array(cell)
        if math.dist(point, goal) <= 2*resolution and scene.segment(point, goal, radius, deadline, canceled):
            cells = [cell]
            while cell in parents:
                cell = parents[cell]
                cells.append(cell)
            route = [(origin+resolution*np.array(item)).tolist() for item in reversed(cells)]+[goal.tolist()]
            # Preserve checked edges; later shortcut segments use the same
            # continuous clearance test, rather than unchecked smoothing.
            simplified, index = [route[0]], 0
            try:
                while index < len(route)-1:
                    next_index = index+1
                    for candidate in range(len(route)-1, index, -1):
                        if scene.segment(route[index], route[candidate], radius, deadline, canceled):
                            next_index = candidate
                            break
                    simplified.append(route[next_index])
                    index = next_index
            except TimeoutError:
                if canceled():
                    raise
                # Every original A* edge was already checked. Shortcutting
                # is optional and must not discard a valid bounded result.
                simplified = route
            return simplified, dict(expanded=expanded, method='bounded 26-neighbor 3D A*',
                radius_m=radius, resolution_m=resolution, geometry_scope='static exact SDF surfaces; no actor prediction')
        for move in moves:
            neighbor = tuple(np.array(cell)+move)
            candidate = origin+resolution*np.array(neighbor)
            if np.any(candidate < lower) or np.any(candidate > upper):
                continue
            candidate_cost = cost+resolution*float(np.linalg.norm(move))
            if candidate_cost >= costs.get(neighbor, math.inf):
                continue
            if not scene.segment(point, candidate, radius, deadline, canceled):
                continue
            costs[neighbor], parents[neighbor] = candidate_cost, cell
            heapq.heappush(heap, (candidate_cost+math.dist(candidate, goal), next(counter), candidate_cost, neighbor))
    raise ValueError('No collision-free route within the bounded local 3D search volume')
