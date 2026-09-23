"""Lowest collision point of a URDF at its rest pose, without a simulator.

A robot's root link is not uniformly at floor level: wheeled bases are
rooted at the footprint, quadrupeds at the trunk, humanoids at the pelvis.
Spawning every robot at a map's literal spawn height therefore put dog and
humanoid legs into the floor.  The PyBullet, MuJoCo and Isaac spawners lift
a robot from their own collision bounds after loading it; Gazebo spawns
through ``ros_gz_sim create``, so its launch needs the same number up front.

Collision primitives are exact; meshes (STL, OBJ, Collada) are read and
transformed vertex by vertex.  Pure Python with NumPy.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from .sdf_world import compose, quaternion_from_rpy
from .sim_frames import rotate, urdf_link_frames


def _floats(text, count, default=0.0):
    values = [float(v) for v in (text or "").split()]
    return (values + [default] * count)[:count]


def _origin(element):
    origin = element.find("origin")
    xyz = _floats(origin.get("xyz") if origin is not None else "", 3)
    rpy = _floats(origin.get("rpy") if origin is not None else "", 3)
    return tuple(xyz), quaternion_from_rpy(*rpy)


def _mesh_vertices(path):
    from . import mesh_assets
    extension = os.path.splitext(path)[1].lower()
    if extension == ".dae":
        vertices, _ = mesh_assets.parse_collada_mesh(path)
        return vertices
    if extension == ".stl":
        if mesh_assets.is_ascii_stl(path):
            vertices, _ = mesh_assets.parse_ascii_stl(path)
        else:
            vertices, _ = mesh_assets.read_binary_stl(path)
        return np.asarray(vertices, dtype=float)
    if extension == ".obj":
        rows = [line.split()[1:4] for line in open(path, errors="replace")
                if line.startswith("v ")]
        return np.asarray(rows, dtype=float)
    raise ValueError("unsupported mesh format %s" % extension)


def _local_points(geometry, resolve_mesh):
    """Points whose lowest one bounds the geometry from below, in its frame.

    Returns ``(points, radius)``: the lowest point is the lowest of *points*
    minus *radius* (spheres and cylinder rims are handled that way).
    """
    box = geometry.find("box")
    if box is not None:
        sx, sy, sz = (v / 2.0 for v in _floats(box.get("size"), 3, 0.0))
        return [(x, y, z) for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)], 0.0
    sphere = geometry.find("sphere")
    if sphere is not None:
        return [(0.0, 0.0, 0.0)], float(sphere.get("radius", 0.0))
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.get("radius", 0.0))
        half = float(cylinder.get("length", 0.0)) / 2.0
        rim = [(radius * math.cos(a), radius * math.sin(a), z)
               for a in np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
               for z in (-half, half)]
        return rim, 0.0
    mesh = geometry.find("mesh")
    if mesh is not None:
        path = resolve_mesh(mesh.get("filename", ""))
        if not path or not os.path.isfile(path):
            return [], 0.0
        scale = _floats(mesh.get("scale"), 3, 1.0) if mesh.get("scale") else [1.0] * 3
        return _mesh_vertices(path) * np.asarray(scale), 0.0
    return [], 0.0


def lowest_collision_z(urdf_text, resolve_mesh=lambda uri: uri):
    """Lowest z of any collision geometry in the root link's frame.

    Joints are at zero (the URDF rest pose), which is the pose every backend
    spawns.  Returns None when the description has no usable collision
    geometry.  *resolve_mesh* maps a mesh ``filename`` to a local path.
    """
    root = ET.fromstring(urdf_text)
    frames, _ = urdf_link_frames(urdf_text)
    lowest = None
    for link in root.findall("link"):
        link_frame = frames.get(link.get("name"))
        if link_frame is None:
            continue
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            try:
                points, radius = _local_points(geometry, resolve_mesh)
            except Exception:
                continue
            if not len(points):
                continue
            position, quaternion = compose(link_frame, _origin(collision))
            # Only the world-z row of the rotation matters: R[2, j] is the z
            # component of the rotated local axis j.
            z_axis = np.array([rotate(axis, quaternion)[2]
                               for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))])
            z = float(np.min(np.asarray(points, dtype=float) @ z_axis)) + position[2] - radius
            lowest = z if lowest is None else min(lowest, z)
    return lowest


def package_resolver(package_share, base_dir=""):
    """Mesh resolver for package://, file://, absolute and relative filenames.

    Relative filenames are looked up against *base_dir* (the description's
    directory) and its ancestors, as the other backends do.
    """
    def resolve(uri):
        uri = (uri or "").strip()
        if uri.startswith("package://"):
            package, _, rest = uri[len("package://"):].partition("/")
            try:
                return os.path.join(package_share(package), rest)
            except Exception:
                return ""
        if uri.startswith("file://"):
            return uri[len("file://"):]
        if os.path.isabs(uri) or not base_dir:
            return uri
        base = os.path.abspath(base_dir)
        for _ in range(8):
            candidate = os.path.join(base, uri)
            if os.path.isfile(candidate):
                return candidate
            base = os.path.dirname(base)
        return ""
    return resolve


def clear_spawn_z(spawn_z, lowest_z, clearance=0.002):
    """Spawn height that keeps the lowest collision point above z=0.

    Never lowers a configured spawn: maps and robot profiles may place a
    robot higher on purpose.
    """
    if lowest_z is None:
        return float(spawn_z)
    return max(float(spawn_z), clearance - float(lowest_z))
