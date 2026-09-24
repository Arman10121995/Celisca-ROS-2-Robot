"""Mesh staging shared by the simulator backends.

Gazebo world and robot descriptions reference meshes in formats the physics
backends cannot read directly: MuJoCo reads only STL/OBJ/MSH, and PyBullet
cannot build a collision shape from Collada.  The vendored Gazebo model
library ships Collada exclusively, so without conversion every mesh-based
world renders empty in both backends.

This module owns the one conversion pipeline both backends use: resolve a
mesh URI to a real file, convert Collada to binary STL, cap oversized
meshes, and cache the result in a content-addressed directory so the work
happens once per asset.  It lives in robot_lab_utils because
robot_lab_mujoco and robot_lab_pybullet both need it and neither should
depend on the other.
"""

import hashlib
import os
import re
import shutil
import struct
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

def _mesh_staging_dir(robot_name, urdf_text):
    """Per-robot, content-addressed cache dir for converted meshes."""
    tag = hashlib.sha1((_CONVERTER_VERSION + urdf_text).encode("utf-8")).hexdigest()[:12]
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(robot_name or "robot"))
    path = os.path.join(
        tempfile.gettempdir(), "robot_lab_mujoco_meshes", safe_name, tag
    )
    os.makedirs(path, exist_ok=True)
    return path


def _resolve_mesh_source(uri, pkg_map, base_dir=""):
    """Resolve a URDF mesh filename to an absolute host path ('' if unknown).

    Handles package://, file://, absolute paths and — as a last resort —
    URIs relative to *base_dir* (the directory of the source xacro/URDF;
    retargeting a temp-file URDF would otherwise break them).
    """
    uri = (uri or "").strip()
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        pkg, _, rel = rest.partition("/")
        base = pkg_map.get(pkg, "")
        return os.path.join(base, rel) if base else ""
    if uri.startswith("file://"):
        return uri[len("file://"):]
    if os.path.isabs(uri):
        return uri
    if base_dir:
        # Relative URIs are relative to the package layout: the URDF often
        # sits in an 'urdf'/'xacro'/'xml' subdirectory next to 'meshes', so
        # walk up the ancestor chain and try the URI against each one.
        base = os.path.abspath(base_dir)
        for _ in range(8):
            candidate = os.path.join(base, uri)
            if os.path.isfile(candidate):
                return candidate
            parent = os.path.dirname(base)
            if parent == base:
                break
            base = parent
    return ""


# Bump when the converter's output changes, so meshes staged by an older
# version are not reused from the content-addressed cache.
_CONVERTER_VERSION = "3"


def _node_matrix(node, q):
    """4x4 local transform of a Collada <node> (transform elements in order)."""
    matrix = np.eye(4)
    for element in node:
        tag = element.tag.rsplit("}", 1)[-1]
        try:
            values = [float(v) for v in (element.text or "").split()]
        except ValueError:
            continue
        local = np.eye(4)
        if tag == "matrix" and len(values) == 16:
            local = np.array(values).reshape(4, 4)
        elif tag == "translate" and len(values) == 3:
            local[:3, 3] = values
        elif tag == "scale" and len(values) == 3:
            local[:3, :3] = np.diag(values)
        elif tag == "rotate" and len(values) == 4:
            axis = np.array(values[:3])
            norm = np.linalg.norm(axis)
            if norm < 1e-12:
                continue
            x, y, z = axis / norm
            angle = np.radians(values[3])
            c, s_, t = np.cos(angle), np.sin(angle), 1.0 - np.cos(angle)
            local[:3, :3] = [[t * x * x + c, t * x * y - s_ * z, t * x * z + s_ * y],
                             [t * x * y + s_ * z, t * y * y + c, t * y * z - s_ * x],
                             [t * x * z - s_ * y, t * y * z + s_ * x, t * z * z + c]]
        else:
            continue
        matrix = matrix @ local
    return matrix


def _geometry_instances(root, q):
    """``(geometry id, 4x4 world transform)`` for every instanced geometry.

    Transforms come from the <visual_scene> node hierarchy that <scene>
    instantiates.  Exporters such as Blender put a 90 degree rotation on the
    node rather than in the vertices; ignoring it showed Unitree legs and
    trunks rotated a quarter turn against their collision shapes, so each
    robot appeared twice, crossed.  Empty when the file has no scene.
    """
    scenes = {}
    for library in root.findall(q("library_visual_scenes")):
        for scene in library.findall(q("visual_scene")):
            scenes["#" + (scene.get("id") or "")] = scene
    chosen = None
    scene_ref = root.find(q("scene"))
    if scene_ref is not None:
        instance = scene_ref.find(q("instance_visual_scene"))
        if instance is not None:
            chosen = scenes.get(instance.get("url", ""))
    if chosen is None and scenes:
        chosen = next(iter(scenes.values()))
    if chosen is None:
        return []
    instances = []

    def walk(node, parent):
        world = parent @ _node_matrix(node, q)
        for inst in node.findall(q("instance_geometry")):
            instances.append((inst.get("url", "").lstrip("#"), world))
        for child in node.findall(q("node")):
            walk(child, world)

    for node in chosen.findall(q("node")):
        walk(node, np.eye(4))
    return instances


def _parse_collada_mesh(dae_path):
    """Minimal Collada reader -> (vertices Nx3, faces Mx3) in Z-up metres.

    Supports the triangle/polygon primitives a robot URDF references for
    display, placed by the file's scene-graph node transforms, then scaled
    by <unit> and rotated from the declared <up_axis> to Z-up.  Raises
    ValueError when the file contains no usable geometry (the caller then
    falls back to a placeholder box).
    """
    tree = ET.parse(dae_path)
    root = tree.getroot()
    match = re.match(r"\{(.+)\}COLLADA", root.tag)
    ns = match.group(1) if match else ""

    def _q(tag):
        return "{%s}%s" % (ns, tag) if ns else tag

    up_axis = "Y_UP"
    unit_metres = 1.0
    for asset in root.findall(_q("asset")):
        axis = asset.find(_q("up_axis"))
        if axis is not None and axis.text:
            up_axis = axis.text.strip()
        # <unit meter="0.01" name="centimeter"/> declares how many metres
        # one file unit is (Collada default 1.0).  AWS RoboMaker warehouse
        # models export in centimetres; ignoring this made every world mesh
        # 100x too large and the LiDAR/RGB-D geometry wrong (R8.1).
        unit = asset.find(_q("unit"))
        if unit is not None and unit.get("meter"):
            try:
                unit_metres = max(float(unit.get("meter")), 1e-9)
            except ValueError:
                pass

    geometries = {}
    for geometry in root.iter(_q("geometry")):
        mesh = geometry.find(_q("mesh"))
        if mesh is None:
            continue
        vertices, faces = _collada_mesh_triangles(mesh, _q)
        if faces:
            geometries[geometry.get("id") or ""] = (
                np.array(vertices, dtype=np.float64), faces)

    instances = [(gid, m) for gid, m in _geometry_instances(root, _q)
                 if gid in geometries]
    if not instances:
        # No scene graph (or it references nothing we parsed): every
        # geometry once, untransformed, as the reader always did.
        instances = [(gid, np.eye(4)) for gid in geometries]

    all_vertices, all_faces = [], []
    for gid, matrix in instances:
        vertices, faces = geometries[gid]
        base = sum(len(v) for v in all_vertices)
        all_vertices.append(vertices @ matrix[:3, :3].T + matrix[:3, 3])
        all_faces.extend((a + base, b + base, c + base) for a, b, c in faces)
    if not all_faces:
        raise ValueError("no triangle geometry found")
    verts = np.vstack(all_vertices) * unit_metres
    if up_axis == "Y_UP":
        verts = verts[:, [0, 2, 1]] * np.array([1.0, -1.0, 1.0])
    elif up_axis == "X_UP":
        verts = verts[:, [1, 0, 2]] * np.array([-1.0, 1.0, 1.0])
    tris = np.array(all_faces, dtype=np.int64)
    return verts, tris


def _collada_mesh_triangles(mesh, _q):
    """(vertices, faces) of one Collada <mesh>, polygons fanned to triangles."""
    vertices = []
    faces = []
    sources = {}
    for source in mesh.findall(_q("source")):
        arr = source.find(_q("float_array"))
        if arr is None or not arr.text or not source.get("id"):
            continue
        try:
            sources["#" + source.get("id")] = [
                float(v) for v in arr.text.split()
            ]
        except ValueError:
            continue
    vertex_position = {}
    for vtx in mesh.findall(_q("vertices")):
        for inp in vtx.findall(_q("input")):
            if inp.get("semantic") == "POSITION":
                vertex_position["#" + (vtx.get("id") or "")] = \
                    inp.get("source", "")

    for prim_tag in ("triangles", "polylist", "polygons"):
        for prim in mesh.findall(_q(prim_tag)):
            inputs = prim.findall(_q("input"))
            if not inputs:
                continue
            stride = max(int(inp.get("offset", 0)) for inp in inputs) + 1
            vertex_offset = None
            positions = None
            for inp in inputs:
                if inp.get("semantic") == "VERTEX":
                    vertex_offset = int(inp.get("offset", 0))
                    src = inp.get("source", "")
                    positions = sources.get(vertex_position.get(src, src))
            if vertex_offset is None or not positions:
                continue
            groups = []
            if prim_tag == "polygons":
                # One <p> per polygon.
                for p_elem in prim.findall(_q("p")):
                    try:
                        idx = [int(v) for v in (p_elem.text or "").split()]
                    except ValueError:
                        continue
                    groups.append(idx)
            else:
                p_elem = prim.find(_q("p"))
                if p_elem is None or not p_elem.text:
                    continue
                try:
                    idx = [int(v) for v in p_elem.text.split()]
                except ValueError:
                    continue
                counts = None
                vc = prim.find(_q("vcount"))
                if vc is not None and vc.text:
                    try:
                        counts = [int(v) for v in vc.text.split()]
                    except ValueError:
                        counts = None
                if not counts:
                    counts = [3] * (len(idx) // max(stride, 1))
                cursor = 0
                for count in counts:
                    if count < 3 or cursor + count * stride > len(idx):
                        cursor += max(count, 0) * stride
                        continue
                    groups.append(idx[cursor:cursor + count * stride])
                    cursor += count * stride
            for group in groups:
                corner_ids = group[vertex_offset::stride]
                if len(corner_ids) < 3:
                    continue
                base = len(vertices)
                for vi in corner_ids:
                    if 0 <= vi * 3 + 2 < len(positions):
                        vertices.append(positions[vi * 3:vi * 3 + 3])
                    else:
                        vertices.append([0.0, 0.0, 0.0])
                for k in range(1, len(corner_ids) - 1):
                    faces.append((base, base + k, base + k + 1))
    return vertices, faces


def _write_binary_stl(stl_path, vertices, faces):
    """Write a binary STL; facet normals are recomputed from the winding.

    The whole facet array is packed in one vectorised step: a per-facet
    Python loop costs minutes on the map meshes (millions of facets), which
    is what made a furniture world look like a hang.
    """
    tris = np.asarray(vertices, dtype=np.float64)[np.asarray(faces, dtype=np.int64)]
    if len(tris) == 0:
        facet_bytes = b""
    else:
        v1 = tris[:, 1] - tris[:, 0]
        v2 = tris[:, 2] - tris[:, 0]
        normals = np.cross(v1, v2)
        norm = np.linalg.norm(normals, axis=1)
        norm[norm == 0.0] = 1.0
        normals = normals / norm[:, None]
        # 50-byte facet record: normal(3f) 3 vertices(3f) attribute(H).
        records = np.zeros((len(tris), 50), dtype=np.uint8)
        floats = np.concatenate(
            [normals, tris.reshape(len(tris), 9)], axis=1).astype("<f4")
        records[:, :48] = floats.view(np.uint8).reshape(-1, 48)
        facet_bytes = records.tobytes()
    with open(stl_path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(tris)))
        fh.write(facet_bytes)


def _write_placeholder_stl(stl_path, size=0.025):
    """Small cube standing in for meshes MuJoCo cannot use."""
    s = size
    corners = np.array([
        [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
        [-s, -s, s], [s, -s, s], [s, s, s], [-s, s, s],
    ], dtype=np.float64)
    quads = [
        (0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
        (2, 3, 7, 6), (1, 2, 6, 5), (3, 0, 4, 7),
    ]
    faces = []
    for a, b, c, d in quads:
        faces.append((a, b, c))
        faces.append((a, c, d))
    _write_binary_stl(stl_path, corners, np.array(faces, dtype=np.int64))


_MUJOCO_MAX_STL_FACES = 180000  # MuJoCo decoder cap is 200000; keep headroom

# World/map meshes are the collision geometry of a whole floor plan and are
# two orders of magnitude heavier than a robot's meshes: MuJoCo builds a
# convex hull for every mesh at compile time and the other backends cook a
# collision BVH / mesh, so a 180k-facet furniture map takes minutes to load
# (it looks like a hang).  Map meshes are capped far below the decoder limit
# instead.
_WORLD_MAX_STL_FACES = 150000


def _binary_stl_face_count(path):
    """Face count of a binary STL from its header/size ('' unsafe)."""
    try:
        size = os.path.getsize(path)
    except OSError:
        return 0
    if size < 84:
        return 0
    return (size - 84) // 50


def _read_binary_stl(path):
    """Binary STL reader -> (vertices Nx3, faces Mx3)."""
    with open(path, "rb") as fh:
        fh.read(80)
        (count,) = struct.unpack("<I", fh.read(4))
        raw = fh.read(count * 50)
    data = np.frombuffer(raw, dtype=np.uint8)
    if len(data) < count * 50:
        raise ValueError("truncated binary STL")
    data = data.reshape(count, 50)
    floats = data[:, 12:48].copy().view("<f4").reshape(count, 3, 3)
    verts = floats.reshape(-1, 3).astype(np.float64)
    faces = np.arange(len(verts), dtype=np.int64).reshape(-1, 3)
    return verts, faces


def _cap_faces(vertices, faces, max_faces=_MUJOCO_MAX_STL_FACES):
    """Naive decimation keeping the model under MuJoCo's decoder cap."""
    if len(faces) <= max_faces:
        return vertices, faces
    step = int(np.ceil(len(faces) / max_faces))
    return vertices, faces[::step]


def _is_ascii_stl(path):
    """True when *path* is an ASCII STL (MuJoCo only decodes binary STL)."""
    try:
        with open(path, "rb") as fh:
            head = fh.read(512)
    except OSError:
        return False
    return head.lstrip().startswith(b"solid") and b"facet" in head


def _stage_stl(source, staged, notes, uri, max_faces=_MUJOCO_MAX_STL_FACES):
    """Stage one STL mesh (binary copy, ASCII/beyond-cap conversion)."""
    if (os.path.exists(staged) and not _is_ascii_stl(staged)
            and 0 < _binary_stl_face_count(staged) <= max_faces):
        return staged
    if _is_ascii_stl(source):
        try:
            verts, faces = _parse_ascii_stl(source)
            verts, faces = _cap_faces(verts, faces, max_faces)
            _write_binary_stl(staged, verts, faces)
            return staged
        except Exception as exc:
            notes.append("mesh '%s' not convertible (%s); placeholder used"
                         % (os.path.basename(uri), exc))
            return ""
    if _binary_stl_face_count(source) > max_faces:
        try:
            verts, faces = _read_binary_stl(source)
            verts, faces = _cap_faces(verts, faces, max_faces)
            _write_binary_stl(staged, verts, faces)
            notes.append("mesh '%s' decimated to %d faces (limit %d)"
                         % (os.path.basename(uri), len(faces), max_faces))
            return staged
        except Exception as exc:
            notes.append("mesh '%s' not convertible (%s); placeholder used"
                         % (os.path.basename(uri), exc))
            return ""
    try:
        shutil.copyfile(source, staged)
        return staged
    except OSError:
        return ""


def _parse_ascii_stl(path):
    """Minimal ASCII STL reader -> (vertices Nx3, faces Mx3)."""
    vertices = []
    faces = []
    current = []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            parts = line.split()
            if len(parts) >= 4 and parts[0] == "vertex":
                try:
                    current.append([float(parts[1]), float(parts[2]),
                                    float(parts[3])])
                except ValueError:
                    current = []
                    continue
                if len(current) == 3:
                    base = len(vertices)
                    vertices.extend(current)
                    faces.append((base, base + 1, base + 2))
                    current = []
            elif parts and parts[0] in ("endsolid", "solid"):
                current = []
    if not faces:
        raise ValueError("no facets found in ASCII STL")
    return np.array(vertices, dtype=np.float64), np.array(faces, dtype=np.int64)

# Public names (the implementations above keep their historical private
# spelling so the backends' own module-level aliases stay valid).
mesh_staging_dir = _mesh_staging_dir
resolve_mesh_source = _resolve_mesh_source
parse_collada_mesh = _parse_collada_mesh
write_binary_stl = _write_binary_stl
write_placeholder_stl = _write_placeholder_stl
binary_stl_face_count = _binary_stl_face_count
read_binary_stl = _read_binary_stl
cap_faces = _cap_faces
is_ascii_stl = _is_ascii_stl
stage_stl = _stage_stl
parse_ascii_stl = _parse_ascii_stl
MAX_STL_FACES = _MUJOCO_MAX_STL_FACES
load_indexed_mesh = None  # bound below, after its definition
WORLD_MAX_STL_FACES = _WORLD_MAX_STL_FACES


def stage_mesh_file(source, cache_dir, stem=None,
                    max_faces=_MUJOCO_MAX_STL_FACES):
    """Stage one mesh file into *cache_dir*, converting when needed.

    Returns the path of a file the physics backends can load, or '' when the
    source could not be resolved.  Collada is converted to binary STL; STL is
    normalised (ASCII -> binary, face cap); OBJ/MSH are passed through.
    *max_faces* is the facet budget — the backends pass
    :data:`WORLD_MAX_STL_FACES` for map meshes, whose collision cost dominates
    the load time, and keep the default for robot meshes.
    """
    if not source or not os.path.isfile(source):
        return ""
    if max_faces == _WORLD_MAX_STL_FACES:
        return stage_world_mesh(source)
    extension = os.path.splitext(source)[1].lower()
    if extension in (".obj", ".msh"):
        return source
    os.makedirs(cache_dir, exist_ok=True)
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_",
                  stem or os.path.splitext(os.path.basename(source))[0] or "mesh")
    # The facet budget is part of the cache key: a mesh staged for one budget
    # must not be reused for another.
    suffix = (".stl" if max_faces == _MUJOCO_MAX_STL_FACES
              else "_%df.stl" % max_faces)
    staged = os.path.join(cache_dir, stem + suffix)
    if extension == ".stl":
        return _stage_stl(source, staged, [], source, max_faces) or ""
    try:
        vertices, faces = _parse_collada_mesh(source)
        if len(faces) > max_faces:
            vertices, faces = _cap_faces(vertices, faces, max_faces)
        _write_binary_stl(staged, vertices, faces)
        return staged
    except Exception:
        return ""


def _load_indexed_mesh(source):
    """(vertices, faces) with shared vertices, without optional packages.

    trimesh reads Collada only through pycollada, which the system Python
    that runs the ROS nodes does not have: every Collada world (small_house,
    the warehouses, Celisca furniture) failed to stage and the MuJoCo
    spawner retried forever ("missing pip install pycollada") without ever
    opening.  Collada and STL go through this module's own readers.
    """
    extension = os.path.splitext(source)[1].lower()
    if extension == ".dae":
        vertices, faces = _parse_collada_mesh(source)
    elif extension == ".stl":
        vertices, faces = (_parse_ascii_stl(source) if _is_ascii_stl(source)
                           else _read_binary_stl(source))
    else:
        import trimesh
        mesh = trimesh.load(source, force='mesh', process=True)
        return np.asarray(mesh.vertices, dtype=float), np.asarray(mesh.faces, dtype=np.int64)
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=np.int64)
    # Weld coincident corners (STL and the Collada reader emit a triangle
    # soup) so the surface is connected for simplification.
    keys = np.round(vertices / 1e-6).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    welded = np.zeros((len(unique), 3))
    welded[inverse.ravel()] = vertices
    faces = inverse.ravel()[faces]
    keep = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2]))
    return welded, faces[keep]


def _cluster_simplify(vertices, faces, target_faces):
    """Vertex-clustering decimation to about *target_faces* triangles.

    Used when fast_simplification is not installed.  Vertices are merged on
    a grid whose cell grows until the budget is met; surfaces thinner than a
    cell collapse, but no triangle is discarded at random, so walls keep
    their extent and stay closed.
    """
    extent = float(np.max(np.ptp(vertices, axis=0))) if len(vertices) else 0.0
    if not extent:
        return vertices, faces
    cell = extent / 4096.0
    for _ in range(24):
        keys = np.floor(vertices / cell).astype(np.int64)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        inverse = inverse.ravel()
        mapped = inverse[faces]
        keep = ((mapped[:, 0] != mapped[:, 1]) & (mapped[:, 1] != mapped[:, 2])
                & (mapped[:, 0] != mapped[:, 2]))
        mapped = mapped[keep]
        _, first = np.unique(np.sort(mapped, axis=1), axis=0, return_index=True)
        mapped = mapped[np.sort(first)]
        if len(mapped) <= target_faces:
            sums = np.zeros((len(unique), 3))
            np.add.at(sums, inverse, vertices)
            counts = np.bincount(inverse, minlength=len(unique))[:, None]
            return sums / np.maximum(counts, 1), mapped
        cell *= 1.15  # small steps: land near the budget, keep detail
    return vertices, faces


def stage_world_mesh(source, mujoco_format=False):
    """Simplify connected surfaces, never sample and discard triangles.

    Non-manifold furniture may prevent reaching the requested face budget.
    Keep those remaining faces; MuJoCo uses OBJ above its STL decoder limit.
    All backends share this cache, so expensive conversion happens once.
    """
    import trimesh
    stat = os.stat(source)
    cache = mesh_staging_dir('world_surface',
        '%s:%s:%s:%s' % (os.path.realpath(source), stat.st_mtime_ns,
                          stat.st_size, _WORLD_MAX_STL_FACES))
    staged = os.path.join(cache, 'surface.stl')
    if not os.path.isfile(staged):
        if source.lower().endswith('.stl') and not _is_ascii_stl(source) \
                and _binary_stl_face_count(source) <= _WORLD_MAX_STL_FACES:
            return source
        vertices, faces = _load_indexed_mesh(source)
        if len(faces) > _WORLD_MAX_STL_FACES:
            try:
                import fast_simplification
            except ImportError:
                # Optional pip package; the ROS interpreter usually lacks it.
                fast_simplification = None
            if fast_simplification is None:
                vertices, faces = _cluster_simplify(vertices, faces,
                                                    _WORLD_MAX_STL_FACES)
            for _ in range(2 if fast_simplification is not None else 0):
                # Opposite-winding duplicates create non-manifold edges
                # that prevent the quadric simplifier collapsing a surface.
                _, indices = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
                faces = faces[np.sort(indices)]
                if len(faces) <= _WORLD_MAX_STL_FACES:
                    break
                vertices, faces = fast_simplification.simplify(
                    vertices, faces, target_count=_WORLD_MAX_STL_FACES,
                    # agg 7 thinned 0.1 m walls to 86% of their volume;
                    # 3 keeps closed surfaces closed at full volume.
                    agg=3)
        temporary = staged + '.%d.tmp' % os.getpid()
        _write_binary_stl(temporary, vertices, faces)
        os.replace(temporary, staged)
    if mujoco_format and _binary_stl_face_count(staged) > _MUJOCO_MAX_STL_FACES:
        obj = os.path.join(cache, 'surface.obj')
        if not os.path.isfile(obj):
            mesh = trimesh.load(staged, force='mesh', process=True)
            temporary = obj + '.%d.tmp' % os.getpid()
            mesh.export(temporary, file_type='obj', include_normals=False)
            os.replace(temporary, obj)
        return obj
    return staged


load_indexed_mesh = _load_indexed_mesh
