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
    tag = hashlib.sha1(urdf_text.encode("utf-8")).hexdigest()[:12]
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


def _parse_collada_mesh(dae_path):
    """Minimal Collada reader -> (vertices Nx3, faces Mx3) in Z-up order.

    Supports the triangle/polygon primitives a robot URDF references for
    display.  Raises ValueError when the file contains no usable geometry
    (the caller then falls back to a placeholder box).
    """
    tree = ET.parse(dae_path)
    root = tree.getroot()
    match = re.match(r"\{(.+)\}COLLADA", root.tag)
    ns = match.group(1) if match else ""

    def _q(tag):
        return "{%s}%s" % (ns, tag) if ns else tag

    up_axis = "Y_UP"
    for asset in root.findall(_q("asset")):
        axis = asset.find(_q("up_axis"))
        if axis is not None and axis.text:
            up_axis = axis.text.strip()

    vertices = []
    faces = []
    for geometry in root.iter(_q("geometry")):
        mesh = geometry.find(_q("mesh"))
        if mesh is None:
            continue
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

        for prim_tag in ("triangles", "polylist"):
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
                    group = idx[cursor:cursor + count * stride]
                    cursor += count * stride
                    corner_ids = group[vertex_offset::stride]
                    base = len(vertices)
                    for vi in corner_ids:
                        if 0 <= vi * 3 + 2 < len(positions):
                            vertices.append(positions[vi * 3:vi * 3 + 3])
                        else:
                            vertices.append([0.0, 0.0, 0.0])
                    for k in range(1, len(corner_ids) - 1):
                        faces.append((base, base + k, base + k + 1))

    if not faces:
        raise ValueError("no triangle geometry found")
    verts = np.array(vertices, dtype=np.float64)
    if up_axis == "Y_UP":
        verts = verts[:, [0, 2, 1]] * np.array([1.0, -1.0, 1.0])
    elif up_axis == "X_UP":
        verts = verts[:, [1, 0, 2]] * np.array([-1.0, 1.0, 1.0])
    tris = np.array(faces, dtype=np.int64)
    return verts, tris


def _write_binary_stl(stl_path, vertices, faces):
    """Write a binary STL; facet normals are recomputed from the winding."""
    tris = vertices[faces]
    v1 = tris[:, 1] - tris[:, 0]
    v2 = tris[:, 2] - tris[:, 0]
    normals = np.cross(v1, v2)
    norm = np.linalg.norm(normals, axis=1)
    norm[norm == 0.0] = 1.0
    normals = normals / norm[:, None]
    with open(stl_path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(faces)))
        for normal, tri in zip(normals, tris):
            fh.write(struct.pack("<3f", float(normal[0]), float(normal[1]),
                                 float(normal[2])))
            for vertex in tri:
                fh.write(struct.pack("<3f", float(vertex[0]), float(vertex[1]),
                                     float(vertex[2])))
            fh.write(struct.pack("<H", 0))


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


def _stage_stl(source, staged, notes, uri):
    """Stage one STL mesh (binary copy, ASCII/beyond-cap conversion)."""
    if (os.path.exists(staged) and not _is_ascii_stl(staged)
            and 0 < _binary_stl_face_count(staged) <= _MUJOCO_MAX_STL_FACES):
        return staged
    if _is_ascii_stl(source):
        try:
            verts, faces = _parse_ascii_stl(source)
            verts, faces = _cap_faces(verts, faces)
            _write_binary_stl(staged, verts, faces)
            return staged
        except Exception as exc:
            notes.append("mesh '%s' not convertible (%s); placeholder used"
                         % (os.path.basename(uri), exc))
            return ""
    if _binary_stl_face_count(source) > _MUJOCO_MAX_STL_FACES:
        try:
            verts, faces = _read_binary_stl(source)
            verts, faces = _cap_faces(verts, faces)
            _write_binary_stl(staged, verts, faces)
            notes.append("mesh '%s' decimated to %d faces (MuJoCo limit)"
                         % (os.path.basename(uri), len(faces)))
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


def stage_mesh_file(source, cache_dir, stem=None):
    """Stage one mesh file into *cache_dir*, converting when needed.

    Returns the path of a file the physics backends can load, or '' when the
    source could not be resolved.  Collada is converted to binary STL; STL is
    normalised (ASCII -> binary, face cap); OBJ/MSH are passed through.
    """
    if not source or not os.path.isfile(source):
        return ""
    extension = os.path.splitext(source)[1].lower()
    if extension in (".obj", ".msh"):
        return source
    os.makedirs(cache_dir, exist_ok=True)
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_",
                  stem or os.path.splitext(os.path.basename(source))[0] or "mesh")
    staged = os.path.join(cache_dir, stem + ".stl")
    if extension == ".stl":
        return _stage_stl(source, staged, [], source) or ""
    try:
        vertices, faces = _parse_collada_mesh(source)
        _write_binary_stl(staged, vertices, faces)
        return staged
    except Exception:
        return ""
