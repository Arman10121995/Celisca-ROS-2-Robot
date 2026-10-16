"""Textured world meshes for the non-Gazebo renderers.

Gazebo draws a world model's visual meshes with their Collada materials:
the AWS RoboMaker house shows its wood floor and bedding.  MuJoCo, PyBullet
and Isaac drew the collision meshes in one grey, so a camera saw flat-shaded
geometry and visual odometry had nothing to track in them (R7.2,
2026-10-15).  This module reads a Collada visual mesh into one part per
material - triangle corners in Z-up metres, texture coordinates, the diffuse
image or colour - and stages each part as an OBJ with an MTL and a PNG, which
MuJoCo (mesh + material), PyBullet (OBJ/MTL) and Isaac (primvars:st and a
preview surface) can all draw.

It reads the Collada itself, like ``mesh_assets``: the ROS interpreter has
no pycollada.  Unsupported content (no triangles, missing images) degrades
to untextured parts rather than failing a world.
"""
import hashlib
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

from .mesh_assets import _node_matrix, mesh_staging_dir

# Bump when the staged output changes.
_TEXTURED_VERSION = "2"
_MAX_TEXTURE_PX = 512


def _namespace(root):
    match = re.match(r"\{(.+)\}COLLADA", root.tag)
    ns = match.group(1) if match else ""
    return lambda tag: "{%s}%s" % (ns, tag) if ns else tag


def _floats(text):
    try:
        return [float(v) for v in (text or "").split()]
    except ValueError:
        return []


def _images(root, q, base_dir):
    images = {}
    for library in root.findall(q("library_images")):
        for image in library.findall(q("image")):
            init = image.find(q("init_from"))
            if init is None:
                continue
            ref = init.find(q("ref"))   # Collada 1.5
            path = ((ref.text if ref is not None else init.text) or "").strip()
            if path.startswith("file://"):
                path = path[len("file://"):]
            if path:
                images[image.get("id", "")] = os.path.normpath(os.path.join(base_dir, path))
    return images


def _effects(root, q, images):
    """effect id -> (image path or None, rgba)."""
    effects = {}
    for library in root.findall(q("library_effects")):
        for effect in library.findall(q("effect")):
            profile = effect.find(q("profile_COMMON"))
            if profile is None:
                continue
            surfaces, samplers = {}, {}
            for param in profile.iter(q("newparam")):
                surface = param.find(q("surface"))
                if surface is not None:
                    init = surface.find(q("init_from"))
                    if init is not None and init.text:
                        surfaces[param.get("sid", "")] = init.text.strip()
                sampler = param.find(q("sampler2D"))
                if sampler is not None:
                    source = sampler.find(q("source"))
                    instance = sampler.find(q("instance_image"))   # Collada 1.5
                    if source is not None and source.text:
                        samplers[param.get("sid", "")] = ("surface", source.text.strip())
                    elif instance is not None:
                        samplers[param.get("sid", "")] = ("image", instance.get("url", "").lstrip("#"))
            image, rgba = None, [0.8, 0.8, 0.8, 1.0]
            diffuse = next(iter(profile.iter(q("diffuse"))), None)
            if diffuse is not None:
                texture = diffuse.find(q("texture"))
                color = diffuse.find(q("color"))
                if texture is not None:
                    key = texture.get("texture", "")
                    kind, target = samplers.get(key, ("image", key))
                    image_id = surfaces.get(target, target) if kind == "surface" else target
                    image = images.get(image_id)
                    rgba = [1.0, 1.0, 1.0, 1.0]
                elif color is not None:
                    values = _floats(color.text)
                    if len(values) >= 3:
                        rgba = (values + [1.0])[:4]
            effects[effect.get("id", "")] = (image, rgba)
    return effects


def _materials(root, q, effects):
    materials = {}
    for library in root.findall(q("library_materials")):
        for material in library.findall(q("material")):
            instance = material.find(q("instance_effect"))
            if instance is not None:
                materials[material.get("id", "")] = effects.get(
                    instance.get("url", "").lstrip("#"), (None, [0.8, 0.8, 0.8, 1.0]))
    return materials


def _primitive_corners(mesh, q):
    """[(material symbol, positions (n, 3), uv (n, 2) or None)] per primitive,
    polygons fanned to triangles, one row per triangle corner."""
    sources = {}
    for source in mesh.findall(q("source")):
        array = source.find(q("float_array"))
        technique = source.find(q("technique_common"))
        accessor = technique.find(q("accessor")) if technique is not None else None
        stride = int(accessor.get("stride", 3)) if accessor is not None else 3
        values = _floats(array.text if array is not None else "")
        if values and stride > 0:
            sources["#" + source.get("id", "")] = np.array(values[:len(values) // stride * stride]).reshape(
                -1, stride)
    vertex_inputs = {}
    for vertices in mesh.findall(q("vertices")):
        for inp in vertices.findall(q("input")):
            vertex_inputs.setdefault("#" + vertices.get("id", ""), {})[inp.get("semantic")] = inp.get("source", "")
    primitives = []
    for tag in ("triangles", "polylist", "polygons"):
        for prim in mesh.findall(q(tag)):
            inputs = prim.findall(q("input"))
            if not inputs:
                continue
            stride = max(int(inp.get("offset", 0)) for inp in inputs) + 1
            position_offset = uv_offset = None
            positions = uvs = None
            for inp in inputs:
                semantic, offset = inp.get("semantic"), int(inp.get("offset", 0))
                if semantic == "VERTEX":
                    position_offset = offset
                    linked = vertex_inputs.get(inp.get("source", ""), {})
                    positions = sources.get(linked.get("POSITION", inp.get("source", "")))
                    if "TEXCOORD" in linked:
                        uv_offset, uvs = offset, sources.get(linked["TEXCOORD"])
                elif semantic == "TEXCOORD" and (uvs is None or inp.get("set", "0") == "0"):
                    uv_offset, uvs = offset, sources.get(inp.get("source", ""))
            if position_offset is None or positions is None:
                continue
            if tag == "polygons":
                groups = [[int(v) for v in (p.text or "").split()] for p in prim.findall(q("p"))]
            else:
                p = prim.find(q("p"))
                indices = [int(v) for v in (p.text or "").split()] if p is not None else []
                counts_elem = prim.find(q("vcount"))
                counts = [int(v) for v in (counts_elem.text or "").split()] if counts_elem is not None \
                    else [3] * (len(indices) // stride)
                groups, cursor = [], 0
                for count in counts:
                    groups.append(indices[cursor:cursor + count * stride])
                    cursor += count * stride
            corner_positions, corner_uvs = [], []
            for group in groups:
                ids = group[position_offset::stride]
                uv_ids = group[uv_offset::stride] if uvs is not None else None
                for k in range(1, len(ids) - 1):
                    for corner in (0, k, k + 1):
                        index = ids[corner]
                        corner_positions.append(positions[index, :3] if index < len(positions) else (0, 0, 0))
                        if uv_ids is not None:
                            uv_index = uv_ids[corner]
                            corner_uvs.append(uvs[uv_index, :2] if uv_index < len(uvs) else (0, 0))
            if corner_positions:
                primitives.append((prim.get("material", ""), np.array(corner_positions, dtype=np.float64),
                                   np.array(corner_uvs, dtype=np.float64) if corner_uvs else None))
    return primitives


def parse_collada_parts(path):
    """One part per (image, colour): dict(positions (n, 3) in Z-up metres,
    one row per triangle corner; uv (n, 2) or None; image path or None;
    rgba).  Raises ValueError when the file has no triangles."""
    root = ET.parse(path).getroot()
    q = _namespace(root)
    up_axis, unit = "Y_UP", 1.0
    for asset in root.findall(q("asset")):
        axis = asset.find(q("up_axis"))
        if axis is not None and axis.text:
            up_axis = axis.text.strip()
        unit_elem = asset.find(q("unit"))
        if unit_elem is not None and unit_elem.get("meter"):
            try:
                unit = max(float(unit_elem.get("meter")), 1e-9)
            except ValueError:
                pass
    images = _images(root, q, os.path.dirname(os.path.abspath(path)))
    materials = _materials(root, q, _effects(root, q, images))
    geometries = {}
    for geometry in root.iter(q("geometry")):
        mesh = geometry.find(q("mesh"))
        if mesh is not None:
            geometries[geometry.get("id", "")] = _primitive_corners(mesh, q)

    instances = []
    scene = None
    scene_elem = root.find(q("scene"))
    scene_ref = scene_elem.find(q("instance_visual_scene")) if scene_elem is not None else None
    scenes = {s.get("id", ""): s for s in root.iter(q("visual_scene"))}
    if scene_ref is not None:
        scene = scenes.get(scene_ref.get("url", "").lstrip("#"))
    if scene is None and scenes:
        scene = next(iter(scenes.values()))

    def walk(node, parent):
        world = parent @ _node_matrix(node, q)
        for inst in node.findall(q("instance_geometry")):
            binding = {m.get("symbol", ""): m.get("target", "").lstrip("#")
                       for m in inst.iter(q("instance_material"))}
            instances.append((inst.get("url", "").lstrip("#"), world, binding))
        for child in node.findall(q("node")):
            walk(child, world)
    if scene is not None:
        for node in scene.findall(q("node")):
            walk(node, np.eye(4))
    if not instances:
        instances = [(gid, np.eye(4), {}) for gid in geometries]

    parts = {}
    for gid, matrix, binding in instances:
        for symbol, positions, uv in geometries.get(gid, []):
            image, rgba = materials.get(binding.get(symbol, symbol), (None, [0.8, 0.8, 0.8, 1.0]))
            if image is not None and not os.path.isfile(image):
                image, rgba = None, [0.8, 0.8, 0.8, 1.0]
            if uv is None:
                image = None
            key = (image, tuple(round(v, 4) for v in rgba))
            placed = positions @ matrix[:3, :3].T + matrix[:3, 3]
            part = parts.setdefault(key, {"positions": [], "uv": [], "image": image, "rgba": list(rgba)})
            part["positions"].append(placed)
            part["uv"].append(uv if image is not None else np.zeros((len(placed), 2)))
    if not parts:
        raise ValueError("no triangle geometry in %s" % path)
    result = []
    for part in parts.values():
        positions = np.vstack(part["positions"]) * unit
        if len(positions) < 6:
            continue   # MuJoCo needs four vertices; one triangle draws nothing useful
        if up_axis == "Y_UP":
            positions = positions[:, [0, 2, 1]] * np.array([1.0, -1.0, 1.0])
        elif up_axis == "X_UP":
            positions = positions[:, [1, 0, 2]] * np.array([-1.0, 1.0, 1.0])
        result.append({"positions": positions,
                       "uv": np.vstack(part["uv"]) if part["image"] else None,
                       "image": part["image"], "rgba": part["rgba"]})
    return result


def _write_obj(path, positions, uv, material_name=None, mtl_name=None):
    """Unindexed OBJ: corner k of triangle t is vertex 3t+k (+1)."""
    with open(path, "w") as stream:
        if mtl_name:
            stream.write("mtllib %s\n" % mtl_name)
        stream.write("".join("v %.6f %.6f %.6f\n" % tuple(p) for p in positions))
        if uv is not None:
            stream.write("".join("vt %.6f %.6f\n" % tuple(t) for t in uv))
        if material_name:
            stream.write("usemtl %s\n" % material_name)
        count = len(positions) // 3
        if uv is not None:
            stream.write("".join("f %d/%d %d/%d %d/%d\n" % (3*i+1, 3*i+1, 3*i+2, 3*i+2, 3*i+3, 3*i+3)
                                 for i in range(count)))
        else:
            stream.write("".join("f %d %d %d\n" % (3*i+1, 3*i+2, 3*i+3) for i in range(count)))


def stage_textured_parts(source, scale=(1.0, 1.0, 1.0)):
    """Stage a Collada visual mesh as textured OBJ parts (cached).

    Returns [dict(obj=path, texture=png path or None, rgba=[r, g, b, a])];
    each OBJ has an MTL beside it naming the texture (for PyBullet).
    Non-Collada sources return [] (the caller keeps its untextured mesh).
    """
    if not source.lower().endswith(".dae") or not os.path.isfile(source):
        return []
    stat = os.stat(source)
    cache = mesh_staging_dir("world_visual", "%s:%s:%s:%s:%s:%s" % (
        _TEXTURED_VERSION, os.path.realpath(source), stat.st_mtime_ns, stat.st_size,
        ",".join("%g" % s for s in scale), "parts"))
    index = os.path.join(cache, "parts.txt")
    if os.path.isfile(index):
        return _read_index(index, cache)
    from PIL import Image
    parts = parse_collada_parts(source)
    lines = []
    for number, part in enumerate(parts):
        positions = part["positions"] * np.asarray(scale, dtype=np.float64)
        stem = "part%03d" % number
        texture = None
        if part["image"]:
            texture = os.path.join(cache, stem + ".png")
            try:
                with Image.open(part["image"]) as image:
                    image = image.convert("RGB")
                    # A 320 x 240 camera needs no more; full-size AWS
                    # textures would cost MuJoCo 12 MB of model each.
                    image.thumbnail((_MAX_TEXTURE_PX, _MAX_TEXTURE_PX))
                    image.save(texture)
            except Exception:
                texture = None
        uv = part["uv"] if texture else None
        mtl = os.path.join(cache, stem + ".mtl")
        with open(mtl, "w") as stream:
            stream.write("newmtl %s\nKd %.4f %.4f %.4f\nd %.4f\n" % (
                stem, part["rgba"][0], part["rgba"][1], part["rgba"][2], part["rgba"][3]))
            if texture:
                stream.write("map_Kd %s\n" % os.path.basename(texture))
        _write_obj(os.path.join(cache, stem + ".obj"), positions, uv, stem, os.path.basename(mtl))
        lines.append("%s %s %s" % (stem, os.path.basename(texture) if texture else "-",
                                    " ".join("%.4f" % v for v in part["rgba"])))
    temporary = index + ".%d.tmp" % os.getpid()
    with open(temporary, "w") as stream:
        stream.write("\n".join(lines) + "\n")
    os.replace(temporary, index)
    return _read_index(index, cache)


def _read_index(index, cache):
    parts = []
    for line in open(index):
        fields = line.split()
        if len(fields) < 6:
            continue
        parts.append({"obj": os.path.join(cache, fields[0] + ".obj"),
                      "texture": None if fields[1] == "-" else os.path.join(cache, fields[1]),
                      "rgba": [float(v) for v in fields[2:6]]})
    return parts


def content_key(path):
    """Short, stable name fragment for a staged file."""
    return hashlib.sha1(os.path.realpath(path).encode()).hexdigest()[:10]
