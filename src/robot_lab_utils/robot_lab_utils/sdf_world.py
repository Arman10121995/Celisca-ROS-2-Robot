"""Static world geometry from Gazebo SDF worlds, for the other backends.

Gazebo reads its ``.world`` files natively; every other backend has to be
told what is in them.  Each backend used to carry its own partial SDF reader
and each had the same gaps - primitives ignored, ``model://`` includes left
unresolved, model/link poses dropped - so a box-built arena showed up empty.
This module is the one pure-Python reader: it walks world -> model -> link ->
collision/visual -> geometry, composes the full pose chain, resolves
includes, and returns backend-neutral shapes that a backend only has to
materialise.

It has no ROS or simulator dependency, so it can run on the ROS side, in a
build tool, or be handed across a process boundary as plain data.

Shape records::

    {"type": "box" | "sphere" | "cylinder" | "plane" | "mesh",
     "model": <top-level model name>,
     "position": [x, y, z],
     "orientation": [w, x, y, z],
     "size": box [sx, sy, sz] full extents | sphere [r] |
             cylinder [r, length] | plane [sx, sy] full extents,
     "mesh": <absolute path>, "scale": [sx, sy, sz]}      # meshes only
"""
import math
import os
import xml.etree.ElementTree as ET

_IDENTITY = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
_MAX_INCLUDE_DEPTH = 8


def _tag(element):
    return element.tag.rsplit("}", 1)[-1]


def _child(element, name):
    if element is None:
        return None
    for child in element:
        if _tag(child) == name:
            return child
    return None


def _children(element, name):
    if element is None:
        return []
    return [child for child in element if _tag(child) == name]


def _text(element, name, default=""):
    """Leaf text, compared to None: a childless element is falsy."""
    child = _child(element, name)
    if child is None or child.text is None:
        return default
    return child.text


def _floats(text, count, default):
    values = [float(v) for v in (text or "").split()]
    return (values + [default] * count)[:count]


def quaternion_from_rpy(roll, pitch, yaw):
    """(w, x, y, z) for SDF roll/pitch/yaw (extrinsic X-Y-Z)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def _rotate(vector, quaternion):
    w, x, y, z = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def compose(parent, child):
    """Compose two (position, quaternion) frames."""
    rotated = _rotate(child[0], parent[1])
    return (tuple(p + r for p, r in zip(parent[0], rotated)),
            _multiply(parent[1], child[1]))


def frame_of(element):
    """(position, quaternion) of an element's <pose>, identity when absent."""
    pose = _child(element, "pose")
    if pose is None:
        return _IDENTITY
    x, y, z, roll, pitch, yaw = _floats(pose.text, 6, 0.0)
    return ((x, y, z), quaternion_from_rpy(roll, pitch, yaw))


def uri_resolver(model_dirs=(), package_share=None):
    """Build a resolver for model://, package://, file:// and relative URIs.

    *model_dirs* are searched for ``model://`` names; *package_share* maps a
    package name to its share directory (``get_package_share_directory`` on
    the ROS side, a source-tree lookup in tools).
    """
    def resolve(uri, base_dir):
        uri = (uri or "").strip()
        if not uri:
            return ""
        if uri.startswith("model://"):
            rest = uri[len("model://"):]
            for directory in model_dirs:
                candidate = os.path.join(directory, rest)
                if os.path.exists(candidate):
                    return candidate
            return ""
        if uri.startswith("package://"):
            if package_share is None:
                return ""
            package, _, rest = uri[len("package://"):].partition("/")
            try:
                candidate = os.path.join(package_share(package), rest)
            except Exception:
                return ""
            return candidate if os.path.exists(candidate) else ""
        if uri.startswith("file://"):
            uri = uri[len("file://"):]
        if os.path.isabs(uri):
            return uri if os.path.exists(uri) else ""
        candidate = os.path.normpath(os.path.join(base_dir, uri))
        return candidate if os.path.exists(candidate) else ""
    return resolve


def _shape(geometry, frame, base_dir, resolve, model_name, skipped):
    position, orientation = frame
    record = {"model": model_name, "position": list(position),
              "orientation": list(orientation)}
    box = _child(geometry, "box")
    if box is not None:
        return dict(record, type="box", size=_floats(_text(box, "size"), 3, 1.0))
    sphere = _child(geometry, "sphere")
    if sphere is not None:
        return dict(record, type="sphere",
                    size=_floats(_text(sphere, "radius"), 1, 0.5))
    cylinder = _child(geometry, "cylinder")
    if cylinder is not None:
        return dict(record, type="cylinder",
                    size=[_floats(_text(cylinder, "radius"), 1, 0.5)[0],
                          _floats(_text(cylinder, "length"), 1, 1.0)[0]])
    plane = _child(geometry, "plane")
    if plane is not None:
        return dict(record, type="plane",
                    size=_floats(_text(plane, "size"), 2, 100.0))
    mesh = _child(geometry, "mesh")
    if mesh is not None:
        uri = _text(mesh, "uri")
        path = resolve(uri, base_dir)
        if not path:
            skipped.append("unresolved mesh %s" % uri.strip())
            return None
        return dict(record, type="mesh", mesh=path,
                    scale=_floats(_text(mesh, "scale"), 3, 1.0))
    for unsupported in ("heightmap", "polyline"):
        if _child(geometry, unsupported) is not None:
            skipped.append("unsupported geometry <%s>" % unsupported)
    return None


def _walk_model(model, frame, base_dir, resolve, shapes, skipped,
                model_name, depth=0):
    if depth > _MAX_INCLUDE_DEPTH:
        skipped.append("include nesting too deep")
        return
    frame = compose(frame, frame_of(model))

    include = _child(model, "include")
    if include is not None:
        uri = _text(include, "uri")
        directory = resolve(uri, base_dir)
        sdf_path = os.path.join(directory, "model.sdf") if directory else ""
        if not sdf_path or not os.path.isfile(sdf_path):
            skipped.append("unresolved include %s" % uri.strip())
        else:
            try:
                root = ET.parse(sdf_path).getroot()
            except ET.ParseError as exc:
                skipped.append("unparseable %s: %s" % (sdf_path, exc))
            else:
                include_frame = compose(frame, frame_of(include))
                for nested in _children(root, "model"):
                    _walk_model(nested, include_frame,
                                os.path.dirname(sdf_path), resolve, shapes,
                                skipped, model_name, depth + 1)

    for link in _children(model, "link"):
        link_frame = compose(frame, frame_of(link))
        # Collision geometry is what physics sees; visual keeps decorative
        # models visible when they declare no collision.
        sources = _children(link, "collision") or _children(link, "visual")
        for source in sources:
            geometry = _child(source, "geometry")
            if geometry is None:
                continue
            shape = _shape(geometry, compose(link_frame, frame_of(source)),
                           base_dir, resolve, model_name, skipped)
            if shape is not None:
                shapes.append(shape)

    for nested in _children(model, "model"):
        _walk_model(nested, frame, base_dir, resolve, shapes, skipped,
                    model_name, depth + 1)


def extract_static_shapes(world_path, resolve):
    """Return (shapes, skipped_notes) for every static shape in a world.

    Scripted ``<actor>`` elements are not static geometry and are reported in
    *skipped_notes* rather than dropped silently.
    """
    root = ET.parse(world_path).getroot()
    world = _child(root, "world")
    if world is None:
        world = root
    base_dir = os.path.dirname(os.path.abspath(world_path))
    shapes, skipped = [], []
    for model in _children(world, "model"):
        _walk_model(model, _IDENTITY, base_dir, resolve, shapes, skipped,
                    model.get("name", "model"))
    for include in _children(world, "include"):
        wrapper = ET.Element("model")
        wrapper.append(include)
        _walk_model(wrapper, _IDENTITY, base_dir, resolve, shapes, skipped,
                    _text(include, "name") or "include")
    for actor in _children(world, "actor"):
        skipped.append("actor '%s' (scripted motion, not static geometry)"
                       % actor.get("name", "?"))
    return shapes, skipped
