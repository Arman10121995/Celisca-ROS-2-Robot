#!/usr/bin/env python3
"""Generate a MuJoCo MJCF world for every Gazebo ``.world`` in this package.

The MuJoCo backend needs an MJCF description of the environment; without one
it silently fell back to ``mjcf/empty.xml``, so 23 of the 26 registered maps
appeared as a bare ground plane.  This tool derives the MJCF from the SDF
world that Gazebo already uses, so both backends describe the *same*
geometry and stay in sync by construction.

What is converted
-----------------
* ``<box>``, ``<sphere>``, ``<cylinder>`` and ``<plane>`` collision geometry,
  with the full model -> link -> geometry pose chain composed into a single
  world pose (quaternion);
* ``<mesh>`` geometry, emitted as an ``<asset><mesh>`` entry pointing at the
  resolved file.  ``model://`` and ``package://`` URIs are resolved against
  the vendored model/package share directories.  MuJoCo cannot read Collada,
  so the spawner stages/converts world meshes at load time exactly as it
  already does for robot meshes;
* ``<include><uri>model://...</uri></include>``, by recursing into the
  referenced ``model.sdf``.

What is skipped (and reported)
------------------------------
* ``<actor>`` elements - scripted moving obstacles are not static geometry;
  the dynamics metadata in the registry describes them instead;
* ``<heightmap>`` and ``<polyline>`` - no MJCF equivalent is emitted rather
  than emitting something that does not match the Gazebo world.

Usage::

    python3 gen_mjcf_worlds.py [--check] [--maps-dir DIR] [--out-dir DIR]

``--check`` regenerates into a temporary directory and fails when the
checked-in MJCF differs, which is what CI runs.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

_HERE = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_HERE)
_SRC_ROOT = os.path.dirname(_PACKAGE_ROOT)

# Where model:// URIs are looked up, in order of preference.
_MODEL_DIRS = (
    os.path.join(_SRC_ROOT, "robot_lab_models", "models"),
)


def _tag(element):
    return element.tag.rsplit("}", 1)[-1]


def _find(element, name):
    for child in element:
        if _tag(child) == name:
            return child
    return None


def _findall(element, name):
    return [child for child in element if _tag(child) == name]


def _text(element, name, default=""):
    """Text of a child element.

    An ElementTree element with no children is falsy, so `_find(x, "uri") or
    fallback` silently discards every leaf value - which is exactly what a
    <uri>/<size>/<radius> element is.  This helper compares against None.
    """
    child = _find(element, name) if element is not None else None
    if child is None or child.text is None:
        return default
    return child.text


def _floats(text, count, default=0.0):
    values = [float(v) for v in (text or "").split()] if text else []
    values += [default] * (count - len(values))
    return values[:count]


def _pose_of(element):
    """(x, y, z, roll, pitch, yaw) from an SDF <pose>, zeros when absent."""
    pose = _find(element, "pose") if element is not None else None
    if pose is None:
        return [0.0] * 6
    return _floats(pose.text, 6)


def _quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _rotate(vector, quaternion):
    w, x, y, z = quaternion
    vx, vy, vz = vector
    # v + 2 * q_vec x (q_vec x v + w * v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _compose(parent, child):
    """Compose two (position, quaternion) frames."""
    parent_position, parent_quaternion = parent
    child_position, child_quaternion = child
    rotated = _rotate(child_position, parent_quaternion)
    return (
        tuple(p + r for p, r in zip(parent_position, rotated)),
        _multiply(parent_quaternion, child_quaternion),
    )


def _frame_of(element):
    pose = _pose_of(element)
    return (tuple(pose[:3]), _quaternion(pose[3], pose[4], pose[5]))


def _resolve_uri(uri, base_dir):
    """Resolve a model://, package:// or relative mesh URI to a real path."""
    uri = (uri or "").strip()
    if not uri:
        return ""
    if uri.startswith("model://"):
        rest = uri[len("model://"):]
        for directory in _MODEL_DIRS:
            candidate = os.path.join(directory, rest)
            if os.path.exists(candidate):
                return candidate
        return ""
    if uri.startswith("package://"):
        package, _, rest = uri[len("package://"):].partition("/")
        candidate = os.path.join(_SRC_ROOT, package, rest)
        if os.path.exists(candidate):
            return candidate
        # Installed share layout (…/share/<package>/<rest>).
        candidate = os.path.join(_SRC_ROOT, package, "share", package, rest)
        return candidate if os.path.exists(candidate) else ""
    if uri.startswith("file://"):
        uri = uri[len("file://"):]
    if os.path.isabs(uri):
        return uri if os.path.exists(uri) else ""
    candidate = os.path.normpath(os.path.join(base_dir, uri))
    return candidate if os.path.exists(candidate) else ""


class _Collector:
    """Accumulates MJCF geoms and mesh assets while walking an SDF tree."""

    def __init__(self):
        self.geoms = []
        self.meshes = {}
        self.skipped = []

    def mesh_name(self, path, scale):
        key = (path, scale)
        if key not in self.meshes:
            stem = os.path.splitext(os.path.basename(path))[0]
            name = "".join(c if c.isalnum() or c == "_" else "_" for c in stem)
            name = name or "mesh"
            candidate, index = name, 1
            existing = {value[0] for value in self.meshes.values()}
            while candidate in existing:
                index += 1
                candidate = "%s_%d" % (name, index)
            self.meshes[key] = (candidate, path, scale)
        return self.meshes[key][0]

    def add_geometry(self, geometry, frame, base_dir, rgba):
        position, quaternion = frame
        common = {
            "pos": "%.6g %.6g %.6g" % position,
            "quat": "%.6g %.6g %.6g %.6g" % quaternion,
            "rgba": rgba,
        }
        box = _find(geometry, "box")
        if box is not None:
            size = _floats(_text(box, "size"), 3, 1.0)
            half = " ".join("%.6g" % (max(value, 1e-6) / 2.0) for value in size)
            self.geoms.append(dict(common, type="box", size=half))
            return
        sphere = _find(geometry, "sphere")
        if sphere is not None:
            radius = _floats(_text(sphere, "radius"), 1, 0.5)[0]
            self.geoms.append(dict(common, type="sphere", size="%.6g" % radius))
            return
        cylinder = _find(geometry, "cylinder")
        if cylinder is not None:
            radius = _floats(_text(cylinder, "radius"), 1, 0.5)[0]
            length = _floats(_text(cylinder, "length"), 1, 1.0)[0]
            self.geoms.append(dict(common, type="cylinder",
                                   size="%.6g %.6g" % (radius, length / 2.0)))
            return
        plane = _find(geometry, "plane")
        if plane is not None:
            size = _floats(_text(plane, "size"), 2, 100.0)
            self.geoms.append(dict(common, type="plane",
                                   size="%.6g %.6g 0.1" % (size[0] / 2.0, size[1] / 2.0)))
            return
        mesh = _find(geometry, "mesh")
        if mesh is not None:
            uri = _text(mesh, "uri")
            path = _resolve_uri(uri, base_dir)
            if not path:
                self.skipped.append("unresolved mesh %s" % uri.strip())
                return
            scale_values = _floats(_text(mesh, "scale"), 3, 1.0)
            scale = " ".join("%.6g" % value for value in scale_values)
            name = self.mesh_name(path, scale)
            self.geoms.append(dict(common, type="mesh", mesh=name))
            return
        for unsupported in ("heightmap", "polyline"):
            if _find(geometry, unsupported) is not None:
                self.skipped.append("unsupported geometry <%s>" % unsupported)
                return


def _walk_model(model, frame, base_dir, collector, depth=0):
    if depth > 8:  # pragma: no cover - defensive against cyclic includes
        collector.skipped.append("include nesting too deep")
        return
    frame = _compose(frame, _frame_of(model))

    include = _find(model, "include")
    if include is not None:
        uri = _text(include, "uri")
        directory = _resolve_uri(uri, base_dir)
        include_frame = _compose(frame, _frame_of(include))
        sdf_path = os.path.join(directory, "model.sdf") if directory else ""
        if not sdf_path or not os.path.isfile(sdf_path):
            collector.skipped.append("unresolved include %s" % uri.strip())
        else:
            try:
                root = ET.parse(sdf_path).getroot()
            except ET.ParseError as exc:
                collector.skipped.append("unparseable %s: %s" % (sdf_path, exc))
            else:
                for nested in _findall(root, "model"):
                    _walk_model(nested, include_frame, os.path.dirname(sdf_path),
                                collector, depth + 1)

    for link in _findall(model, "link"):
        link_frame = _compose(frame, _frame_of(link))
        # Prefer collision geometry (what the physics sees); fall back to
        # visual so decorative models still appear.
        sources = _findall(link, "collision") or _findall(link, "visual")
        for source in sources:
            geometry = _find(source, "geometry")
            if geometry is None:
                continue
            collector.add_geometry(
                geometry, _compose(link_frame, _frame_of(source)),
                base_dir, "0.55 0.57 0.6 1")

    for nested in _findall(model, "model"):
        _walk_model(nested, frame, base_dir, collector, depth + 1)


def convert_world(world_path, name=None):
    """Return (mjcf_text, skipped_notes) for one SDF ``.world`` file."""
    name = name or os.path.splitext(os.path.basename(world_path))[0]
    root = ET.parse(world_path).getroot()
    world = _find(root, "world") or root
    base_dir = os.path.dirname(os.path.abspath(world_path))

    collector = _Collector()
    identity = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    for model in _findall(world, "model"):
        _walk_model(model, identity, base_dir, collector)
    for include in _findall(world, "include"):
        wrapper = ET.Element("model")
        wrapper.append(include)
        _walk_model(wrapper, identity, base_dir, collector)
    for actor in _findall(world, "actor"):
        collector.skipped.append(
            "actor '%s' (scripted motion, not static geometry)"
            % actor.get("name", "?"))

    mujoco = ET.Element("mujoco", {"model": name})
    ET.SubElement(mujoco, "option", {"gravity": "0 0 -9.81"})
    ET.SubElement(mujoco, "compiler", {"angle": "radian"})

    if collector.meshes:
        asset = ET.SubElement(mujoco, "asset")
        for mesh_name, path, scale in sorted(collector.meshes.values()):
            ET.SubElement(asset, "mesh",
                          {"name": mesh_name, "file": path, "scale": scale})

    worldbody = ET.SubElement(mujoco, "worldbody")
    has_plane = any(geom["type"] == "plane" for geom in collector.geoms)
    if not has_plane:
        # Every world needs something to stand on; box arenas model their
        # floor as a thin box, which is kept as well.
        ET.SubElement(worldbody, "geom",
                      {"type": "plane", "size": "100 100 0.1",
                       "rgba": "0.35 0.37 0.4 1"})
    for geom in collector.geoms:
        ET.SubElement(worldbody, "geom",
                      {k: v for k, v in geom.items() if v is not None})
    ET.SubElement(worldbody, "light",
                  {"diffuse": "0.8 0.8 0.8", "pos": "0 0 10",
                   "dir": "0 0 -1", "castshadow": "true"})
    ET.SubElement(worldbody, "light",
                  {"diffuse": "0.3 0.3 0.3", "pos": "0 -6 6", "dir": "0 1 -1"})

    _indent(mujoco)
    header = ("<!-- Generated by robot_lab_maps/tools/gen_mjcf_worlds.py from\n"
              "     maps/%s/worlds/%s.world - do not edit by hand. -->\n"
              % (name, name))
    return header + ET.tostring(mujoco, encoding="unicode") + "\n", collector.skipped


def _indent(element, level=0):
    pad = "\n" + "  " * level
    if len(element):
        if not (element.text or "").strip():
            element.text = pad + "  "
        for child in element:
            _indent(child, level + 1)
        if not (element.tail or "").strip():
            element.tail = pad
        if not (element[-1].tail or "").strip():
            element[-1].tail = pad
    elif level and not (element.tail or "").strip():
        element.tail = pad


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maps-dir", default=os.path.join(_PACKAGE_ROOT, "maps"))
    parser.add_argument("--out-dir", default=os.path.join(_PACKAGE_ROOT, "mjcf"))
    parser.add_argument("--check", action="store_true",
                        help="fail when the checked-in MJCF is out of date")
    args = parser.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    stale, written = [], 0
    for map_name in sorted(os.listdir(args.maps_dir)):
        world_path = os.path.join(args.maps_dir, map_name, "worlds",
                                  map_name + ".world")
        if not os.path.isfile(world_path):
            continue
        mjcf, skipped = convert_world(world_path, map_name)
        target = os.path.join(args.out_dir, map_name + ".xml")
        existing = ""
        if os.path.isfile(target):
            with open(target) as handle:
                existing = handle.read()
        if args.check:
            if existing != mjcf:
                stale.append(map_name)
            continue
        if existing != mjcf:
            with open(target, "w") as handle:
                handle.write(mjcf)
            written += 1
        note = (" [%d skipped: %s]" % (len(skipped), "; ".join(skipped[:2]))
                if skipped else "")
        print("%-28s %d geoms, %d meshes%s"
              % (map_name, mjcf.count("<geom"), mjcf.count("<mesh "), note))

    if args.check:
        if stale:
            print("Out-of-date MJCF worlds: %s" % ", ".join(stale),
                  file=sys.stderr)
            return 1
        print("All MJCF worlds are up to date.")
        return 0
    print("Wrote/updated %d MJCF world(s) in %s" % (written, args.out_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
