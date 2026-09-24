#!/usr/bin/env python3
"""Generate a MuJoCo MJCF world for every Gazebo ``.world`` in this package.

The MuJoCo backend needs an MJCF description of the environment; without one
it silently fell back to ``mjcf/empty.xml``, so 23 of the 26 registered maps
appeared as a bare ground plane.  This tool derives the MJCF from the SDF
world that Gazebo already uses, so both backends describe the *same*
geometry and stay in sync by construction.

The SDF is read by ``robot_lab_utils/sdf_world.py``, the reader the PyBullet
and Isaac Sim backends use at runtime, so all three non-Gazebo backends see
one interpretation of each world.

What is converted
-----------------
* ``<box>``, ``<sphere>``, ``<cylinder>`` and ``<plane>`` collision geometry,
  with the full model -> link -> geometry pose chain composed into a single
  world pose (quaternion);
* ``<mesh>`` geometry, emitted as an ``<asset><mesh>`` entry pointing at the
  resolved file.  ``model://`` and ``package://`` URIs are resolved against
  the vendored model/package source directories.  MuJoCo cannot read
  Collada, so the spawner stages/converts world meshes at load time exactly
  as it already does for robot meshes;
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
import os
import sys
import xml.etree.ElementTree as ET

_HERE = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_HERE)
_SRC_ROOT = os.path.dirname(_PACKAGE_ROOT)

# The source-tree reader wins over an installed copy, so the generated files
# always reflect the reader in this checkout.
_UTILS_SOURCE = os.path.join(_SRC_ROOT, "robot_lab_utils")
if os.path.isdir(_UTILS_SOURCE) and _UTILS_SOURCE not in sys.path:
    sys.path.insert(0, _UTILS_SOURCE)

from robot_lab_utils.sdf_world import (  # noqa: E402
    extract_static_shapes, uri_resolver)

# Where model:// URIs are looked up, in order of preference.
_MODEL_DIRS = (
    os.path.join(_SRC_ROOT, "robot_lab_models", "models"),
)

_GEOM_RGBA = "0.55 0.57 0.6 1"

_resolve_in_source = uri_resolver(
    _MODEL_DIRS, lambda package: os.path.join(_SRC_ROOT, package))
_resolve_in_install_layout = uri_resolver(
    (), lambda package: os.path.join(_SRC_ROOT, package, "share", package))


def _resolve_uri(uri, base_dir):
    """Resolve a model://, package://, file:// or relative URI to a real path.

    ``package://`` is looked up in the source tree first, then in an
    installed share layout (``<package>/share/<package>/...``).
    """
    return (_resolve_in_source(uri, base_dir)
            or _resolve_in_install_layout(uri, base_dir))


def _mesh_name(meshes, path, scale):
    """Unique MJCF asset name for one (mesh file, scale) pair."""
    key = (path, scale)
    if key not in meshes:
        stem = os.path.splitext(os.path.basename(path))[0]
        name = "".join(c if c.isalnum() or c == "_" else "_" for c in stem)
        name = name or "mesh"
        candidate, index = name, 1
        existing = {value[0] for value in meshes.values()}
        while candidate in existing:
            index += 1
            candidate = "%s_%d" % (name, index)
        meshes[key] = (candidate, path, scale)
    return meshes[key][0]


def _geom(shape, meshes):
    """MJCF geom attributes for one shared-reader shape record.

    MJCF sizes are half extents where SDF sizes are full extents.
    """
    size = shape.get("size") or []
    geom = {
        "pos": "%.6g %.6g %.6g" % tuple(shape["position"]),
        "quat": "%.6g %.6g %.6g %.6g" % tuple(shape["orientation"]),
        "rgba": _GEOM_RGBA,
    }
    kind = shape["type"]
    if kind == "box":
        geom.update(type="box", size=" ".join(
            "%.6g" % (max(value, 1e-6) / 2.0) for value in size))
    elif kind == "sphere":
        geom.update(type="sphere", size="%.6g" % size[0])
    elif kind == "cylinder":
        geom.update(type="cylinder", size="%.6g %.6g" % (size[0], size[1] / 2.0))
    elif kind == "plane":
        geom.update(type="plane",
                    size="%.6g %.6g 0.1" % (size[0] / 2.0, size[1] / 2.0))
    else:
        scale = " ".join("%.6g" % value for value in shape["scale"])
        geom.update(type="mesh", mesh=_mesh_name(meshes, shape["mesh"], scale))
    return geom


def convert_world(world_path, name=None):
    """Return (mjcf_text, skipped_notes) for one SDF ``.world`` file."""
    name = name or os.path.splitext(os.path.basename(world_path))[0]
    shapes, skipped = extract_static_shapes(world_path, _resolve_uri)
    meshes = {}
    geoms = [_geom(shape, meshes) for shape in shapes]

    mujoco = ET.Element("mujoco", {"model": name})
    ET.SubElement(mujoco, "option", {"gravity": "0 0 -9.81"})
    ET.SubElement(mujoco, "compiler", {"angle": "radian"})

    if meshes:
        asset = ET.SubElement(mujoco, "asset")
        for mesh_name, path, scale in sorted(meshes.values()):
            ET.SubElement(asset, "mesh",
                          {"name": mesh_name, "file": path, "scale": scale})

    worldbody = ET.SubElement(mujoco, "worldbody")
    has_plane = any(geom["type"] == "plane" for geom in geoms)
    if not has_plane:
        # Every world needs something to stand on; box arenas model their
        # floor as a thin box, which is kept as well.  A Celisca world's
        # visible ground must match its 40 x 25 m SDF physics floor, or the
        # default 200 x 200 m plane dwarfs the building in the viewer.
        ground_size = "20 12.5 0.1" if name.startswith("celisca_") else "100 100 0.1"
        ET.SubElement(worldbody, "geom",
                      {"type": "plane", "size": ground_size,
                       "rgba": "0.35 0.37 0.4 1"})
    for geom in geoms:
        ET.SubElement(worldbody, "geom", geom)
    ET.SubElement(worldbody, "light",
                  {"diffuse": "0.8 0.8 0.8", "pos": "0 0 10",
                   "dir": "0 0 -1", "castshadow": "true"})
    ET.SubElement(worldbody, "light",
                  {"diffuse": "0.3 0.3 0.3", "pos": "0 -6 6", "dir": "0 1 -1"})

    _indent(mujoco)
    header = ("<!-- Generated by robot_lab_maps/tools/gen_mjcf_worlds.py from\n"
              "     maps/%s/worlds/%s.world - do not edit by hand. -->\n"
              % (name, name))
    return header + ET.tostring(mujoco, encoding="unicode") + "\n", skipped


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
