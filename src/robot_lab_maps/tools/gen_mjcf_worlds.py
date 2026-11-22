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
* ``<heightmap>`` terrain, emitted as a real ``<asset><hfield>`` plus an
  ``<geom type="hfield">``.  The raster is re-encoded as PNG (MuJoCo reads
  only PNG) and the SDF ``<size>`` becomes MuJoCo's
  (radius_x, radius_y, elevation_x, elevation_y).  Both engines normalise the
  raster by its own maximum, so the surface matches the layout measured
  against Gazebo instead of being rescaled twice.

What is skipped (and reported)
------------------------------
* ``<actor>`` elements - scripted moving obstacles are not static geometry;
  the dynamics metadata in the registry describes them instead;
* ``<polyline>`` - no MJCF equivalent is emitted rather than emitting
  something that does not match the Gazebo world.

Usage::

    python3 gen_mjcf_worlds.py [--check] [--maps-dir DIR] [--out-dir DIR]

``--check`` regenerates into a temporary directory and fails when the
checked-in MJCF differs, which is what CI runs.
"""
import argparse
import hashlib
import math
import os
import sys
import tempfile
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
    extract_dynamic_bodies, extract_static_shapes, uri_resolver)

# Where model:// URIs are looked up, in order of preference.
_MODEL_DIRS = (
    os.path.join(_SRC_ROOT, "robot_lab_models", "models"),
)

_GEOM_RGBA = "0.55 0.57 0.6 1"
# Dedicated physics fixtures have their own generators.
SPECIALIZED_WORLDS = {"gait_slip_patch": "gait_slip_patch.py"}

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


def _hfield_name(hfields, shape):
    """Unique MJCF asset name for one heightmap shape record."""
    key = shape["heightmap"]
    if key not in hfields:
        stem = os.path.splitext(os.path.basename(key))[0]
        name = "".join(c if c.isalnum() or c == "_" else "_" for c in stem)
        name = name or "hfield"
        candidate, index = name, 1
        existing = {value[0] for value in hfields.values()}
        while candidate in existing:
            index += 1
            candidate = "%s_%d" % (name, index)
        hfields[key] = (candidate, key, [float(v) for v in shape["size"]])
    return hfields[key][0]


def _write_hfield_png(shape, asset_dir):
    """Write the raster as a PNG MuJoCo can load, and return its path.

    MuJoCo accepts only PNG heightfields, while SDF heightmaps are often PGM
    or a DEM-derived raster, so the image is re-encoded next to the generated
    MJCF.  The pixel values are preserved exactly - MuJoCo normalises by the
    raster maximum itself, which is the same rule measured in Gazebo, so
    re-encoding does not rescale the terrain.
    """
    from robot_lab_utils.heightfield import load_heightmap

    elevations = load_heightmap(shape["heightmap"])
    digest = hashlib.sha1(
        ("%s|%s" % (os.path.abspath(shape["heightmap"]), shape["size"]))
        .encode("utf-8")).hexdigest()[:8]
    stem = os.path.splitext(os.path.basename(shape["heightmap"]))[0]
    stem = "".join(c if c.isalnum() or c in "_-" else "_" for c in stem) or "hfield"
    target = os.path.join(asset_dir, "%s-%s.hfield.png" % (stem, digest))
    if not os.path.isfile(target):
        from PIL import Image

        Image.fromarray(elevations.astype("uint8"), "L").save(target)
    return target


def _geom(shape, meshes, hfields, asset_dir):
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
    elif kind == "heightmap":
        geom.update(type="hfield", hfield=_hfield_name(hfields, shape))
    else:
        scale = " ".join("%.6g" % value for value in shape["scale"])
        geom.update(type="mesh", mesh=_mesh_name(meshes, shape["mesh"], scale))
    return geom


def convert_world(world_path, name=None, asset_dir=None, *, asset_reference_dir=None):
    """Return (mjcf_text, skipped_notes) for one SDF ``.world`` file.

    *asset_dir* is where derived heightfield PNGs are written and defaults
    to the source SDF directory. *asset_reference_dir* makes PNG references
    relative to the final MJCF so a packaged world can be relocated.
    """
    name = name or os.path.splitext(os.path.basename(world_path))[0]
    if asset_dir is None:
        asset_dir = os.path.dirname(world_path)
    shapes, skipped = extract_static_shapes(world_path, _resolve_uri, exclude_dynamic=True)
    # Loose primitive objects (explicit <static>false</static> with a mass) are
    # free bodies, not scenery: a pickable cube must stay pickable.
    bodies = extract_dynamic_bodies(world_path)
    meshes = {}
    hfields = {}
    geoms = [_geom(shape, meshes, hfields, asset_dir) for shape in shapes]

    def png_reference(path):
        return os.path.relpath(path, asset_reference_dir) if asset_reference_dir else os.path.abspath(path)

    # Marine scenery can be inspected as static geometry, but a visual water
    # surface is never a solid lid. Other marine force adapters stay gated.
    source_world = ET.parse(world_path).getroot().find('world')
    marine = source_world is not None and any(
        plugin.get('name', '').endswith('::Buoyancy') for plugin in source_world.findall('plugin'))
    if marine:
        visual_only = {model.get('name') for model in source_world.findall('model')
                       if model.find('.//collision') is None}
        for shape, geom in zip(shapes, geoms):
            if shape.get('model') in visual_only:
                geom.update(contype='0', conaffinity='0', rgba='0.05 0.4 0.65 0.15')

    mujoco = ET.Element("mujoco", {"model": name})
    ET.SubElement(mujoco, "option", {"gravity": "0 0 -9.81"})
    ET.SubElement(mujoco, "compiler", {"angle": "radian"})

    textures = {}
    for index, (shape, geom) in enumerate(zip(shapes, geoms)):
        albedo = shape.get("albedo_map")
        if albedo and os.path.splitext(albedo)[1].lower() == ".png":
            material = textures.setdefault(albedo, "sdf_plane_%d" % index)
            geom.update(material=material, name="visual_sdf_plane_%d" % index, contype="0", conaffinity="0")
            geom.pop("rgba", None)

    if meshes or hfields or textures:
        asset = ET.SubElement(mujoco, "asset")
        for path, material in textures.items():
            ET.SubElement(asset, "texture", {"name": material, "type": "2d", "file": png_reference(path)})
            ET.SubElement(asset, "material", {"name": material, "texture": material})
        for mesh_name, path, scale in sorted(meshes.values()):
            ET.SubElement(asset, "mesh",
                          {"name": mesh_name, "file": path, "scale": scale})
        for shape in shapes:
            if shape["type"] != "heightmap":
                continue
            hfield_name = hfields[shape["heightmap"]][0]
            try:
                png = _write_hfield_png(shape, asset_dir)
            except Exception as exc:
                skipped.append("hfield %s not generated: %s" % (hfield_name, exc))
                continue
            size = shape["size"]
            # MuJoCo's hfield size is (radius_x, radius_y, elevation_x,
            # elevation_y).  The radii are the SDF extent/2 and the elevations
            # the SDF size_z; MuJoCo divides the raster by its own maximum,
            # which is how Gazebo scales it too.
            ET.SubElement(asset, "hfield", {
                "name": hfield_name, "file": png_reference(png),
                "size": "%.6g %.6g %.6g %.6g" % (size[0] / 2.0, size[1] / 2.0,
                                                 size[2], size[2])})

    worldbody = ET.SubElement(mujoco, "worldbody")
    has_plane = any(geom["type"] == "plane" for geom in geoms)
    if not has_plane and not marine:
        # Every world needs something to stand on; the SDF ground collision is
        # authoritative. This fallback is visual scenery only: a collidable
        # plane at the same z as a source ground box doubles every foot contact
        # in MuJoCo (measured 22 common vs 11 native BHL passive contacts).
        # A Celisca world's visible ground must match its 40 x 25 m SDF physics
        # floor, or the default 200 x 200 m plane dwarfs the building.
        ground_size = "22 13.75 0.1" if name.startswith("celisca_") else "100 100 0.1"
        ET.SubElement(worldbody, "geom",
                      {"type": "plane", "size": ground_size,
                       "contype": "0", "conaffinity": "0",
                       "rgba": "0.35 0.37 0.4 1"})
    for geom in geoms:
        ET.SubElement(worldbody, "geom", geom)
    for body in bodies:
        element = ET.SubElement(worldbody, "body", {
            "name": body["name"], "pos": "%.6g %.6g %.6g" % tuple(body["position"]),
            "quat": "%.6g %.6g %.6g %.6g" % tuple(body["orientation"])})
        ET.SubElement(element, "freejoint", {"name": body["name"] + "_free"})
        if body["inertia"]:
            ET.SubElement(element, "inertial", {
                "pos": "0 0 0", "mass": "%.6g" % body["mass"],
                "diaginertia": " ".join("%.6g" % v for v in body["inertia"])})
        volumes = [_primitive_volume(shape) for shape in body["shapes"]]
        total_volume = sum(volumes)
        for shape, volume in zip(body["shapes"], volumes):
            geom = _geom(shape, meshes, hfields, asset_dir)
            geom["rgba"] = "0.85 0.2 0.15 1"
            if not body["inertia"]:
                # With no explicit inertial element, the compiler infers
                # the COM and inertia from geoms with a uniform density.
                geom["mass"] = "%.12g" % (body["mass"] * volume / total_volume)
            if "mu" in shape:
                geom["friction"] = "%.6g 0.005 0.0001" % shape["mu"]
            ET.SubElement(element, "geom", geom)
    ET.SubElement(worldbody, "light",
                  {"diffuse": "0.8 0.8 0.8", "pos": "0 0 10",
                   "dir": "0 0 -1", "castshadow": "true"})
    ET.SubElement(worldbody, "light",
                  {"diffuse": "0.3 0.3 0.3", "pos": "0 -6 6", "dir": "0 1 -1"})

    _indent(mujoco)
    header = ("<!-- Generated by robot_lab_maps/tools/gen_mjcf_worlds.py from\n"
              "     maps/%s/worlds/%s.world - do not edit by hand. -->\n"
              % (name, name))
    if marine:
        header += '<!-- Static marine geometry only; native water/vehicle force adapters are not supplied by this conversion. -->\n'
    return header + ET.tostring(mujoco, encoding="unicode").rstrip() + "\n", skipped


def _primitive_volume(shape):
    size = shape["size"]
    if shape["type"] == "box":
        return math.prod(size)
    if shape["type"] == "sphere":
        return 4.0 / 3.0 * math.pi * size[0] ** 3
    return math.pi * size[0] ** 2 * size[1]


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


def world_signature(text, directory):
    """Compare world definitions and PNG contents independent of path layout."""
    root = ET.fromstring(text)
    for tag in ("hfield", "texture"):
        for asset in root.findall("asset/" + tag):
            file = asset.get("file")
            if file:
                with open(os.path.join(directory, file), "rb") as stream:
                    asset.set("file", hashlib.sha256(stream.read()).hexdigest())
    return ET.tostring(root)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maps-dir", default=os.path.join(_PACKAGE_ROOT, "maps"))
    parser.add_argument("--out-dir", default=os.path.join(_PACKAGE_ROOT, "mjcf"))
    parser.add_argument("--map", action="append", dest="selected",
                        help="convert a named map, repeat to select several")
    parser.add_argument("--check", action="store_true",
                        help="fail when the checked-in MJCF is out of date")
    args = parser.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    stale, written = [], 0
    for map_name in sorted(args.selected or os.listdir(args.maps_dir)):
        world_path = os.path.join(args.maps_dir, map_name, "worlds",
                                  map_name + ".world")
        if not os.path.isfile(world_path):
            if args.selected:
                parser.error("source world not found for " + map_name)
            continue
        if map_name in SPECIALIZED_WORLDS:
            print("%s uses %s" % (map_name, SPECIALIZED_WORLDS[map_name]))
            continue
        target = os.path.join(args.out_dir, map_name + ".xml")
        existing = ""
        if os.path.isfile(target):
            with open(target) as handle:
                existing = handle.read()
        if args.check:
            with tempfile.TemporaryDirectory() as scratch:
                generated, _ = convert_world(world_path, map_name, asset_dir=scratch)
                try:
                    up_to_date = existing and (
                        world_signature(existing, args.out_dir) == world_signature(generated, scratch))
                except (ET.ParseError, OSError):
                    up_to_date = False
            if not up_to_date:
                stale.append(map_name)
            continue
        mjcf, skipped = convert_world(world_path, map_name, asset_reference_dir=args.out_dir)
        if existing.rstrip() != mjcf.rstrip():
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
