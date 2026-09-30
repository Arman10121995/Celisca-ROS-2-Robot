#!/usr/bin/env python3
"""Resize the Celisca maps and worlds by a uniform factor.

One scanned building is published several ways, and all of them have to grow
together or the robot ends up in a different place in each backend:

* the Gazebo/MuJoCo/Isaac/PyBullet **worlds** carry the mesh with an explicit
  ``<scale>``, which sets the building's size in metres;
* each **map.yaml** gives the raster's resolution and origin, which together
  place the 2D occupancy grid in those same metres;
* the **spawn and initial_pose** in
  ``robot_lab_bringup/config/sim_maps.yaml`` are absolute world coordinates.

Growing only the mesh would leave the 2D map describing the old footprint;
growing only the map would point AMCL at geometry that is no longer there.
This tool multiplies all of them by the same factor.  The MJCF worlds are
generated from the SDF, so ``gen_mjcf_worlds.py`` is run afterwards.

Every value is written as ``<base> * factor`` against a recorded base rather
than than read from the file it is editing, so repeated runs cannot compound
rounding error.

Usage::

    python3 resize_celisca_maps.py --factor 1.2          # apply
    python3 resize_celisca_maps.py --factor 1.2 --check  # report only
"""
import argparse
import math
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT = os.path.dirname(_HERE)
_SRC_ROOT = os.path.dirname(_PACKAGE_ROOT)
MAPS_DIR = os.path.join(_PACKAGE_ROOT, "maps")
SIM_MAPS = os.path.join(_SRC_ROOT, "robot_lab_bringup", "config", "sim_maps.yaml")

FLOORS = (
    "celisca_floor_1",
    "celisca_floor_2",
    "celisca_floor_1_furniture",
    "celisca_floor_2_furniture",
    "celisca_f1_actor",
    "celisca_f2_actor",
)

BASE_WORLD_SCALE = 1.155
BASE_RESOLUTION = 0.01925000004
BASE_ORIGIN = (-19.173, -8.3930000385)
#: Spawn keys that are footprint coordinates and scale with the building.
#: ``z`` is deliberately absent: it is a drop height above the floor, and
#: scaling it would launch the robot into the ceiling.
SCALED_SPAWN_KEYS = ("x", "y")

#: The unscaled spawn/initial_pose every Celisca map starts from.  Substitution
#: matches these exact literals, which is what makes the tool idempotent: a
#: second run finds nothing left to match and changes nothing, instead of
#: scaling an already-scaled value again.
BASE_SPAWN = {"x": "0.0", "y": "1.1"}


def _fmt(value):
    """A plain decimal number: no exponent, no trailing float noise."""
    text = "%.10f" % value
    return text


def _sub(path, pattern, replacement, check, flags=re.M):
    """Rewrite *path* in place unless *check*; return True if it changed."""
    with open(path) as handle:
        text = handle.read()
    scaled = re.sub(pattern, replacement, text, flags=flags)
    if scaled == text:
        return False
    if not check:
        with open(path, "w") as handle:
            handle.write(scaled)
    return True


def scale_worlds(factor, check):
    """Rewrite the mesh <scale> in every Celisca .world."""
    target = BASE_WORLD_SCALE * factor
    changed = []
    for floor in FLOORS:
        world_dir = os.path.join(MAPS_DIR, floor, "worlds")
        if not os.path.isdir(world_dir):
            continue
        for name in sorted(os.listdir(world_dir)):
            if not name.endswith(".world"):
                continue
            path = os.path.join(world_dir, name)
            if _sub(path, r"<scale>[^<]*</scale>",
                    "<scale>%s %s %s</scale>" % (_fmt(target), _fmt(target),
                                                 _fmt(target)), check):
                changed.append(os.path.relpath(path, _SRC_ROOT))
    return changed


def scale_maps(factor, check):
    """Rewrite resolution and origin in every Celisca map.yaml."""
    resolution = BASE_RESOLUTION * factor
    ox, oy = BASE_ORIGIN[0] * factor, BASE_ORIGIN[1] * factor
    changed = []
    for floor in FLOORS:
        path = os.path.join(MAPS_DIR, floor, "maps", "map.yaml")
        if not os.path.isfile(path):
            continue
        if _sub(path, r"^resolution: .*$", "resolution: %s" % _fmt(resolution),
                check):
            _sub(path, r"^origin: .*$",
                 "origin: [%s, %s, 0]" % (_fmt(ox), _fmt(oy)), check)
            changed.append(os.path.relpath(path, _SRC_ROOT))
    return changed


def scale_spawns(factor, check):
    """Scale the absolute spawn and initial_pose of each Celisca map.

    Only x and y move: they place the robot in the footprint, so they scale
    with the building.  ``z`` is a drop height above that floor and ``yaw`` is
    an angle, so neither scales.
    """
    if not os.path.isfile(SIM_MAPS):
        return []
    with open(SIM_MAPS) as handle:
        text = handle.read()

    # Split into top-level map entries so a substitution can only touch the
    # floors this tool owns.
    parts = re.split(r"(?m)^(?=  [A-Za-z0-9_]+:[ \t]*$)", text)
    changed = []
    for index, part in enumerate(parts):
        name = re.match(r"  ([A-Za-z0-9_]+):", part)
        if not name or name.group(1) not in FLOORS:
            continue
        updated = part
        for key in SCALED_SPAWN_KEYS:
            base = BASE_SPAWN[key]
            # Match only the recorded base literal, so a second run finds
            # nothing to change instead of scaling an already-scaled value.
            updated = re.sub(
                r'(?m)^(\s+%s:\s*)"%s"(\s*)$' % (re.escape(key), re.escape(base)),
                lambda m, k=key: '%s"%s"%s' % (
                    m.group(1), _fmt(float(BASE_SPAWN[k]) * factor), m.group(2)),
                updated)
        if updated == part:
            continue
        parts[index] = updated
        changed.append("%s (%s)" % (os.path.relpath(SIM_MAPS, _SRC_ROOT),
                                     name.group(1)))
    if changed and not check:
        with open(SIM_MAPS, "w") as handle:
            handle.write("".join(parts))
    return changed


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Resize the Celisca maps and worlds by a uniform factor.")
    parser.add_argument("--factor", type=float, required=True,
                        help="linear scale to apply, e.g. 1.2 for 20%% bigger")
    parser.add_argument("--check", action="store_true",
                        help="report what would change without writing")
    args = parser.parse_args(argv)
    if args.factor <= 0 or not math.isfinite(args.factor):
        parser.error("--factor must be a positive finite number")

    print("Celisca scale factor %g -> world scale %s, resolution %s, origin [%s, %s]"
          % (args.factor, _fmt(BASE_WORLD_SCALE * args.factor),
             _fmt(BASE_RESOLUTION * args.factor),
             _fmt(BASE_ORIGIN[0] * args.factor), _fmt(BASE_ORIGIN[1] * args.factor)))
    changed = (scale_worlds(args.factor, args.check)
               + scale_maps(args.factor, args.check)
               + scale_spawns(args.factor, args.check))
    for path in changed:
        print("  %s %s" % ("would update" if args.check else "updated", path))
    if not changed:
        print("  already at this scale")
    if changed and not args.check:
        print("\nRegenerate the MuJoCo worlds so they match the SDF:")
        print("  python3 %s"
              % os.path.join("robot_lab_maps", "tools", "gen_mjcf_worlds.py"))
    return 0


if __name__ == "__main__":
    sys.exit(main())