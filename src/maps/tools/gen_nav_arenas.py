#!/usr/bin/env python3
"""
P4.2 - Deterministic navigation arena generator.

Generates, for each of the five P4.2 deterministic navigation arenas, the
pair of artifacts that must stay perfectly in sync:

  1. `maps/<arena>/worlds/<arena>.world` - a Gazebo Classic SDF world built
     exclusively from static box primitives (no external mesh dependencies),
     so every wall/obstacle is reproducible and deterministic.
  2. `maps/<arena>/maps/map.pgm` + `map.yaml` - a Nav2 occupancy map whose
     occupied pixels are derived from the *exact same* box rectangles used to
     build the world. This guarantees the world geometry and the localization
     map always agree.

All arenas live on a ground plane centered near the origin. Each arena is
defined by a ground-rectangle (min_x, min_y) .. (max_x, max_y) plus a list of
`Box(center_x, center_y, half_x, half_y)` static obstacles (walls/shelves).

usage:
    python3 tools/gen_nav_arenas.py [--out-dir src/maps/maps]
"""

from __future__ import annotations

import argparse
from pathlib import Path

WALL_THICKNESS = 0.20  # m, outer wall / interior wall thickness
WALL_HEIGHT = 1.8      # m
RESOLUTION = 0.05      # m per pixel
FREE = 254             # pgm byte for free space
OCCUPIED = 0           # pgm byte for occupied


class Box:
    """A static rectangular obstacle: center (x, y), half-extents (hx, hy)."""

    def __init__(self, cx, cy, hx, hy):
        self.cx = cx
        self.cy = cy
        self.hx = hx
        self.hy = hy


# ---------------------------------------------------------------------------
# Arena layout definitions
#
# Each arena is a dict: {name, min_xy, max_xy, boxes, spawn, goal}
#   min_xy / max_xy : ground-rectangle corners (outer walls sit on this rect)
#   boxes           : list of Box obstacles (outer walls + interior)
#   spawn / goal    : [x, y, yaw] reference poses
# ---------------------------------------------------------------------------

# Helper to build the four outer walls of a rectangle.
def outer_walls(lo_x, lo_y, hi_x, hi_y, t=WALL_THICKNESS):
    cx = (lo_x + hi_x) / 2.0
    cy = (lo_y + hi_y) / 2.0
    hw_x = (hi_x - lo_x) / 2.0
    hw_y = (hi_y - lo_y) / 2.0
    return [
        Box(cx, lo_y - t / 2.0, hw_x + t, t / 2.0),  # south
        Box(cx, hi_y + t / 2.0, hw_x + t, t / 2.0),  # north
        Box(lo_x - t / 2.0, cy, t / 2.0, hw_y + t),  # west
        Box(hi_x + t / 2.0, cy, t / 2.0, hw_y + t),  # east
    ]


ARENAS = {}

# ---------------------------------------------------------------------------
# nav_empty - a large open floor (12m x 12m) with only the boundary fence and
# slim reference posts. Mostly free space for straight-line planning.
# ---------------------------------------------------------------------------
ARENAS["nav_empty"] = {
    "min_xy": (-6.0, -6.0),
    "max_xy": (6.0, 6.0),
    "boxes": (
        outer_walls(-6.0, -6.0, 6.0, 6.0)
        + [
            Box(-4.5, -4.5, 0.10, 0.10),
            Box(4.5, 4.5, 0.10, 0.10),
            Box(-4.5, 4.5, 0.10, 0.10),
            Box(4.5, -4.5, 0.10, 0.10),
        ]
    ),
    "spawn": [-0.5, -0.5, 0.0],
    "goal": [0.0, 0.0, 0.0],
}

# ---------------------------------------------------------------------------
# nav_obstacle - a 17m x 17m floor with ~10 scattered box obstacles of varied
# size, forcing multi-waypoint avoidance planning.
# ---------------------------------------------------------------------------
nav_obstacle_lo = -8.5
nav_obstacle_hi = 8.5
ARENAS["nav_obstacle"] = {
    "min_xy": (nav_obstacle_lo, nav_obstacle_lo),
    "max_xy": (nav_obstacle_hi, nav_obstacle_hi),
    "boxes": (
        outer_walls(nav_obstacle_lo, nav_obstacle_lo, nav_obstacle_hi, nav_obstacle_hi)
        + [
            Box(-5.5, -5.5, 1.2, 0.8),
            Box(-5.5, 0.0, 0.8, 1.6),
            Box(-5.5, 5.5, 1.4, 0.7),
            Box(0.0, -5.5, 1.6, 0.9),
            Box(0.0, 0.0, 1.0, 1.0),
            Box(0.0, 5.5, 0.9, 1.5),
            Box(5.5, -5.5, 0.7, 1.3),
            Box(5.5, 0.0, 1.5, 0.8),
            Box(5.5, 5.5, 1.1, 1.1),
            Box(-2.5, -2.0, 0.5, 0.5),
            Box(2.5, 2.5, 0.5, 0.5),
            Box(2.5, -2.5, 0.45, 0.45),
        ]
    ),
    "spawn": [-7.0, -7.0, 0.0],
    "goal": [7.0, 7.0, 0.0],
}


# ---------------------------------------------------------------------------
# nav_maze - a 16m x 16m maze with an outer wall and interior segments forming
# a winding corridor with a single start (west) and single goal (east).
# ---------------------------------------------------------------------------
maze_lo = -8.0
maze_hi = 8.0
maze_cx = 0.0
maze_cy = 0.0
t = WALL_THICKNESS
ARENAS["nav_maze"] = {
    "min_xy": (maze_lo, maze_lo),
    "max_xy": (maze_hi, maze_hi),
    "boxes": (
        # Outer walls with gaps for the entrance (west) and exit (east).
        [
            Box(maze_cx, maze_lo - t / 2.0, maze_hi - maze_lo + t, t / 2.0),  # south full
            Box(maze_cx, maze_hi + t / 2.0, maze_hi - maze_lo + t, t / 2.0),  # north full
            Box(maze_lo - t / 2.0, maze_cy + 1.0, t / 2.0, maze_hi - maze_lo / 2.0),  # west, gap at bottom
            Box(maze_hi + t / 2.0, maze_cy - 1.0, t / 2.0, maze_hi - maze_lo / 2.0),  # east, gap at top
        ]
        + [
            Box(-6.0, -6.0, 2.2, t / 2.0),
            Box(-4.5, -3.5, t / 2.0, 2.8),
            Box(-4.5, 4.0, 2.0, t / 2.0),
            Box(-2.0, 0.0, t / 2.0, 4.5),
            Box(-2.0, 6.5, 2.5, t / 2.0),
            Box(0.5, -6.0, t / 2.0, 2.5),
            Box(0.5, 3.0, 3.0, t / 2.0),
            Box(3.5, 3.0, t / 2.0, 3.2),
            Box(3.5, -2.0, 3.0, t / 2.0),
            Box(6.0, -2.0, t / 2.0, 5.0),
            Box(6.0, 6.0, 3.0, t / 2.0),
        ]
    ),
    "spawn": [-7.0, -7.0, 0.0],
    "goal": [7.0, 7.0, 0.0],
}


# ---------------------------------------------------------------------------
# nav_narrow_passage - a 14m x 14m floor with successive barriers that have
# offset gaps, forcing zigzag navigation through narrow passages (~1m gaps).
# ---------------------------------------------------------------------------
np_lo = -7.0
np_hi = 7.0
t = WALL_THICKNESS
ARENAS["nav_narrow_passage"] = {
    "min_xy": (np_lo, np_lo),
    "max_xy": (np_hi, np_hi),
    "boxes": (
        outer_walls(np_lo, np_lo, np_hi, np_hi)
        + [
            # Four horizontal barriers, each with a different offset gap.
            Box(0.0, -4.5, 5.5, t / 2.0),   # gap on the +x side
            Box(0.0, -1.5, 5.5, t / 2.0),   # gap on the -x side
            Box(0.0, 1.5, 5.5, t / 2.0),    # gap on the +x side
            Box(0.0, 4.5, 5.5, t / 2.0),    # gap on the -x side
            # Vertical stub walls to narrow the gaps.
            Box(5.2, -4.5, t / 2.0, 0.55),
            Box(-5.2, -1.5, t / 2.0, 0.55),
            Box(5.2, 1.5, t / 2.0, 0.55),
            Box(-5.2, 4.5, t / 2.0, 0.55),
        ]
    ),
    "spawn": [-6.0, -6.0, 0.0],
    "goal": [6.0, 6.0, 0.0],
}

# ---------------------------------------------------------------------------
# nav_warehouse - a 18m x 18m warehouse with parallel shelf rows leaving
# driving aisles, plus pallet boxes.
# ---------------------------------------------------------------------------
wh_lo = -9.0
wh_hi = 9.0
shelf_half = 0.4            # shelf half-thickness
t = WALL_THICKNESS
ARENAS["nav_warehouse"] = {
    "min_xy": (wh_lo, wh_lo),
    "max_xy": (wh_hi, wh_hi),
    "boxes": (
        outer_walls(wh_lo, wh_lo, wh_hi, wh_hi)
        + [
            Box(0.0, -6.0, 6.5, shelf_half),
            Box(0.0, -2.8, 6.5, shelf_half),
            Box(0.0, 0.4, 6.5, shelf_half),
            Box(0.0, 3.6, 6.5, shelf_half),
            Box(0.0, 6.8, 6.5, shelf_half),
            Box(-3.0, -4.4, 0.6, 0.6),
            Box(3.0, -1.2, 0.5, 0.5),
            Box(-1.5, 2.0, 0.5, 0.5),
            Box(2.0, 5.2, 0.6, 0.6),
            Box(6.5, -6.0, t / 2.0, shelf_half),
            Box(-6.5, -2.8, t / 2.0, shelf_half),
            Box(6.5, 0.4, t / 2.0, shelf_half),
            Box(-6.5, 3.6, t / 2.0, shelf_half),
            Box(6.5, 6.8, t / 2.0, shelf_half),
        ]
    ),
    "spawn": [-8.0, -8.0, 0.0],
    "goal": [8.0, 8.0, 0.0],
}


# ---------------------------------------------------------------------------
# Generation helpers
# ---------------------------------------------------------------------------

def build_world_sdf(name, min_xy, max_xy, boxes):
    lo_x, lo_y = min_xy
    hi_x, hi_y = max_xy
    sx = hi_x - lo_x
    sy = hi_y - lo_y

    def box_el(b, label):
        hx = b.hx
        hy = b.hy
        return f"""    <model name="{label}">
      <static>true</static>
      <pose>{b.cx:.3f} {b.cy:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{2*hx:.3f} {2*hy:.3f} {WALL_HEIGHT:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{2*hx:.3f} {2*hy:.3f} {WALL_HEIGHT:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.45 0.42 0.38 1</ambient>
            <diffuse>0.45 0.42 0.38 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""

    boxes_xml = "\n\n".join(
        box_el(b, f"obstacle_{i:02d}") for i, b in enumerate(boxes)
    )

    return f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.01</max_step_size>
    </physics>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.3 -1.0</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{sx + 4:.1f} {sy + 4:.1f} 0.1</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>100</mu><mu2>50</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx + 4:.1f} {sy + 4:.1f} 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.55 0.55 0.5 1</ambient>
            <diffuse>0.55 0.55 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{boxes_xml}

  </world>
</sdf>
"""


def build_occupancy(name, min_xy, max_xy, boxes):
    """Rasterize boxes onto a PGM and return (pgm_bytes, map_yaml_text)."""
    lo_x, lo_y = min_xy
    hi_x, hi_y = max_xy

    # Add a small margin so the outer wall is fully inside the image.
    margin = 1.0
    map_lo_x = lo_x - margin
    map_lo_y = lo_y - margin
    map_hi_x = hi_x + margin
    map_hi_y = hi_y + margin

    cols = int(round((map_hi_x - map_lo_x) / RESOLUTION))
    rows = int(round((map_hi_y - map_lo_y) / RESOLUTION))

    # 2D grid, default free. Row 0 = north (max y).
    grid = [[FREE] * cols for _ in range(rows)]

    def world_to_pixel(wx, wy):
        c = int(round((wx - map_lo_x) / RESOLUTION))
        r = int(round((map_hi_y - wy) / RESOLUTION))  # flip y
        return c, r

    for b in boxes:
        c0, r0 = world_to_pixel(b.cx - b.hx - RESOLUTION, b.cy + b.hy + RESOLUTION)
        c1, r1 = world_to_pixel(b.cx + b.hx + RESOLUTION, b.cy - b.hy - RESOLUTION)
        c0 = max(0, min(cols - 1, c0))
        c1 = max(0, min(cols - 1, c1))
        r0 = max(0, min(rows - 1, r0))
        r1 = max(0, min(rows - 1, r1))
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                grid[r][c] = OCCUPIED

    # Compose PGM binary (P5) payload.
    header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
    body = bytearray()
    for r in range(rows):
        body.extend(bytes(grid[r]))
    pgm = header + bytes(body)

    origin_x = map_lo_x
    origin_y = map_lo_y  # nav2 origin is the lower-left (world min corner)
    map_yaml = (
        f"image: map.pgm\n"
        f"mode: trinary\n"
        f"resolution: {RESOLUTION:.3f}\n"
        f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0.000000]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    return pgm, map_yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=str(Path(__file__).resolve().parent.parent / "maps"))
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    for name, spec in ARENAS.items():
        world_dir = out_dir / name / "worlds"
        map_dir = out_dir / name / "maps"
        world_dir.mkdir(parents=True, exist_ok=True)
        map_dir.mkdir(parents=True, exist_ok=True)

        boxes = list(spec["boxes"])
        (world_dir / f"{name}.world").write_text(
            build_world_sdf(name, spec["min_xy"], spec["max_xy"], boxes)
        )

        pgm, map_yaml = build_occupancy(name, spec["min_xy"], spec["max_xy"], boxes)
        (map_dir / "map.pgm").write_bytes(pgm)
        (map_dir / "map.yaml").write_text(map_yaml)

        print(f"generated {name}: okay")


if __name__ == "__main__":
    main()

