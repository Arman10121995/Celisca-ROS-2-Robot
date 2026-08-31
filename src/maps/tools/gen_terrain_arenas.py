#!/usr/bin/env python3
"""
P4.3 - Rough terrain / stairs / stepping-stone arena generator.

Generates three deterministic 3D arenas for legged/humanoid robots, each as a
Gazebo Classic SDF world plus a Nav2 occupancy map (and map.yaml) rasterized
from the exact same platform rectangles, so the world geometry and the
localization map always agree:

  1. terrain_rough            - a 20m x 20m outdoor floor with scattered
                                low-height raised platforms (the original
                                `outdoor_terrain` placeholder, promoted to an
                                actually-on-disk world).
  2. terrain_stairs           - an ascending staircase plus a descending ramp.
  3. terrain_stepping_stones  - a path of raised stepping-stone platforms with
                                pits between them.

All arenas are deterministic (fixed geometry, no meshes), which keeps them
reproducible for benchmarking. Platform rectangles are the single source of
truth: they appear in the SDF world AND are rasterized into the occupancy PGM.

Each arena is defined by:
  - a ground rectangle (min_x, min_y) .. (max_x, max_y)
  - a list of Box(center_x, center_y, half_x, half_y, z, height) static
    platforms. `z` is the base elevation, `height` the box vertical extent.
    The footprint rectangle (cx, cy, hx, hy) is what gets rasterized;
    z/height only affect the 3D world.

usage:
    python3 tools/gen_terrain_arenas.py [--out-dir src/maps/maps]
"""

from __future__ import annotations

import argparse
from pathlib import Path

WALL_THICKNESS = 0.20  # m
WALL_HEIGHT = 1.8      # m (fence height)
RESOLUTION = 0.05      # m per pixel
FREE = 254             # pgm byte for free space
OCCUPIED = 0           # pgm byte for occupied


class Box:
    """A static rectangular platform: center (x, y), half-extents (hx, hy)."""

    def __init__(self, cx, cy, hx, hy, z=0.0, height=WALL_HEIGHT):
        self.cx = cx
        self.cy = cy
        self.hx = hx
        self.hy = hy
        self.z = z
        self.height = height


def outer_walls(lo_x, lo_y, hi_x, hi_y, t=WALL_THICKNESS):
    cx = (lo_x + hi_x) / 2.0
    cy = (lo_y + hi_y) / 2.0
    hw_x = (hi_x - lo_x) / 2.0
    hw_y = (hi_y - lo_y) / 2.0
    return [
        Box(cx, lo_y - t / 2.0, hw_x + t, t / 2.0, 0.0, WALL_HEIGHT),
        Box(cx, hi_y + t / 2.0, hw_x + t, t / 2.0, 0.0, WALL_HEIGHT),
        Box(lo_x - t / 2.0, cy, t / 2.0, hw_y + t, 0.0, WALL_HEIGHT),
        Box(hi_x + t / 2.0, cy, t / 2.0, hw_y + t, 0.0, WALL_HEIGHT),
    ]


ARENAS = {}

# ---------------------------------------------------------------------------
# terrain_rough - a 20m x 20m outdoor floor (rough) with scattered low-height
# raised platforms; promotes the `outdoor_terrain` placeholder to a real world.
# ---------------------------------------------------------------------------
rough_lo = -10.0
rough_hi = 10.0
rough_platforms = [
    Box(-6.5, -6.5, 1.5, 1.5, 0.0, 0.40),
    Box(-6.5, 0.0, 1.2, 2.0, 0.0, 0.30),
    Box(-6.5, 6.5, 2.0, 1.2, 0.0, 0.50),
    Box(0.0, -6.5, 2.0, 1.2, 0.0, 0.35),
    Box(0.0, 0.0, 1.5, 1.5, 0.0, 0.25),
    Box(0.0, 6.5, 1.2, 2.0, 0.0, 0.45),
    Box(6.5, -6.5, 1.2, 2.0, 0.0, 0.40),
    Box(6.5, 0.0, 2.0, 1.2, 0.0, 0.30),
    Box(6.5, 6.5, 1.5, 1.5, 0.0, 0.40),
]
ARENAS["terrain_rough"] = {
    "min_xy": (rough_lo, rough_lo),
    "max_xy": (rough_hi, rough_hi),
    "boxes": outer_walls(rough_lo, rough_lo, rough_hi, rough_hi) + rough_platforms,
    "spawn": [-9.0, -9.0, 0.0],
    "goal": [9.0, 9.0, 0.0],
}


# ---------------------------------------------------------------------------
# terrain_stairs - a 12m x 16m arena with an ascending staircase (5 steps) on
# the west and a descending ramp on the east. Tests climb and recovery.
# ---------------------------------------------------------------------------
st_lo = -8.0
st_hi = 8.0
step_w = 2.2          # step half-width (x)
step_d = 0.55         # step half-depth (y)
step_h = 0.18         # step rise
step_x0 = -5.0        # first step center x
stairs = []
for i in range(5):
    z = i * step_h
    stairs.append(Box(step_x0 + i * 2 * step_d, -4.0, step_w, step_d, z, step_h))
# Descending ramp (a wedge approximated by 4 low steps on the east).
ramp = []
for i in range(4):
    z = (3 - i) * step_h
    ramp.append(Box(step_x0 + 9.0 + i * 2 * step_d, 4.0, step_w, step_d, z, step_h))
ARENAS["terrain_stairs"] = {
    "min_xy": (st_lo, st_lo),
    "max_xy": (st_hi, st_hi),
    "boxes": outer_walls(st_lo, st_lo, st_hi, st_hi) + stairs + ramp,
    "spawn": [-7.0, -5.0, 0.0],
    "goal": [7.0, 5.0, 0.0],
}


# ---------------------------------------------------------------------------
# terrain_stepping_stones - an 18m x 14m arena with a serpentine path of raised
# stepping-stone platforms (0.3m high) with pits between them.
# ---------------------------------------------------------------------------
ss_lo = -9.0
ss_hi = 9.0
stones = [
    Box(-6.5, -5.0, 0.7, 0.7, 0.0, 0.30),
    Box(-4.5, -5.0, 0.7, 0.7, 0.0, 0.30),
    Box(-2.5, -3.5, 0.7, 0.7, 0.0, 0.30),
    Box(-0.5, -2.5, 0.7, 0.7, 0.0, 0.30),
    Box(1.5, -2.5, 0.7, 0.7, 0.0, 0.30),
    Box(3.5, -3.5, 0.7, 0.7, 0.0, 0.30),
    Box(5.5, -2.5, 0.7, 0.7, 0.0, 0.30),
    Box(7.5, -1.5, 0.7, 0.7, 0.0, 0.30),
    Box(7.0, 0.5, 0.7, 0.7, 0.0, 0.30),
    Box(5.0, 1.5, 0.7, 0.7, 0.0, 0.30),
    Box(3.0, 1.5, 0.7, 0.7, 0.0, 0.30),
    Box(1.0, 2.5, 0.7, 0.7, 0.0, 0.30),
    Box(-1.0, 2.5, 0.7, 0.7, 0.0, 0.30),
    Box(-3.0, 1.5, 0.7, 0.7, 0.0, 0.30),
]
ARENAS["terrain_stepping_stones"] = {
    "min_xy": (ss_lo, -7.0),
    "max_xy": (ss_hi, 7.0),
    "boxes": outer_walls(ss_lo, -7.0, ss_hi, 7.0) + stones,
    "spawn": [-8.0, -6.0, 0.0],
    "goal": [8.0, 6.0, 0.0],
}



# ---------------------------------------------------------------------------
# Generation helpers (shared with the 2D nav arenas).
# ---------------------------------------------------------------------------

def build_world_sdf(name, min_xy, max_xy, boxes):
    lo_x, lo_y = min_xy
    hi_x, hi_y = max_xy
    sx = hi_x - lo_x
    sy = hi_y - lo_y

    def box_el(b, label):
        hx = b.hx
        hy = b.hy
        hm = b.height / 2.0
        cz = b.z + hm
        return f"""    <model name="{label}">
      <static>true</static>
      <pose>{b.cx:.3f} {b.cy:.3f} {cz:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{2*hx:.3f} {2*hy:.3f} {b.height:.3f}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{2*hx:.3f} {2*hy:.3f} {b.height:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.42 0.44 0.40 1</ambient>
            <diffuse>0.42 0.44 0.40 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""

    boxes_xml = "\n\n".join(
        box_el(b, f"platform_{i:02d}") for i, b in enumerate(boxes)
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
            <ambient>0.5 0.52 0.42 1</ambient>
            <diffuse>0.5 0.52 0.42 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{boxes_xml}

  </world>
</sdf>
"""



def build_occupancy(name, min_xy, max_xy, boxes):
    """Rasterize box footprints onto a PGM and return (pgm, map_yaml)."""
    lo_x, lo_y = min_xy
    hi_x, hi_y = max_xy
    margin = 1.0
    map_lo_x = lo_x - margin
    map_lo_y = lo_y - margin
    map_hi_x = hi_x + margin
    map_hi_y = hi_y + margin

    cols = int(round((map_hi_x - map_lo_x) / RESOLUTION))
    rows = int(round((map_hi_y - map_lo_y) / RESOLUTION))
    grid = [[FREE] * cols for _ in range(rows)]

    def world_to_pixel(wx, wy):
        c = int(round((wx - map_lo_x) / RESOLUTION))
        r = int(round((map_hi_y - wy) / RESOLUTION))
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

    header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
    body = bytearray()
    for r in range(rows):
        body.extend(bytes(grid[r]))
    pgm = header + bytes(body)

    origin_x = map_lo_x
    origin_y = map_lo_y
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

