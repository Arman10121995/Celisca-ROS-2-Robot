#!/usr/bin/env python3
"""
P4.4 - 3D / aerial course generator.

Generates two deterministic 3D courses for indoor and outdoor aerial robot
navigation, each as a Gazebo Classic SDF world plus a Nav2 occupancy map
rasterized from the exact same obstacle footprints:

  1. aerial_course  - outdoor 100m x 100m slalom course (gate pylons + gantry).
                      Promotes the existing aerial_course cataloged placeholder.
  2. aerial_indoor  - indoor 40m x 40m multi-level course (raised deck,
                      mezzanine plates, vertical pylons).

usage:
    python3 tools/gen_aerial_arenas.py [--out-dir src/robot_lab_maps/maps]
"""

from __future__ import annotations

import argparse
from pathlib import Path

WALL_THICKNESS = 0.20
RESOLUTION = 0.05
FREE = 254
OCCUPIED = 0


class Box:
    def __init__(self, cx, cy, hx, hy, z=0.0, height=2.0):
        self.cx = cx
        self.cy = cy
        self.hx = hx
        self.hy = hy
        self.z = z
        self.height = height


ARENAS = {}

# --- aerial_course (outdoor slalom) -----------------------------------------
ac_lo = -50.0
ac_hi = 50.0
gate_pylons = []
for gx, gy in [(-30.0, -20.0), (-15.0, -25.0), (0.0, -20.0), (15.0, -25.0),
               (30.0, -20.0), (20.0, 0.0), (5.0, 5.0), (-20.0, 10.0)]:
    gate_pylons.append(Box(gx, gy, 1.2, 1.2, 0.0, 3.0))
gantry = [
    Box(-5.0, 30.0, 0.5, 0.5, 0.0, 6.0),
    Box(5.0, 30.0, 0.5, 0.5, 0.0, 6.0),
    Box(0.0, 30.0, 5.5, 0.5, 6.0, 1.0),
]
ARENAS["aerial_course"] = {
    "min_xy": (ac_lo, ac_lo),
    "max_xy": (ac_hi, ac_hi),
    "boxes": gate_pylons + gantry,
    "spawn": [-40.0, -40.0, 0.0],
    "goal": [40.0, 40.0, 0.0],
}

# --- aerial_indoor (multi-level interior) -----------------------------------
ai_lo = -20.0
ai_hi = 20.0
indoor = [
    Box(0.0, 0.0, 14.0, 10.0, 2.0, 0.3),      # upper deck plate
    Box(-10.0, -6.0, 0.4, 0.4, 0.0, 2.0),      # deck columns
    Box(10.0, -6.0, 0.4, 0.4, 0.0, 2.0),
    Box(-10.0, 6.0, 0.4, 0.4, 0.0, 2.0),
    Box(10.0, 6.0, 0.4, 0.4, 0.0, 2.0),
    Box(-14.0, 0.0, 0.5, 0.5, 0.0, 3.0),      # doorway pylons
    Box(14.0, 0.0, 0.5, 0.5, 0.0, 3.0),
    Box(0.0, -10.0, 0.5, 0.5, 0.0, 3.0),
    Box(0.0, 10.0, 0.5, 0.5, 0.0, 3.0),
    Box(-18.0, 14.0, 2.0, 4.0, 1.2, 0.3),     # mezzanine plates
    Box(18.0, -14.0, 2.0, 4.0, 1.2, 0.3),
    Box(-8.0, -8.0, 0.6, 0.6, 0.0, 2.5),      # slalom pylons
    Box(-4.0, 8.0, 0.6, 0.6, 0.0, 2.5),
    Box(4.0, -8.0, 0.6, 0.6, 0.0, 2.5),
    Box(8.0, 8.0, 0.6, 0.6, 0.0, 2.5),
]
ARENAS["aerial_indoor"] = {
    "min_xy": (ai_lo, ai_lo),
    "max_xy": (ai_hi, ai_hi),
    "boxes": indoor,
    "spawn": [-18.0, -18.0, 0.0],
    "goal": [18.0, 18.0, 0.0],
}


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
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{2*hx:.3f} {2*hy:.3f} {b.height:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.42 0.46 1</ambient>
            <diffuse>0.4 0.42 0.46 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    boxes_xml = "\n\n".join(
        box_el(b, f"obstacle_{i:02d}") for i, b in enumerate(boxes)
    )
    return f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.7\">
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
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx + 4:.1f} {sy + 4:.1f} 0.1</size></box>
          </geometry>
        </visual>
      </link>
    </model>

{boxes_xml}

  </world>
</sdf>
"""


def build_occupancy(name, min_xy, max_xy, boxes):
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

    def w2p(wx, wy):
        c = int(round((wx - map_lo_x) / RESOLUTION))
        r = int(round((map_hi_y - wy) / RESOLUTION))
        return c, r

    for b in boxes:
        c0, r0 = w2p(b.cx - b.hx - RESOLUTION, b.cy + b.hy + RESOLUTION)
        c1, r1 = w2p(b.cx + b.hx + RESOLUTION, b.cy - b.hy - RESOLUTION)
        c0 = max(0, min(cols - 1, c0)); c1 = max(0, min(cols - 1, c1))
        r0 = max(0, min(rows - 1, r0)); r1 = max(0, min(rows - 1, r1))
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                grid[r][c] = OCCUPIED

    header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
    body = bytearray()
    for r in range(rows):
        body.extend(bytes(grid[r]))
    pgm = header + bytes(body)
    map_yaml = (
        f"image: map.pgm\n"
        f"mode: trinary\n"
        f"resolution: {RESOLUTION:.3f}\n"
        f"origin: [{map_lo_x:.3f}, {map_lo_y:.3f}, 0.000000]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    return pgm, map_yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir",
                    default=str(Path(__file__).resolve().parent.parent / "maps"))
    args = ap.parse_args()
    out_dir = Path(args.out_dir)
    for name, spec in ARENAS.items():
        world_dir = out_dir / name / "worlds"
        map_dir = out_dir / name / "maps"
        world_dir.mkdir(parents=True, exist_ok=True)
        map_dir.mkdir(parents=True, exist_ok=True)
        boxes = list(spec["boxes"])
        (world_dir / f"{name}.world").write_text(
            build_world_sdf(name, spec["min_xy"], spec["max_xy"], boxes))
        pgm, map_yaml = build_occupancy(name, spec["min_xy"], spec["max_xy"], boxes)
        (map_dir / "map.pgm").write_bytes(pgm)
        (map_dir / "map.yaml").write_text(map_yaml)
        print(f"generated {name}: okay")


if __name__ == "__main__":
    main()
