#!/usr/bin/env python3
"""
P4.5 - Dynamic-obstacle and sensor-degradation variant generator.

Produces deterministic arenas that vary the two environment dimensions left
static in P4.2-P4.4:

  1. nav_dynamic          - a grid arena with scripted dynamic-obstacle actors
                           (moving boxes) on top of a static maze-like floor.
  2. nav_sensor_degraded  - an arena whose static walls deliberately occlude a
                           LIDAR (featuring sensor-degradation. The static
                           geometry is still rasterized into an occupancy map,
                           while the moving actors are excluded from the map
                           and instead documented as dynamics.

Both arenas expose a static occupancy map for the static geometry (so Nav2
localization/planning still works in the static frame) and declare
`dynamics.dynamic_obstacles` in the environment registry metadata.

usage:
    python3 tools/gen_dynamic_arenas.py [--out-dir src/maps/maps]
"""

from __future__ import annotations

import argparse
from pathlib import Path

WALL_THICKNESS = 0.20
RESOLUTION = 0.05
FREE = 254
OCCUPIED = 0
WALL_HEIGHT = 1.8


class Box:
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
        Box(cx, lo_y - t / 2.0, hw_x + t, t / 2.0),
        Box(cx, hi_y + t / 2.0, hw_x + t, t / 2.0),
        Box(lo_x - t / 2.0, cy, t / 2.0, hw_y + t),
        Box(hi_x + t / 2.0, cy, t / 2.0, hw_y + t),
    ]


ARENAS = {}

# --- nav_dynamic: 16m x 16m floor with a central cross wall and a scripted
#     moving obstacle actor crossing the arena periodically. ----------------
nd_lo = -8.0
nd_hi = 8.0
static_boxes = outer_walls(nd_lo, nd_lo, nd_hi, nd_hi) + [
    Box(0.0, 0.0, WALL_THICKNESS / 2.0, 5.0, 0.0, WALL_HEIGHT),
    Box(0.0, 0.0, 5.0, WALL_THICKNESS / 2.0, 0.0, WALL_HEIGHT),
    Box(-3.0, 3.0, 0.5, 0.5, 0.0, WALL_HEIGHT),
    Box(3.0, -3.0, 0.5, 0.5, 0.0, WALL_HEIGHT),
]
ARENAS["nav_dynamic"] = {
    "min_xy": (nd_lo, nd_lo),
    "max_xy": (nd_hi, nd_hi),
    "boxes": static_boxes,
    "actors": [
        {"name": "moving_box_1", "cx": -6.0, "cy": -3.0,
         "hx": 0.4, "hy": 0.4, "height": 1.0,
         "x": 12.0, "y": 0.0, "z": 0.0, "phase": 0.0, "period": 12.0},
        {"name": "moving_box_2", "cx": 6.0, "cy": 3.0,
         "hx": 0.4, "hy": 0.4, "height": 1.0,
         "x": -12.0, "y": 0.0, "z": 0.0, "phase": 6.0, "period": 12.0},
    ],
    "spawn": [-7.0, -7.0, 0.0],
    "goal": [7.0, 7.0, 0.0],
}

# --- nav_sensor_degraded: 16m x 16m with interior blind-corners (walls that
#     occlude a 2D LIDAR at 1m height), static geometry only (no actors). ---
sd_lo = -8.0
sd_hi = 8.0
sd_boxes = outer_walls(sd_lo, sd_lo, sd_hi, sd_hi) + [
    Box(-4.0, -4.0, 2.0, 0.4, 0.0, 3.0),
    Box(4.0, -4.0, 2.0, 0.4, 0.0, 3.0),
    Box(-4.0, 4.0, 2.0, 0.4, 0.0, 3.0),
    Box(4.0, 4.0, 2.0, 0.4, 0.0, 3.0),
    Box(0.0, 0.0, 0.6, 0.6, 0.0, 4.0),
    Box(-2.0, 0.0, 0.3, 2.0, 0.0, 3.5),
    Box(2.0, 0.0, 0.3, 2.0, 0.0, 3.5),
]
ARENAS["nav_sensor_degraded"] = {
    "min_xy": (sd_lo, sd_lo),
    "max_xy": (sd_hi, sd_hi),
    "boxes": sd_boxes,
    "actors": [],
    "spawn": [-7.0, -7.0, 0.0],
    "goal": [7.0, 7.0, 0.0],
}


def build_world_sdf(name, min_xy, max_xy, boxes, actors):
    lo_x, lo_y = min_xy
    hi_x, hi_y = max_xy
    sx = hi_x - lo_x
    sy = hi_y - lo_y

    def box_el(b, label):
        cz = b.z + b.height / 2.0
        return f"""    <model name="{label}">
      <static>true</static>
      <pose>{b.cx:.3f} {b.cy:.3f} {cz:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{2*b.hx:.3f} {2*b.hy:.3f} {b.height:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{2*b.hx:.3f} {2*b.hy:.3f} {b.height:.3f}</size></box>
          </geometry>
          <material>
            <ambient>0.45 0.42 0.38 1</ambient>
            <diffuse>0.45 0.42 0.38 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    def actor_el(a):
        # A scripted trajectory actor carrying a visible box body.
        return f"""    <actor name="{a['name']}">
      <pose>{a['cx']} {a['cy']} 0 0 0 0</pose>
      <skin>
        <material script="Gazebo/Wood"/>
      </skin>
      <animation name="move">
        <script>
          <loop>true</loop>
          <step>0.01</step>
          <trajectory id="0" type="sin">
            <control>
              <time>0</time>
              <pose>{a['cx']} {a['cy']} 0 0 0 0</pose>
            </control>
            <control>
              <time>{a['period']}</time>
              <pose>{a['cx'] + a['x']} {a['cy'] + a['y']} 0 0 0 0</pose>
            </control>
          </trajectory>
        </script>
      </animation>
    </actor>"""

    boxes_xml = "\n\n".join(box_el(b, f"obstacle_{i:02d}")
                              for i, b in enumerate(boxes))
    actors_xml = ("\n\n" + "\n\n".join(actor_el(a) for a in actors)) if actors else ""

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
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx + 4:.1f} {sy + 4:.1f} 0.1</size></box>
          </geometry>
        </visual>
      </link>
    </model>

{boxes_xml}{actors_xml}

  </world>
</sdf>
"""


def build_occupancy(name, min_xy, max_xy, boxes):
    lo_x, lo_y = min_xy
    hi_x, hi_y = max_xy
    margin = 1.0
    mlo_x = lo_x - margin
    mlo_y = lo_y - margin
    mhi_x = hi_x + margin
    mhi_y = hi_y + margin
    cols = int(round((mhi_x - mlo_x) / RESOLUTION))
    rows = int(round((mhi_y - mlo_y) / RESOLUTION))
    grid = [[FREE] * cols for _ in range(rows)]

    def w2p(wx, wy):
        return int(round((wx - mlo_x) / RESOLUTION)), int(round((mhi_y - wy) / RESOLUTION))

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
        f"origin: [{mlo_x:.3f}, {mlo_y:.3f}, 0.000000]\n"
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
        actors = list(spec.get("actors", []))
        (world_dir / f"{name}.world").write_text(
            build_world_sdf(name, spec["min_xy"], spec["max_xy"], boxes, actors))
        pgm, map_yaml = build_occupancy(name, spec["min_xy"], spec["max_xy"], boxes)
        (map_dir / "map.pgm").write_bytes(pgm)
        (map_dir / "map.yaml").write_text(map_yaml)
        print(f"generated {name}: {len(actors)} actors, okay")


if __name__ == "__main__":
    main()
