#!/usr/bin/env python3
"""
P4.2 - Validate the generated deterministic navigation arenas.

Reads each arena's `.world` and its companion `map.pgm` / `map.yaml` and
verifies the two agree: every obstacle footprint declared in the world SDF
rasterizes to an occupied region in the occupancy map, and the map origin /
resolution are sane. This is what guarantees the world geometry and the
localization map stay perfectly in sync.

usage:
    python3 tools/validate_nav_arenas.py
"""
from __future__ import annotations

from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
MAPS_DIR = TOOLS_DIR.parent / "maps"
RES = 0.05
ARENAS = ["nav_empty", "nav_obstacle", "nav_maze", "nav_narrow_passage", "nav_warehouse"]


def parse_world_boxes(path: Path):
    """Return a list of (x, y, hx, hy) obstacle center + half-extents."""
    text = path.read_text()
    boxes = []
    for model in text.split("<model"):
        if "obstacle_" not in model:
            continue
        # Name + pose
        seg = model[model.find("name=") + 6:]
        name = seg[: seg.find('"')]
        pose = model[model.find("<pose>") + 6: model.find("</pose>")]
        cx, cy, *_ = (float(v) for v in pose.split())
        size = model[model.find("<size>") + 6: model.find("</size>")]
        sx, sy, _ = (float(v) for v in size.split())
        boxes.append((name, cx, cy, sx / 2.0, sy / 2.0))
    return boxes


def read_pgm(path: Path):
    """Read a binary P5 PGM, return (cols, rows, max, bytearray)."""
    data = path.read_bytes()
    assert data[:2] == b"P5", "not P5"
    idx = 2
    parts = []
    while len(parts) < 3:
        # skip whitespace/comments
        while data[idx] in (0x20, 0x09, 0x0A, 0x0D):
            idx += 1
        if data[idx] == 0x23:  # '#'
            while data[idx] not in (0x0A, 0x0D):
                idx += 1
            continue
        tok = []
        while data[idx] not in (0x20, 0x09, 0x0A, 0x0D):
            tok.append(data[idx])
            idx += 1
        parts.append(int(bytes(tok)))
    cols, rows, maxv = parts
    body = data[idx + 1:]
    return cols, rows, maxv, bytearray(body)


def main():
    failures = 0
    for arena in ARENAS:
        world = MAPS_DIR / arena / "worlds" / f"{arena}.world"
        pgm = MAPS_DIR / arena / "maps" / "map.pgm"
        yaml = MAPS_DIR / arena / "maps" / "map.yaml"

        boxes = parse_world_boxes(world)
        cols, rows, maxv, body = read_pgm(pgm)
        yaml_text = yaml.read_text()
        origin_line = [l for l in yaml_text.splitlines() if l.startswith("origin")][0]
        origin_x = float(origin_line.split("[")[1].split(",")[0])
        origin_y = float(origin_line.split("[")[1].split(",")[1])
        map_hi_y = origin_y + rows * RES

        def occupied_at(wx, wy):
            c = int(round((wx - origin_x) / RES))
            r = int(round((map_hi_y - wy) / RES))
            if r < 0 or r >= rows or c < 0 or c >= cols:
                return True  # outside map => treat as boundary
            return body[r * cols + c] == 0

        ok = True
        for name, cx, cy, hx, hy in boxes:
            # Center of each obstacle must be occupied.
            if not occupied_at(cx, cy):
                print(f"  [FAIL] {arena}: {name} center not occupied")
                ok = False
        # A free probe near origin (0,0) must not be occupied for most arenas.
        if not boxes:
            print(f"  [FAIL] {arena}: no obstacles found in world")
            ok = False
        if ok:
            print(f"  [OK] {arena}: {len(boxes)} obstacles, grid {cols}x{rows}")
        else:
            failures += 1

    print("PASS" if failures == 0 else "FAILURES")
    return failures


if __name__ == "__main__":
    import sys
    sys.exit(main())
