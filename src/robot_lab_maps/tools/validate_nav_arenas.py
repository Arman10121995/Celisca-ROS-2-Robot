#!/usr/bin/env python3
"""Validate box-arena maps and swept circular-footprint reference routes.

Run from source or a sourced install. This is static fixture qualification,
not runtime navigation evidence. Defaults to the Bumperbot test radius (0.14 m);
other robots must be checked with their own conservative footprint radius.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

TOOLS_DIR = Path(__file__).resolve().parent
MAPS_DIR = TOOLS_DIR.parent / "maps"
sys.path.insert(0, str(TOOLS_DIR))
# Source-tree invocation without sourcing a ROS overlay.
if (TOOLS_DIR.parents[1] / "robot_lab_utils").is_dir():
    sys.path.insert(0, str(TOOLS_DIR.parents[1] / "robot_lab_utils"))
from arena_clearance import Box2D, point_clearance, segment_clearance
from gen_nav_arenas import ARENAS
from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver


def parse_world_boxes(path):
    shapes, skipped = extract_static_shapes(str(path), uri_resolver())
    if skipped:
        raise ValueError(f"unsupported world geometry: {skipped}")
    boxes = []
    for shape in shapes:
        if shape["type"] != "box":
            raise ValueError(f"non-box geometry: {shape['model']}")
        x, y, z = shape["position"]
        sx, sy, sz = shape["size"]
        w, qx, qy, qz = shape["orientation"]
        if not all(math.isfinite(v) for v in (x, y, z, sx, sy, sz, w, qx, qy, qz)):
            raise ValueError("non-finite world geometry")
        if min(sx, sy, sz) <= 0 or abs(qx) + abs(qy) > 1e-8:
            raise ValueError("invalid box dimensions or tilted geometry")
        if z + sz / 2 <= 1e-8:  # Floor, below the navigation surface.
            continue
        boxes.append(Box2D(shape["model"], x, y, sx / 2, sy / 2,
                           2 * math.atan2(qz, w)))
    if not boxes:
        raise ValueError("no obstacle boxes found")
    return boxes


def read_pgm(path):
    """Read 8-bit binary P5 without discarding whitespace-valued pixels."""
    data = Path(path).read_bytes()
    index, tokens = 0, []
    whitespace = b" \t\r\n"
    while len(tokens) < 4:
        while index < len(data) and data[index] in whitespace:
            index += 1
        if index >= len(data):
            raise ValueError("truncated PGM header")
        if data[index] == ord("#"):
            while index < len(data) and data[index] not in b"\r\n":
                index += 1
            continue
        start = index
        while index < len(data) and data[index] not in whitespace:
            index += 1
        tokens.append(data[start:index])
    if tokens[0] != b"P5":
        raise ValueError("expected binary P5 PGM")
    cols, rows, maximum = map(int, tokens[1:])
    if cols <= 0 or rows <= 0 or maximum != 255:
        raise ValueError("expected positive PGM dimensions and max value 255")
    delimiter = 2 if data[index:index + 2] == b"\r\n" else 1
    body = data[index + delimiter:]
    if len(body) != cols * rows:
        raise ValueError("PGM payload size does not match dimensions")
    return cols, rows, maximum, bytearray(body)


def map_alignment_errors(boxes, metadata_path):
    """Check all obstacle interiors and clear free space, using map metadata.

    Generated maps pad geometry by one cell, then round inclusive raster bounds.
    Two cell diagonals bound that intentional fringe; outside it occupancy is
    an error. Unknown pixels are not considered free or occupied evidence.
    """
    config = yaml.safe_load(Path(metadata_path).read_text())
    resolution = float(config["resolution"])
    ox, oy, yaw = map(float, config["origin"])
    occupied_threshold, free_threshold = config["occupied_thresh"], config["free_thresh"]
    if (not all(math.isfinite(v) for v in (resolution, ox, oy, yaw))
            or resolution <= 0 or not 0 <= free_threshold < occupied_threshold <= 1
            or config["negate"] not in (0, 1)):
        raise ValueError("invalid map resolution, origin or occupancy thresholds")
    cols, rows, _, body = read_pgm(Path(metadata_path).parent / config["image"])
    col, row = np.meshgrid(np.arange(cols), np.arange(rows))
    lx, ly = (col + 0.5) * resolution, (rows - row - 0.5) * resolution
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = ox + c * lx - s * ly, oy + s * lx + c * ly
    distance = np.full((rows, cols), np.inf)
    errors = []
    for box in boxes:
        bc, bs = math.cos(box.yaw), math.sin(box.yaw)
        dx = np.abs(bc * (x - box.x) + bs * (y - box.y)) - box.hx
        dy = np.abs(-bs * (x - box.x) + bc * (y - box.y)) - box.hy
        signed = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)) + np.minimum(np.maximum(dx, dy), 0)
        distance = np.minimum(distance, signed)
        if not np.any(signed < 0):
            errors.append(f"map contains no interior pixels for {box.name}")
    value = np.frombuffer(body, dtype=np.uint8).reshape(rows, cols) / 255.0
    probability = value if config["negate"] else 1.0 - value
    missing = np.count_nonzero((distance < -1e-8) & (probability <= occupied_threshold))
    extra = np.count_nonzero((distance > 2 * math.sqrt(2) * resolution)
                             & (probability >= free_threshold))
    if missing:
        errors.append(f"{missing} obstacle-interior pixels are not occupied")
    if extra:
        errors.append(f"{extra} clear-space pixels are not free")
    return errors


def validate_arena(name, maps_dir=MAPS_DIR, navigation=None, radius=0.14):
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("footprint radius must be finite and positive")
    maps_dir = Path(maps_dir)
    if navigation is None:
        navigation = yaml.safe_load((maps_dir.parent / "config/arena_navigation.yaml").read_text())["arenas"]
    spec, metadata = ARENAS[name], navigation[name]
    boxes = parse_world_boxes(maps_dir / name / "worlds" / f"{name}.world")
    map_path = maps_dir / name / "maps/map.yaml"
    errors = map_alignment_errors(boxes, map_path)

    def point(pose):
        values = float(pose["x"]), float(pose["y"])
        if not all(math.isfinite(v) for v in (*values, float(pose["yaw"]))):
            raise ValueError("non-finite navigation pose")
        if not all(lo + radius <= v <= hi - radius
                   for v, lo, hi in zip(values, spec["min_xy"], spec["max_xy"])):
            errors.append(f"pose {values} exceeds arena footprint bounds")
        return values

    spawn = spec["spawn"][:2]
    spawn_clearance = point_clearance(boxes, spawn, radius)
    if spawn_clearance < 0:
        errors.append("spawn footprint intersects geometry")
    goals = [point(goal["pose"]) for goal in metadata["goals"]]
    if not goals or not metadata["reference_paths"]:
        errors.append("goals and reference paths must be nonempty")
    for goal in goals:
        if point_clearance(boxes, goal, radius) < 0:
            errors.append(f"goal footprint intersects geometry: {goal}")
    paths = []
    for path in metadata["reference_paths"]:
        points = [point(pose) for pose in path["waypoints"]]
        if len(points) < 2:
            errors.append(f"{path['id']}: fewer than two waypoints")
            continue
        if math.dist(points[0], spawn) > 1e-6:
            errors.append(f"{path['id']}: does not start at declared spawn")
        if not any(math.dist(points[-1], goal) <= 1e-6 for goal in goals):
            errors.append(f"{path['id']}: does not finish at a declared goal")
        clearance = [segment_clearance(boxes, a, b, radius) for a, b in zip(points, points[1:])]
        map_clearance = map_route_clearance(map_path, points, radius)
        if map_clearance < -1e-8:
            errors.append(f"{path['id']}: footprint lacks map clearance ({map_clearance:.6f} m)")
        for index, value in enumerate(clearance):
            if value < -1e-8:
                errors.append(f"{path['id']} segment {index}: footprint intersects geometry ({value:.6f} m)")
        paths.append({"id": path["id"], "minimum_clearance_m": min(clearance),
                      "map_clearance_lower_bound_m": map_clearance,
                      "length_m": sum(math.dist(a, b) for a, b in zip(points, points[1:]))})
    return {"arena": name, "valid": not errors, "footprint_radius_m": radius,
            "spawn_clearance_m": spawn_clearance, "paths": paths, "errors": errors}


def map_route_clearance(metadata_path, points, radius):
    """Conservative swept-disk clearance to occupied/unknown map cells and edges.

    Each blocked cell is enclosed by a circle of half a cell diagonal, so a
    positive result guarantees clearance to the entire pixel, not only its center.
    """
    config = yaml.safe_load(Path(metadata_path).read_text())
    resolution = float(config["resolution"])
    ox, oy, yaw = config["origin"]
    cols, rows, _, body = read_pgm(Path(metadata_path).parent / config["image"])
    value = np.frombuffer(body, dtype=np.uint8).reshape(rows, cols) / 255.0
    probability = value if config["negate"] else 1.0 - value
    row, col = np.nonzero(probability >= config["free_thresh"])
    x, y = (col + 0.5) * resolution, (rows - row - 0.5) * resolution
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(c * (px - ox) + s * (py - oy), -s * (px - ox) + c * (py - oy))
             for px, py in points]
    clearance = min(min(px, py, cols * resolution - px, rows * resolution - py)
                    - radius for px, py in local)
    for (ax, ay), (bx, by) in zip(local, local[1:]):
        if not len(x):
            break
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = np.clip(((x - ax) * dx + (y - ay) * dy) / length2, 0, 1) if length2 else 0
        distance = np.hypot(x - ax - t * dx, y - ay - t * dy)
        clearance = min(clearance, float(np.min(distance)) - radius - resolution / math.sqrt(2))
    return clearance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arena", action="append", choices=list(ARENAS))
    parser.add_argument("--radius", type=float, default=0.14)
    parser.add_argument("--json", type=Path, help="write the static qualification report")
    args = parser.parse_args(argv)
    reports = []
    for arena in args.arena or ARENAS:
        try:
            report = validate_arena(arena, radius=args.radius)
        except (ValueError, KeyError, OSError, TypeError) as error:
            report = {"arena": arena, "valid": False, "errors": [str(error)]}
        reports.append(report)
        print(f"[{'OK' if report['valid'] else 'FAIL'}] {arena}")
        for error in report["errors"]:
            print(f"  {error}")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(reports, indent=2, allow_nan=False) + "\n")
    return int(any(not report["valid"] for report in reports))


if __name__ == "__main__":
    sys.exit(main())
