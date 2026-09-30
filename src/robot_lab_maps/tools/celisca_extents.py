"""Report the real extents of the Celisca world meshes and maps.

Used to size a uniform scale for the map assets: the Gazebo world applies a
``<scale>`` to the STL, the MJCF world bakes the same number in, and the 2D
map is rasterised in metres.  If those three disagree, a robot spawned at the
configured (x, y) sits in a wall in one backend and in the open in another.
"""

import os
import struct
import sys

MAPS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps")


def stl_bounds(path):
    """(lo, hi, triangles) for a binary or ASCII STL."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    count = 0
    with open(path, "rb") as handle:
        head = handle.read(84)
        # An ASCII STL also starts with "solid", so the magic bytes cannot
        # decide it.  A binary STL declares its triangle count in bytes 80-83
        # and the file is then exactly 84 + 50 * n bytes long; an ASCII one
        # never matches.  Trust the size, and fall back to text if it does.
        binary = False
        if len(head) == 84:
            declared = struct.unpack("<I", head[80:84])[0]
            size = os.path.getsize(path)
            binary = size == 84 + 50 * declared and declared > 0
        if binary:
            count = struct.unpack("<I", head[80:84])[0]
            for index in range(count):
                record = handle.read(50)
                if len(record) < 50:
                    # A truncated mesh must not report the partial file's bounds
                    # as if they were the real extents.
                    raise ValueError("truncated binary STL: only %d of %d "
                                     "triangles are present" % (index, count))
                # 3 normals + 9 vertex floats = 12; the last 2 bytes are an
                # attribute byte count that this tool does not need.
                values = struct.unpack("<12f", record[:48])
                for j in range(3):
                    vertex = values[3 + j * 3:6 + j * 3]
                    for k in range(3):
                        lo[k] = min(lo[k], vertex[k])
                        hi[k] = max(hi[k], vertex[k])
            return lo, hi, count
        for line in handle.read().decode("utf-8", "replace").splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                for k in range(3):
                    value = float(parts[k + 1])
                    lo[k] = min(lo[k], value)
                    hi[k] = max(hi[k], value)
                count += 1
    return lo, hi, count


def map_info(name):
    """Free-space structure of a map: largest open region and the spawn cell."""
    from PIL import Image
    import numpy as np

    root = os.path.join(MAPS, name, "maps")
    meta = {}
    with open(os.path.join(root, "map.yaml")) as handle:
        for line in handle:
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            meta[key.strip()] = value.strip()
    res = float(meta["resolution"])
    # map.yaml writes origin as a YAML flow sequence: "[-19.173, -8.393, 0]".
    origin = [float(part) for part in
              meta["origin"].strip().lstrip("[").rstrip("]").split(",")]
    cells = np.array(Image.open(os.path.join(root, "map.pgm")))
    height, width = cells.shape
    free = cells > 200

    out = {
        "resolution": res,
        "origin": origin,
        "pixels": (width, height),
        "metres": (width * res, height * res),
        "x_range": (origin[0], origin[0] + width * res),
        "y_range": (origin[1], origin[1] + height * res),
        "free_fraction": float(free.mean()),
    }

    # Connected component of free space that contains a probe point, so we can
    # tell "in the open" from "inside a sealed room the size of a closet".
    try:
        from scipy import ndimage
    except ImportError:
        return out
    labels, count = ndimage.label(free)
    for px, py in ((0.0, 1.1), (0.0, 0.0)):
        col = int((px - origin[0]) / res)
        row = int((py - origin[1]) / res)
        if not (0 <= col < width and 0 <= row < height):
            out.setdefault("probe", []).append((px, py, "outside the map"))
            continue
        label = labels[row, col]
        if label == 0:
            out.setdefault("probe", []).append((px, py, "occupied cell"))
            continue
        size = int((labels == label).sum())
        ys, xs = np.nonzero(labels == label)
        out.setdefault("probe", []).append((
            px, py, "component area %.1f m2, bbox x %.2f..%.2f y %.2f..%.2f"
            % (size * res * res,
               origin[0] + xs.min() * res, origin[0] + xs.max() * res,
               origin[1] + ys.min() * res, origin[1] + ys.max() * res)))
    out["components"] = count
    return out


def main(argv):
    scale = float(argv[1]) if len(argv) > 1 else 1.155
    for name in sorted(os.listdir(MAPS)):
        mesh_dir = os.path.join(MAPS, name, "meshes")
        if not os.path.isdir(mesh_dir):
            continue
        for mesh in sorted(os.listdir(mesh_dir)):
            if not mesh.lower().endswith(".stl"):
                continue
            try:
                lo, hi, tris = stl_bounds(os.path.join(mesh_dir, mesh))
            except Exception as exc:  # pragma: no cover - diagnostic tool
                print("%-34s %-28s ERROR %s" % (name, mesh, exc))
                continue
            print("%-30s %-26s tris=%-8d raw x %8.3f..%8.3f  y %8.3f..%8.3f  z %7.3f..%7.3f"
                  % (name, mesh, tris, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
            print("%-30s %-26s scale %.3f -> x %8.3f..%8.3f  y %8.3f..%8.3f"
                  % ("", "", scale,
                     lo[0] * scale, hi[0] * scale, lo[1] * scale, hi[1] * scale))
        if os.path.isdir(os.path.join(MAPS, name, "maps")):
            info = map_info(name)
            print("%-30s MAP %-26s %dx%d px @ %.5f m = %.2f x %.2f m, free %.1f%%"
                  % ("", "map.pgm", info["pixels"][0], info["pixels"][1],
                     info["resolution"], info["metres"][0], info["metres"][1],
                     100.0 * info["free_fraction"]))
            print("%-30s %-26s x %.2f..%.2f  y %.2f..%.2f"
                  % ("", "extent", info["x_range"][0], info["x_range"][1],
                     info["y_range"][0], info["y_range"][1]))
            for probe in info.get("probe", ()):
                print("%-30s %-26s probe (%.1f, %.1f): %s"
                      % ("", "", probe[0], probe[1], probe[2]))
        print()


def world_scales():
    """The mesh <scale> each Celisca .world actually applies."""
    found = {}
    for floor in sorted(os.listdir(MAPS)):
        world_dir = os.path.join(MAPS, floor, "worlds")
        if not os.path.isdir(world_dir):
            continue
        for name in sorted(os.listdir(world_dir)):
            if not name.endswith(".world"):
                continue
            path = os.path.join(world_dir, name)
            with open(path) as handle:
                for line in handle:
                    if "<scale>" in line:
                        value = line.split("<scale>")[1].split("</scale>")[0].split()
                        found.setdefault(floor, set()).add(
                            tuple(round(float(v), 6) for v in value))
    return found


def alignment_report():
    """Compare each world's applied mesh scale with its 2D map footprint.

    A world whose mesh is much larger (or offset) relative to the map it
    ships means the robot, the walls and the 2D costmap disagree about where
    the building is - the robot can be inside a wall in 3D while the map says
    the cell is free, which is what "it spawns outside the map" looks like.
    """
    scales = world_scales()
    rows = []
    for floor, values in sorted(scales.items()):
        if len(values) != 1:
            rows.append((floor, None, "worlds disagree on <scale>: %s" % values))
            continue
        applied = values.pop()[0]
        mesh_dir = os.path.join(MAPS, floor, "meshes")
        if not os.path.isdir(mesh_dir):
            continue
        meshes = [m for m in sorted(os.listdir(mesh_dir)) if m.lower().endswith(".stl")]
        if not meshes or not os.path.isdir(os.path.join(MAPS, floor, "maps")):
            continue
        lo, hi, _ = stl_bounds(os.path.join(mesh_dir, meshes[0]))
        info = map_info(floor)
        if "x_range" not in info:
            continue
        mesh_w = (hi[0] - lo[0]) * applied
        mesh_h = (hi[1] - lo[1]) * applied
        map_w = info["x_range"][1] - info["x_range"][0]
        map_h = info["y_range"][1] - info["y_range"][0]
        centre_mx = ((lo[0] + hi[0]) / 2.0) * applied
        centre_my = ((lo[1] + hi[1]) / 2.0) * applied
        centre_px = (info["x_range"][0] + info["x_range"][1]) / 2.0
        centre_py = (info["y_range"][0] + info["y_range"][1]) / 2.0
        offset = ((centre_mx - centre_px) ** 2 + (centre_my - centre_py) ** 2) ** 0.5
        rows.append((floor, applied,
                     "mesh %.1f x %.1f m vs map %.1f x %.1f m (ratio %.2f, %.2f), "
                     "centre offset %.2f m"
                     % (mesh_w, mesh_h, map_w, map_h,
                        mesh_w / map_w if map_w else 0.0,
                        mesh_h / map_h if map_h else 0.0, offset)))
    return rows


if __name__ == "__main__":
    if "--align" in sys.argv[1:]:
        rows = alignment_report()
        lines = ["Celisca world mesh scale vs 2D map footprint:"]
        for floor, applied, note in rows:
            lines.append("  %-26s scale=%-10s %s" % (floor, applied, note))
        print("\n".join(lines))
    else:
        main(sys.argv)

