#!/usr/bin/env python3
"""Regenerate the small physics meshes for the two Celisca furniture worlds.

The source STLs have 3.3 and 4.0 million triangles. Gazebo ODE spends most of
its time testing robot-mesh against furniture-mesh contact and can fail to
advance simulation time. Keep those originals for rendering, and use a 50k
triangle approximation for collision in all simulator backends.

Requires ``pymeshlab`` for regeneration only; the checked-in collision meshes
have no runtime dependency on it.
"""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pymeshlab


MAPS = Path(__file__).resolve().parents[1] / "maps"
TARGET_FACES = 50_000
FLOOR_CUTOFF_METRES = 0.08


def remove_floor_facets(path, scale):
    """Leave the existing physics-floor box to handle ground contact.

    The furniture STL includes tens of thousands of coplanar floor triangles.
    Even after decimation, those triangles overlap the box and the robot's
    collision meshes, triggering ODE's trimesh-trimesh contact overflow.
    Keep triangles reaching above the floor for walls and furnishings.
    """
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    triangle = np.dtype([("normal", "<f4", (3,)),
                         ("vertices", "<f4", (3, 3)),
                         ("attribute", "<u2")])
    facets = np.frombuffer(raw, dtype=triangle, count=count, offset=84)
    keep = facets["vertices"][:, :, 2].max(axis=1) * scale > FLOOR_CUTOFF_METRES
    retained = facets[keep]
    path.write_bytes(raw[:80] + struct.pack("<I", len(retained)) + retained.tobytes())
    return len(retained)


def main():
    for floor in (1, 2):
        stem = f"celisca_floor_{floor}_furniture"
        directory = MAPS / stem / "meshes"
        source = directory / f"{stem}.stl"
        target = directory / f"{stem}_collision.stl"
        mesh = pymeshlab.MeshSet()
        mesh.load_new_mesh(str(source))
        original = mesh.current_mesh().face_number()
        mesh.meshing_decimation_quadric_edge_collapse(
            targetfacenum=TARGET_FACES,
            preserveboundary=False,
            preservetopology=False,
        )
        faces = mesh.current_mesh().face_number()
        if faces > TARGET_FACES * 1.1:
            raise RuntimeError(f"{stem}: decimation left {faces} faces")
        mesh.save_current_mesh(str(target), binary=True)
        world = MAPS / stem / "worlds" / f"{stem}.world"
        root = ET.parse(world).getroot()
        scale_text = root.findtext(
            f".//model[@name='{stem}']/link/visual/geometry/mesh/scale")
        scale = float(scale_text.split()[0])
        collision_faces = remove_floor_facets(target, scale)
        print(f"{stem}: {original} -> {faces} -> {collision_faces} "
              f"above-floor collision faces: {target}")


if __name__ == "__main__":
    main()
