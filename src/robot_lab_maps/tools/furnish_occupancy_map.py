#!/usr/bin/env python3
"""Add a furnished world's furniture to its building's 2D occupancy map.

The furnished Celisca maps shipped the plain floor's PGM unchanged, so the
2D map lacked every table, bed and cabinet the lidar sees: AMCL matched
scans against a map without them and the localization estimate drifted
(Nav2 reported success 0.3-5.4 m from the goal on 2026-10-12, while the
plain floors passed).

This slices the furnished world's *collision* geometry (the same proxy the
simulators collide with) at several heights inside the robots' scan and
body band and rasterizes the cuts onto the base map's own grid (origin and
resolution unchanged, so spawns, goals and the plain map stay aligned).
The sliced geometry - walls and furniture - becomes the occupied cells.
The base map is a floor-plan drawing whose walls differ from the 3D
building by up to ~10 cm (only ~45% of the plain world's own wall cut
lies within 7 cm of them), so its wall cells that the geometry does not
confirm take the base map's background value (205) instead of staying as
phantom walls.  Every other pixel keeps its base value.  (With these maps'
free_thresh of 0.25, 205 reads as free, exactly as in the plain map.)

Checked, not assumed: the spawn cell must stay free and most of the base
free area must remain free (--min-free-kept).

  furnish_occupancy_map.py --base maps/celisca_floor_1/maps/map.yaml \\
      --world maps/celisca_floor_1_furniture/worlds/celisca_floor_1_furniture.world \\
      --output maps/celisca_floor_1_furniture/maps
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
import yaml
from PIL import Image, ImageDraw

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def load_base(map_yaml):
    meta = yaml.safe_load(Path(map_yaml).read_text())
    image = np.array(Image.open(Path(map_yaml).parent/meta['image']))
    return meta, image


def world_meshes(world, package_share=None):
    """(name, trimesh in world coordinates) for every static collision mesh/primitive."""
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_utils.mesh_assets import load_indexed_mesh
    from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver
    shapes, skipped = extract_static_shapes(str(world), uri_resolver(package_share=package_share or get_package_share_directory),
                                            collision_only=True)
    meshes = []
    for shape in shapes:
        transform = trimesh.transformations.quaternion_matrix(shape['orientation'])
        transform[:3, 3] = shape['position']
        if shape['type'] == 'mesh':
            vertices, faces = load_indexed_mesh(shape['mesh'])
            mesh = trimesh.Trimesh(vertices*np.asarray(shape['scale']), faces, process=False)
        elif shape['type'] == 'box':
            mesh = trimesh.creation.box(shape['size'])
        elif shape['type'] == 'cylinder':
            mesh = trimesh.creation.cylinder(radius=shape['size'][0], height=shape['size'][1])
        elif shape['type'] == 'sphere':
            mesh = trimesh.creation.icosphere(radius=shape['size'][0])
        else:
            continue  # planes are floors; heightmaps are not furniture
        mesh.apply_transform(transform)
        meshes.append((shape.get('name', shape['type']), mesh))
    return meshes, skipped


def slice_mask(meshes, heights, meta, shape):
    """Boolean grid of cells crossed by any mesh section at any height."""
    resolution = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    height_px, width_px = shape
    canvas = Image.new('1', (width_px, height_px), 0)
    draw = ImageDraw.Draw(canvas)
    counts = {}
    for z in heights:
        segments = 0
        for _, mesh in meshes:
            if mesh.bounds[0][2] > z or mesh.bounds[1][2] < z:
                continue
            lines = trimesh.intersections.mesh_plane(mesh, [0, 0, 1], [0, 0, z])
            for (x0, y0, _), (x1, y1, _) in lines:
                # Image rows run top-down; map rows run bottom-up.
                draw.line([((x0-ox)/resolution, height_px-1-(y0-oy)/resolution),
                           ((x1-ox)/resolution, height_px-1-(y1-oy)/resolution)], fill=1, width=1)
                segments += 1
        counts['%.3f' % z] = segments
    return np.array(canvas, dtype=bool), counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True, help='plain building map.yaml')
    parser.add_argument('--world', type=Path, required=True, help='furnished world SDF')
    parser.add_argument('--output', type=Path, required=True, help='directory for map.pgm/map.yaml')
    parser.add_argument('--heights', type=float, nargs='+', default=[0.10, 0.15, 0.20, 0.25])
    parser.add_argument('--min-free-kept', type=float, default=0.85)
    parser.add_argument('--spawn', type=float, nargs=2, help='map-frame spawn x y that must stay free')
    args = parser.parse_args()
    meta, base = load_base(args.base)
    meshes, skipped = world_meshes(args.world)
    mask, counts = slice_mask(meshes, args.heights, meta, base.shape)
    occupied_below = 255*(1.0-float(meta['occupied_thresh']))
    free_above = 255*(1.0-float(meta['free_thresh']))
    walls = base < occupied_below
    free = base > free_above
    furnished = base.copy()
    furnished[walls & ~mask] = UNKNOWN
    furnished[mask] = OCCUPIED
    kept = float((free & ~mask).sum())/max(1, int(free.sum()))
    if kept < args.min_free_kept:
        raise SystemExit('Only %.1f%% of the base free area stays free; refusing' % (100*kept))
    if args.spawn:
        resolution = float(meta['resolution'])
        col = int((args.spawn[0]-float(meta['origin'][0]))/resolution)
        row = base.shape[0]-1-int((args.spawn[1]-float(meta['origin'][1]))/resolution)
        if not furnished[row, col] > free_above:
            raise SystemExit('The spawn cell is not free in the furnished map')
    args.output.mkdir(parents=True, exist_ok=True)
    Image.fromarray(furnished).save(args.output/'map.pgm')
    # Keep the base YAML text (number formats and layout) - the Celisca
    # alignment tools parse it - and point it at the new image.
    import re
    text = re.sub(r'^image:.*$', 'image: map.pgm', Path(args.base).read_text(), count=1, flags=re.M)
    (args.output/'map.yaml').write_text(text)
    report = dict(base_map=str(args.base), base_sha256=hashlib.sha256(
                      (args.base.parent/meta['image']).read_bytes()).hexdigest(),
                  world=str(args.world), world_sha256=hashlib.sha256(args.world.read_bytes()).hexdigest(),
                  heights_m=args.heights, segments_per_height=counts, skipped=skipped,
                  occupied_cells=int(mask.sum()), base_wall_cells=int(walls.sum()),
                  base_walls_confirmed=round(float((walls & mask).sum())/max(1, int(walls.sum())), 4),
                  base_free_kept=round(kept, 4),
                  method='collision geometry (walls and furniture) sliced at each height and rasterized '
                         'on the base grid as occupied; unconfirmed base wall pixels set to the base '
                         'background value 205; all other pixels and origin/resolution unchanged',
                  generator='src/robot_lab_maps/tools/furnish_occupancy_map.py')
    (args.output/'map_source.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('occupied_cells', 'base_wall_cells', 'base_walls_confirmed',
                                             'base_free_kept', 'segments_per_height')}))


if __name__ == '__main__':
    main()
