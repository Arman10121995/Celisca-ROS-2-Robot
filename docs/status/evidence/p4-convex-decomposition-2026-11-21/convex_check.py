import sys, time, json, numpy as np, yaml
from pathlib import Path
from PIL import Image
sys.path.insert(0, 'src/robot_lab_mujoco/python'); sys.path.insert(0, 'docs/status/evidence/p4-convex-decomposition-2026-11-21')
from robot_lab_utils.mesh_assets import load_indexed_mesh
from robot_lab_utils.planar_mesh import cached_retriangulation
from world_convex import decompose, check_against_grid
mesh, scale, map_yaml, threshold = sys.argv[1], float(sys.argv[2]), sys.argv[3], float(sys.argv[4])
v, f = load_indexed_mesh(mesh); v, f = cached_retriangulation(v, f); v = v * scale
t = time.time(); hulls = decompose(v, f, threshold=threshold, log=lambda m: print(m, flush=True)); print('hulls', len(hulls), 'seconds', round(time.time() - t, 1), flush=True)
meta = yaml.safe_load(Path(map_yaml).read_text()); image = np.asarray(Image.open(Path(map_yaml).parent / meta['image']).convert('L'))
occ = (255 - image.astype(float)) / 255.
grid = np.where(occ > meta['occupied_thresh'], 100, np.where(occ < meta['free_thresh'], 0, -1))[::-1]   # row 0 = bottom
print(json.dumps(check_against_grid(hulls, grid, meta['origin'][:2], meta['resolution'])))
