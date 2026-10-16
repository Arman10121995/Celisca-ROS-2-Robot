"""Minimum distance from the car footprint to occupied map cells along a trace.

Run from the workspace root with mission reports whose .trace.json sits
beside them: python3 <this file> <report.json>...
"""
import json, math, sys
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

FP = {'ackermann_car': [(0.42, 0.17), (0.42, -0.17), (-0.10, -0.17), (-0.10, 0.17)],
      'anti_ackermann_car': [(0.42, 0.17), (0.42, -0.17), (-0.10, -0.17), (-0.10, 0.17)],
      'rear_steer_car': [(0.10, 0.17), (0.10, -0.17), (-0.42, -0.17), (-0.42, 0.17)]}
img = np.array(Image.open('src/robot_lab_maps/maps/nav_obstacle/maps/map.pgm'))
occ = (255 - img) / 255.0 > 0.65
res, ox, oy = 0.05, -9.5, -9.5
dist = distance_transform_edt(~occ) * res
H = img.shape[0]
def d_at(x, y):
    c, r = int((x - ox) / res), H - 1 - int((y - oy) / res)
    return dist[r, c] if 0 <= r < H and 0 <= c < img.shape[1] else 0.0
for path in sys.argv[1:]:
    car = next(k for k in FP if f'_{k}_' in path)
    rep = json.load(open(path)); tr = json.load(open(path + '.trace.json'))['samples']
    # sample the footprint boundary densely
    poly = FP[car] + [FP[car][0]]
    boundary = [(p[0] + (q[0] - p[0]) * t / 10, p[1] + (q[1] - p[1]) * t / 10) for p, q in zip(poly, poly[1:]) for t in range(10)]
    def clearance(s):
        c, sn = math.cos(s['yaw']), math.sin(s['yaw'])
        return min(d_at(s['x'] + c * bx - sn * by, s['y'] + sn * bx + c * by) for bx, by in boundary)
    cl = [(clearance(s), s) for s in tr]
    m, s = min(cl, key=lambda v: v[0])
    last = cl[-1]
    print(f"{path.split('/')[-1]:55s} min footprint clearance {m:.3f} m at t={s['stamp_s']:.1f} ({s['x']:.2f},{s['y']:.2f}); final {last[0]:.3f} m; B={rep['steps']['near_obstacle_b']['outcome']}")
