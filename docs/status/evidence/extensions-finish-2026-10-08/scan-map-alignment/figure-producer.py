#!/usr/bin/env python3
"""Plot the original measured stationary scan against both actual grids.

Run from the workspace root with the compatible distribution plotting stack:
PYTHONPATH=/usr/lib/python3/dist-packages:src/robot_lab_utils python3 \
  docs/status/evidence/extensions-finish-2026-10-08/scan-map-alignment/figure-producer.py
This isolated plotting environment is not used for ROS tests or controllers.
Raw scan inputs remain at the hashed SSD path in endpoint-comparison.json.
"""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml

from robot_lab_utils.sim_frames import mounted_pose, yaw_of


folder = Path(__file__).resolve().parent
comparison = json.loads((folder/'endpoint-comparison.json').read_text())
trace_path = Path(comparison['trace']['path'])
assert hashlib.sha256(trace_path.read_bytes()).hexdigest() == comparison['trace']['sha256']
trace = json.loads(trace_path.read_text())
report = json.loads((folder/'bumperbot-hospital-mujoco-resolved/report.json').read_text())
scan = trace[-1]
body = scan['poses']['truth']
position, quaternion = mounted_pose(body['position'], body['quaternion'], report['laser_mount'])
ranges = np.asarray([r if r is not None else np.nan for r in scan['ranges']])
angles = scan['angle_min'] + np.arange(len(ranges))*scan['angle_increment'] + yaw_of(quaternion)
valid = np.isfinite(ranges) & (ranges >= scan['range_min']) & (ranges <= scan['range_max'])
xy = np.column_stack((position[0]+ranges[valid]*np.cos(angles[valid]),
                      position[1]+ranges[valid]*np.sin(angles[valid])))
fig, axes = plt.subplots(1, 2, figsize=(12, 6), constrained_layout=True)
for ax, key, title in zip(axes, ('closed-path-v3', 'all-segments-v4'),
                         ('Previous grid: open walls missing', 'Corrected grid: all slice segments')):
    result = comparison[key]
    map_yaml = Path(result['yaml'])
    metadata = yaml.safe_load(map_yaml.read_text())
    image_path = map_yaml.parent/metadata['image']
    assert hashlib.sha256(image_path.read_bytes()).hexdigest() == result['image_sha256']
    grid = np.asarray(Image.open(image_path))
    ox, oy, _ = metadata['origin']
    resolution = metadata['resolution']
    h, w = grid.shape
    ax.imshow(grid, cmap='gray', vmin=0, vmax=254,
              extent=(ox, ox+w*resolution, oy, oy+h*resolution), origin='upper')
    ax.scatter(xy[:, 0], xy[:, 1], s=6, c='#d73b32', label='Actual lidar endpoints')
    ax.scatter([position[0]], [position[1]], s=40, c='#1764ab', marker='x', label='Measured lidar origin')
    ax.set(xlim=(-12, 14), ylim=(0, 26), xlabel='World X (m)', ylabel='World Y (m)',
           title=f'{title}\nMedian grid distance: {result["median_m"]:.3f} m')
    ax.set_aspect('equal')
    ax.legend(loc='lower left', fontsize=8)
fig.suptitle('Hospital occupancy repair — same stationary physical sensor trace\n'
             'Statistics: 20 scans / 6,462 endpoints; plotted: final scan. No navigation claim.')
fig.savefig(folder/'before-after.png', dpi=160)
plt.close(fig)
