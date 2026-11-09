#!/usr/bin/env python3
"""Summarise recorded robot-vacuum jobs (scripts/sim_vacuum_mission.sh).

For each report: result, coverage, cleaned area, time, distance, events;
keep-out compliance from the simulator truth trail (no truth sample of the
robot centre inside a no-go polygon grown by the robot radius, no truth
segment crossing a virtual wall); docking error against the dock pose; and
a figure (map, cleaned area from the run's history file, truth trail,
keep-outs, dock).

Usage: vacuum_run_report.py --map-yaml MAP.yaml --out-dir DIR report.json...
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw


def load_map(path):
    meta = yaml.safe_load(open(path))
    img = np.array(Image.open(Path(path).parent / meta['image']).convert('L'))
    return img, float(meta['resolution']), meta['origin'][:2]


def point_in_polygon(x, y, poly):
    inside = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[i - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def dist_to_polygon(x, y, poly):
    if point_in_polygon(x, y, poly):
        return 0.0
    return min(dist_segment(x, y, poly[i - 1], poly[i]) for i in range(len(poly)))


def dist_segment(x, y, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / max(dx * dx + dy * dy, 1e-12)))
    return math.hypot(x - a[0] - t * dx, y - a[1] - t * dy)


def crosses(p1, p2, q1, q2):
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return (orient(p1, p2, q1) * orient(p1, p2, q2) < 0) and (orient(q1, q2, p1) * orient(q1, q2, p2) < 0)


def analyse(report, radius):
    trail = [s for s in report.get('trail') or [] if s.get('truth')]
    layout = report.get('layout') or {}
    out = {k: report.get(k) for k in ('result', 'mode', 'suction', 'passes', 'edge', 'area_m2', 'coverage_pct',
                                      'duration_s', 'distance_m', 'events', 'skipped', 'recharges',
                                      'bin_empties', 'battery_start', 'battery_end', 'final_state',
                                      'estimate_s')}
    out['trail_samples'] = len(trail)
    if trail:
        est = [s for s in trail if s.get('pose')]
        errs = [math.hypot(s['pose'][0] - s['truth'][0], s['pose'][1] - s['truth'][1]) for s in est]
        out['localisation_error_m'] = {'mean': round(float(np.mean(errs)), 3), 'max': round(float(np.max(errs)), 3)} if errs else None
    nogo = [z['polygon'] for z in layout.get('no_go') or []]
    walls = [(w['p1'], w['p2']) for w in layout.get('walls') or []]
    if nogo:
        clear = min(dist_to_polygon(s['truth'][0], s['truth'][1], poly) for s in trail for poly in nogo) if trail else None
        out['no_go_min_centre_distance_m'] = None if clear is None else round(clear, 3)
        out['no_go_body_entered'] = None if clear is None else bool(clear < radius - 0.02)
    if walls:
        hits = 0
        for a, b in zip(trail, trail[1:]):
            for w in walls:
                if crosses(a['truth'][:2], b['truth'][:2], w[0], w[1]):
                    hits += 1
        out['wall_crossings'] = hits
    dock, truth = report.get('dock'), report.get('final_truth')
    if dock and truth:
        out['dock_error_m'] = round(math.hypot(truth[0] - dock[0], truth[1] - dock[1]), 3)
        out['dock_yaw_error_deg'] = round(math.degrees(math.atan2(math.sin(truth[2] - dock[2]),
                                                                  math.cos(truth[2] - dock[2]))), 1)
    sensors = report.get('sensors') or {}
    out['sensors'] = {'bumper_messages': sensors.get('bumper_messages'),
                      'cliff': {k: v.get('reading_m') for k, v in (sensors.get('cliff') or {}).items()}}
    return out


def figure(report, store, map_img, res, origin, path, radius):
    h, w = map_img.shape
    rgb = np.zeros((h, w, 3), np.uint8)
    rgb[:] = (205, 205, 205)
    rgb[map_img > 250] = (250, 250, 250)
    rgb[map_img < 100] = (40, 40, 40)
    cov_file = Path(store) / 'vacuum' / 'history' / ('%s.npz' % report.get('id'))
    if cov_file.exists():
        data = np.load(cov_file)
        shape = tuple(int(v) for v in data['shape'])
        cov = np.unpackbits(data['coverage'])[:shape[0] * shape[1]].reshape(shape).astype(bool)
        if cov.shape == (h, w):
            cov = cov[::-1]  # grid rows go up, image rows go down
            rgb[cov & (map_img > 250)] = (120, 200, 140)
    img = Image.fromarray(rgb).resize((w * 2, h * 2), Image.NEAREST)
    d = ImageDraw.Draw(img)

    def px(x, y):
        return ((x - origin[0]) / res * 2, (h - (y - origin[1]) / res) * 2)
    layout = report.get('layout') or {}
    for z in layout.get('no_go') or []:
        d.polygon([px(*p) for p in z['polygon']], outline=(220, 30, 60), fill=(240, 150, 160))
    for wl in layout.get('walls') or []:
        d.line([px(*wl['p1']), px(*wl['p2'])], fill=(220, 30, 60), width=5)
    for z in layout.get('zones') or []:
        x0, y0, x1, y1 = z['rect'] if isinstance(z, dict) else z
        d.rectangle([px(x0, y1), px(x1, y0)], outline=(40, 110, 230), width=2)
    req = report.get('request') or {}
    for z in req.get('zones') or []:
        x0, y0, x1, y1 = z['rect'] if isinstance(z, dict) else z
        d.rectangle([px(x0, y1), px(x1, y0)], outline=(40, 110, 230), width=2)
    pts = [px(*s['truth'][:2]) for s in report.get('trail') or [] if s.get('truth')]
    if len(pts) > 1:
        d.line(pts, fill=(230, 120, 20), width=2)
    dock = report.get('dock')
    if dock:
        x, y = px(*dock[:2])
        d.rectangle([x - 6, y - 6, x + 6, y + 6], fill=(250, 210, 40), outline=(0, 0, 0))
    known = np.argwhere(map_img != 205)
    if len(known):
        (r0, c0), (r1, c1) = known.min(0), known.max(0)
        img = img.crop((max(0, c0 * 2 - 20), max(0, r0 * 2 - 20), min(w * 2, c1 * 2 + 20), min(h * 2, r1 * 2 + 20)))
    img.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map-yaml', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--radius', type=float, default=0.165)
    ap.add_argument('reports', nargs='+')
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    img, res, origin = load_map(args.map_yaml)
    summary = {}
    for path in args.reports:
        report = json.load(open(path))
        name = Path(path).stem
        summary[name] = analyse(report, args.radius)
        if report.get('trail') is not None:
            figure(report, str(Path(path).with_suffix('')) + '_store', img, res, origin, out / (name + '.png'), args.radius)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1, sort_keys=True))
    print(json.dumps(summary, indent=1, sort_keys=True))


if __name__ == '__main__':
    main()
