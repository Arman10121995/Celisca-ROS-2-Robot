#!/usr/bin/env python3
"""Tabulate a sim_nav_mission_matrix.sh directory as Markdown.

One row per <sim>_<robot>_<map>.json cell, failures included: overall
result, the cancel stop time, and for the resumed detour goal A and the
near-obstacle goal B the Nav2 outcome and the final position/heading error
against simulator truth.

  python3 scripts/nav_mission_matrix_report.py <dir> [--markdown out.md] [--json out.json]
"""
import argparse
import glob
import json
import os
from pathlib import Path

import yaml

SIMULATORS = ('mujoco', 'pybullet', 'gazebo', 'isaac')
MAPS_YAML = Path(__file__).resolve().parent.parent/'src/robot_lab_bringup/config/sim_maps.yaml'


def known_maps():
    data = yaml.safe_load(MAPS_YAML.read_text()) or {}
    return set((data.get('maps') or data).keys())


def split_cell(name, maps):
    """'<sim>_<robot>_<map>.json' -> (sim, robot, map); robot and map may contain '_'."""
    stem = name[:-len('.json')]
    sim, _, rest = stem.partition('_')
    if sim not in SIMULATORS:
        return None
    parts = rest.split('_')
    for cut in range(1, len(parts)):
        if '_'.join(parts[cut:]) in maps:
            return sim, '_'.join(parts[:cut]), '_'.join(parts[cut:])
    return None


def load(directory):
    maps = known_maps()
    rows = []
    for path in sorted(glob.glob(os.path.join(directory, '*.json'))):
        name = os.path.basename(path)
        if name.endswith('.trace.json'):
            continue
        cell = split_cell(name, maps)
        if cell is None:
            continue
        try:
            data = json.load(open(path))
        except ValueError:
            data = {'error': 'unreadable result'}
        rows.append(dict(simulator=cell[0], robot=cell[1], map=cell[2], result=data))
    return rows


def step(data, key):
    entry = (data.get('steps') or {}).get(key) or {}
    if not entry:
        return 'not run'
    text = entry.get('outcome', '?')
    if entry.get('final_error_truth_m') is not None:
        text += ' %.3f m / %.1f deg' % (entry['final_error_truth_m'], entry.get('final_yaw_error_truth_deg', float('nan')))
    return text


def markdown(rows):
    lines = ['| Simulator | Robot | Map | Result | Cancel stop (sim s) | Goal A (truth error) | '
             'Goal B near obstacle (truth error) |', '|---|---|---|---|---:|---|---|']
    for row in rows:
        data = row['result']
        cancel = ((data.get('steps') or {}).get('cancel_a') or {})
        result = 'error: '+data['error'] if data.get('error') else ('pass' if data.get('passed') else 'FAIL')
        lines.append('| %s | %s | %s | %s | %s | %s | %s |' % (
            row['simulator'], row['robot'], row['map'], result,
            '%.2f' % cancel['stop_sim_s'] if cancel.get('stop_sim_s') is not None else 'n/a',
            step(data, 'resume_a'), step(data, 'near_obstacle_b')))
    passed = sum(1 for row in rows if row['result'].get('passed'))
    lines.append('')
    lines.append('%d of %d cells pass.' % (passed, len(rows)))
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--markdown')
    parser.add_argument('--json')
    args = parser.parse_args()
    rows = load(args.directory)
    text = markdown(rows)
    print(text)
    if args.markdown:
        open(args.markdown, 'w').write(text)
    if args.json:
        summary = [dict(simulator=r['simulator'], robot=r['robot'], map=r['map'],
                        passed=bool(r['result'].get('passed')), error=r['result'].get('error'),
                        steps=r['result'].get('steps'), start=r['result'].get('start'),
                        goal_a=r['result'].get('goal_a'), goal_b=r['result'].get('goal_b'))
                   for r in rows]
        open(args.json, 'w').write(json.dumps(summary, indent=1)+'\n')


if __name__ == '__main__':
    main()
