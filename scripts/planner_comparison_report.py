#!/usr/bin/env python3
"""Aggregate a planner_comparison.sh directory into distributions per planner.

Every goal of every seed counts, failures included: success rate, and for
successful goals the median / mean / std / min / max of simulated time,
path length, mean speed, final error and minimum footprint clearance.  The
same seeds give the same goals for every planner (matched inputs), so a
per-goal paired comparison is reported too (time and path-length ratios on
goals both planners reached).

  python3 scripts/planner_comparison_report.py <dir> [--markdown report.md]
"""
import argparse
import glob
import json
import math
import os
import re
import statistics


def load(directory):
    runs = {}
    for path in sorted(glob.glob(os.path.join(directory, '*_seed*.json'))):
        if path.endswith('.trace.json'):
            continue
        match = re.match(r'(.+)_seed(\d+)\.json$', os.path.basename(path))
        if not match:
            continue
        try:
            data = json.load(open(path))
        except ValueError:
            data = {'error': 'unreadable report'}
        runs.setdefault(match.group(1), {})[int(match.group(2))] = data
    return runs


def summary(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return None
    return dict(n=len(values), median=round(statistics.median(values), 3),
                mean=round(statistics.fmean(values), 3),
                std=round(statistics.pstdev(values), 3) if len(values) > 1 else 0.0,
                min=round(min(values), 3), max=round(max(values), 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--markdown')
    args = parser.parse_args()
    runs = load(args.directory)
    report = dict(planners={}, paired={})
    goals_by = {}
    for planner, seeds in runs.items():
        goals, errors = [], []
        for seed, data in sorted(seeds.items()):
            if data.get('error'):
                errors.append(dict(seed=seed, error=data['error']))
                continue
            for goal in data.get('goals', []):
                goal = dict(goal, seed=seed)
                goals.append(goal)
                goals_by.setdefault(planner, {})[(seed, goal.get('index'))] = goal
        ok = [g for g in goals if g.get('outcome') == 'succeeded']
        outcomes = {}
        for g in goals:
            outcomes[g.get('outcome')] = outcomes.get(g.get('outcome'), 0)+1
        report['planners'][planner] = dict(
            seeds=sorted(seeds), trial_errors=errors, goals=len(goals), succeeded=len(ok),
            success_rate=round(len(ok)/len(goals), 3) if goals else None, outcomes=outcomes,
            duration_sim_s=summary([g.get('duration_sim_s') for g in ok]),
            path_length_m=summary([g.get('path_length_m') for g in ok]),
            mean_speed_mps=summary([g.get('mean_speed_mps') for g in ok]),
            final_error_truth_m=summary([g.get('final_error_truth_m') for g in ok]),
            min_footprint_clearance_m=summary([g.get('min_footprint_clearance_m') for g in goals
                                               if g.get('min_footprint_clearance_m') is not None]))
    planners = sorted(goals_by)
    for i, a in enumerate(planners):
        for b in planners[i+1:]:
            keys = [k for k in goals_by[a] if k in goals_by[b]
                    and goals_by[a][k].get('outcome') == goals_by[b][k].get('outcome') == 'succeeded']
            same_goal = [k for k in keys if goals_by[a][k].get('goal') == goals_by[b][k].get('goal')]
            report['paired']['%s vs %s' % (a, b)] = dict(
                goals_both_succeeded=len(keys), identical_goal_positions=len(same_goal),
                time_ratio=summary([goals_by[a][k]['duration_sim_s']/goals_by[b][k]['duration_sim_s']
                                    for k in keys if goals_by[b][k].get('duration_sim_s')]),
                path_ratio=summary([goals_by[a][k]['path_length_m']/goals_by[b][k]['path_length_m']
                                    for k in keys if goals_by[b][k].get('path_length_m')]))
    manifest = os.path.join(args.directory, 'manifest.json')
    if os.path.isfile(manifest):
        report['manifest'] = json.load(open(manifest))
    print(json.dumps(report, indent=1))
    if args.markdown:
        lines = ['| Planner | Goals | Success | Sim time median (s) | Path median (m) | '
                 'Final error median (m) | Min clearance min (m) |', '|---|---|---|---|---|---|---|']
        for planner, row in sorted(report['planners'].items()):
            def med(key):
                value = row[key]
                return '%.3f' % value['median'] if value else 'n/a'
            clearance = row['min_footprint_clearance_m']
            lines.append('| %s | %d | %s | %s | %s | %s | %s |' % (
                planner, row['goals'], '%d/%d' % (row['succeeded'], row['goals']),
                med('duration_sim_s'), med('path_length_m'), med('final_error_truth_m'),
                '%.3f' % clearance['min'] if clearance else 'n/a'))
        open(args.markdown, 'w').write('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
