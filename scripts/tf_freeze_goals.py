#!/usr/bin/env python3
"""Count Nav2 goals that ended after a server's TF view froze.

The signature is a Nav2 server log line "Transform data too old when
converting" (controller_server: map->odom stale) or "has 0 poses"
(controller fed a plan that starts at an old planner pose).  Once it
appears, the server's TF view stays frozen for the rest of the launch.
A goal counts as frozen when it ended at or after the first such line.

  python3 scripts/tf_freeze_goals.py <comparison dir> [...]
"""
import json, re, sys, glob, os, collections
ansi = re.compile(r'\x1b\[[0-9;]*m')
def scan(run):
    onset = None; begins = []; ends = []
    for line in open(run + '.launch.log', errors='replace'):
        line = ansi.sub('', line)
        m = re.search(r'\[(\d{10}\.\d+)\]', line)
        if not m: continue
        t = float(m.group(1))
        if onset is None and ('too old when converting' in line or 'has 0 poses' in line):
            onset = t
        if 'Begin navigating' in line: begins.append(t)
        if re.search(r'Goal (succeeded|failed|canceled)', line): ends.append(t)
    goals = json.load(open(run + '.json')).get('goals', [])
    out = []
    for i, g in enumerate(goals):
        end = ends[i] if i < len(ends) else None
        start = begins[i] if i < len(begins) else None
        frozen = onset is not None and end is not None and end >= onset
        out.append((g['outcome'], frozen, start is not None and onset is not None and onset <= start))
    return onset, out
for d in sys.argv[1:]:
    per = collections.OrderedDict()
    for js in sorted(glob.glob(d + '/*_seed*.json')):
        run = js[:-5]
        if not os.path.exists(run + '.launch.log'): continue
        planner = os.path.basename(run).rsplit('_seed', 1)[0]
        onset, goals = scan(run)
        p = per.setdefault(planner, collections.Counter())
        for outcome, frozen, before in goals:
            p['goals'] += 1
            p['succeeded'] += outcome == 'succeeded'
            if outcome != 'succeeded' and frozen: p['failed_after_freeze'] += 1
            if outcome == 'succeeded' and frozen: p['succeeded_after_freeze'] += 1
        if onset: p['frozen_trials'] += 1
    print('==', d)
    for k, v in per.items():
        print(k, dict(v))
