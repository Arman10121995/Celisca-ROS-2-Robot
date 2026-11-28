#!/usr/bin/env python3
"""Flocking phase metrics from swarm_probe traces: mean pair distance (median, max), share of
samples with every pair within 3.5 m (the members' sensing range), and the group centroid's travel.

usage: flock_metrics.py <trace.jsonl>...
"""
import json
import math
import statistics
import sys

for path in sys.argv[1:]:
    rows = [json.loads(line) for line in open(path) if line.strip()]
    rows = [r for r in rows if r['phase'] == 'local_flocking']
    means = [sum(r['pairs'].values()) / len(r['pairs']) for r in rows]
    within = sum(1 for r in rows if max(r['pairs'].values()) <= 3.5) / len(rows)
    def centroid(r):
        points = list(r['truth'].values())
        return [sum(p[i] for p in points) / len(points) for i in range(2)]
    first, last = centroid(rows[0]), centroid(rows[-1])
    print(json.dumps(dict(trace=path.split('/')[-3], samples=len(rows), mean_pair_median_m=round(statistics.median(means), 2),
                          mean_pair_max_m=round(max(means), 2), all_within_sensing_share=round(within, 3),
                          centroid_travel_m=round(math.dist(first, last), 2))))
