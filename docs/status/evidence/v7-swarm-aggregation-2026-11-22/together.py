#!/usr/bin/env python3
"""Share of local_aggregation samples with all three members within 1.5 m of each other
("together", as in the 2026-11-18 receipt), the first time they were, and the second-half share.

usage: together.py <trace.jsonl>...
"""
import json
import sys

for path in sys.argv[1:]:
    rows = [json.loads(line) for line in open(path) if line.strip()]
    rows = [r for r in rows if r['phase'] == 'local_aggregation']
    flags = [max(r['pairs'].values()) <= 1.5 for r in rows]
    first = next((r['sim']-rows[0]['sim'] for r, f in zip(rows, flags) if f), None)
    half = flags[len(flags)//2:]
    print(json.dumps(dict(trace=path, samples=len(rows), together=round(sum(flags)/len(flags), 3),
                          together_second_half=round(sum(half)/len(half), 3),
                          first_together_s=round(first, 1) if first is not None else None)))
