#!/usr/bin/env python3
"""Add the independent Gazebo body summary to each world's flight report.

As in the November 10 round (px4-2026-11-10/<world>/report-measured.json):
the support matrix's flight_manual_screen reads ``body_trace`` (samples and
the initial/min/max height of /px4/odometry_truth). It is derived from the
run's trace.json, which then moves to SSD with its hash recorded here.

usage: derive_measured.py <evidence-dir> <ssd-dir> <world>...
"""
import hashlib
import json
from pathlib import Path
import shutil
import sys


def digest(path):
    data = path.read_bytes()
    return dict(path=str(path), bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


def main():
    evidence, ssd, worlds = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3:]
    for world in worlds:
        folder = evidence/world
        report = json.loads((folder/'report.json').read_text())
        if not report.get('passed') and 'directions' not in report:
            continue   # no flight to summarise
        target = ssd/world
        target.mkdir(parents=True, exist_ok=True)
        for name in ('trace.json', 'gui-output.log', 'live-gui-output.log'):
            if (folder/name).exists():
                shutil.move(str(folder/name), str(target/name))
        body = json.loads((target/'trace.json').read_text())['body']
        heights = [sample['z'] for sample in body]
        report['body_trace'] = dict(samples=len(body), initial_z=heights[0], min_z=min(heights), max_z=max(heights))
        report['derived_from'] = dict(original_report=digest(folder/'report.json'),
                                      physical_trace=digest(target/'trace.json'))
        (folder/'report-measured.json').write_text(json.dumps(report, indent=2)+'\n')
        print(world, report['passed'], report['body_trace'])


if __name__ == '__main__':
    main()
