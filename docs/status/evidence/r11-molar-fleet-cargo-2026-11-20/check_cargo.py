"""Combine an actual completed RMF trip with the recorded loose-body observations."""
import json
import math
from pathlib import Path
import sys


def check(directory):
    mission = json.loads((directory/'mission.json').read_text())
    with (directory/'trace.jsonl').open() as trace:
        samples = [next(member for member in json.loads(line)['fleet']['members']
                        if member['id'] == 'molar').get('cargo') for line in trace]
    first = next((index for index, item in enumerate(samples)
                  if item and item['retained']), len(samples))
    witnessed = samples[first:]
    retained = [item for item in witnessed if item and item['retained'] and not item['fault']]
    levels = [item['world_xyz'][2] for item in retained]
    height_travel = max(levels)-min(levels) if levels else 0.
    max_shift = max((math.dist(item['relative_xyz'], retained[0]['relative_xyz']) for item in retained), default=0.)
    passed = (mission['phase'] == 'completed' and bool(witnessed)
              and len(witnessed) == len(retained) and height_travel >= 2.9)
    result = dict(passed=passed, roundtrip=mission['phase'], arrivals=mission['arrivals'],
                  samples_since_first_retention=len(witnessed), samples_retained=len(retained),
                  measured_box_vertical_travel_m=height_travel, max_relative_center_drift_m=max_shift)
    contents = json.dumps(result, indent=2)+'\n'
    (directory/'cargo-result.json').write_text(contents)
    print(contents, end='')
    return passed


if __name__ == '__main__':
    raise SystemExit(0 if check(Path(sys.argv[1])) else 1)
