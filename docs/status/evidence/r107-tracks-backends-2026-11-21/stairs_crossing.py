"""Score a stairs_probe trace: did the vehicle get up onto the step tread and down the far side?

Usage: stairs_crossing.py <case_dir> [tread_z]. tread_z is the truth frame's height on the
tread (0.18 m for base_footprint on MuJoCo/PyBullet; 0.252 m for Gazebo's base_link truth).
Crossed = level samples on the tread (|pitch| < 0.1, within 0.04 m of tread_z, y in the
tread span) and the run ends on the floor beyond the far edge (y > -3.45).
"""
import json, sys
from pathlib import Path
case = Path(sys.argv[1]); tread = float(sys.argv[2]) if len(sys.argv) > 2 else .18
rows = [json.loads(line) for line in (case / 'trace.jsonl').open()]
level = [r for r in rows if -4.45 < r['y'] < -3.5 and abs(r['z'] - tread) < .04 and abs(r['pitch']) < .1]
floor = rows[-1]['z'] < tread - .12
result = dict(case=case.name, samples=len(rows), level_on_tread=len(level),
              tread_y=[round(level[0]['y'], 3), round(level[-1]['y'], 3)] if level else None,
              end_y=round(rows[-1]['y'], 3), end_z=round(rows[-1]['z'], 3), max_tilt=round(max(abs(r['pitch']) for r in rows), 3),
              max_roll=round(max(abs(r['roll']) for r in rows), 3), crossed=bool(level) and floor and rows[-1]['y'] > -3.45)
(case / 'crossing.json').write_text(json.dumps(result, indent=2) + '\n'); print(json.dumps(result))
