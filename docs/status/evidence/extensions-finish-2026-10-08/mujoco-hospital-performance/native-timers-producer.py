#!/usr/bin/env python3
"""Profile native stages of the exact captured scene in a bounded replay.

No ROS, sensors, viewer, movement controller or navigation runs here. Timers
use perf_counter seconds, chosen explicitly as permitted by mjcb_time:
https://mujoco.readthedocs.io/en/stable/APIreference/APIglobals.html#mjcb-time
Instrumentation overhead remains. This does not qualify simulator RTF.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--profile', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--steps', type=int, default=500)
parser.add_argument('--warmup', type=int, default=100)
args = parser.parse_args()
assert 0 < args.steps <= 5000 and 0 <= args.warmup <= 500
captured = json.loads(args.profile.read_text())
scene = Path(captured['scene']['path'])
assert hashlib.sha256(scene.read_bytes()).hexdigest() == captured['scene']['sha256']
assert args.output.parent.stat().st_dev != Path('/').stat().st_dev
report = dict(scope='Actual bounded offline native-stage timing of captured Labbot/hospital model with zero actuator commands. No ROS/sensor/viewer/controller/nav or full RTF qualification; instrumentation overhead retained.',
    engine_version=mujoco.__version__, captured_profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
    scene=captured['scene'], steps=args.steps, warmup_steps=args.warmup,
    timer_unit='seconds', timer_callback='time.perf_counter', completed=False)
previous_callback = mujoco.get_mjcb_time()
try:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    report['model_counts'] = dict(nbody=model.nbody,ngeom=model.ngeom,nmesh=model.nmesh,
        nflex=model.nflex,nflexvert=model.nflexvert,nflexelem=model.nflexelem)
    mujoco.mj_forward(model,data)
    for _ in range(args.warmup):
        mujoco.mj_step(model,data)
    timers = {name:int(value) for name,value in mujoco.mjtTimer.__members__.items() if name!='mjNTIMER'}
    before = {name:(float(data.timer[index].duration),int(data.timer[index].number)) for name,index in timers.items()}
    sim_start = float(data.time)
    mujoco.set_mjcb_time(time.perf_counter)
    wall_start = time.perf_counter()
    for _ in range(args.steps):
        mujoco.mj_step(model,data)
    report.update(wall_s=time.perf_counter()-wall_start,simulation_s=float(data.time)-sim_start)
    report['timers'] = {}
    for name,index in timers.items():
        elapsed = float(data.timer[index].duration)-before[name][0]
        calls = int(data.timer[index].number)-before[name][1]
        assert np.isfinite(elapsed) and elapsed>=0 and calls>=0
        report['timers'][name] = dict(total_s=elapsed,calls=calls,
                                      mean_ms=elapsed*1000/calls if calls else None)
    assert report['timers']['mjTIMER_STEP']['calls'] == args.steps
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    root_joint = next(i for i in range(model.njnt) if model.jnt_type[i]==mujoco.mjtJoint.mjJNT_FREE)
    root_body = int(model.jnt_bodyid[root_joint])
    report['final_body'] = dict(name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,root_body),
        position=data.xpos[root_body].tolist(),quaternion_wxyz=data.xquat[root_body].tolist())
    report['completed'] = True
finally:
    mujoco.set_mjcb_time(previous_callback)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
print({k:v for k,v in report.items() if k not in ('scene','timers')})
print(report.get('timers'))
