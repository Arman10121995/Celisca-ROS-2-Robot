"""Observe native-stage timers on one normal GUI spawner; no physics changes.

Opt in with this directory on PYTHONPATH and ROBOT_LAB_NATIVE_TIMER_DIRECTORY
pointing to a fresh SSD directory. mjcb_time uses perf_counter seconds:
https://mujoco.readthedocs.io/en/stable/APIreference/APIglobals.html#mjcb-time
No cProfile runs in this stage. Native timer callback overhead remains.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time

if Path(sys.argv[0]).name=='mujoco_spawner' and os.environ.get('ROBOT_LAB_NATIVE_TIMER_DIRECTORY'):
    original_run=threading.Thread.run

    def observed_run(thread):
        target=getattr(thread,'_target',None)
        if (getattr(target,'__name__','')!='_loop'
                or getattr(target,'__module__','')!='robot_lab_mujoco.mujoco_spawner'):
            return original_run(thread)
        import mujoco
        node=target.__self__
        model,data=node._model,node._data
        folder=Path(os.environ['ROBOT_LAB_NATIVE_TIMER_DIRECTORY'])
        assert folder.is_dir() and folder.stat().st_dev!=Path('/').stat().st_dev
        timers={name:int(value) for name,value in mujoco.mjtTimer.__members__.items() if name!='mjNTIMER'}
        before={name:(float(data.timer[i].duration),int(data.timer[i].number)) for name,i in timers.items()}
        source=Path(sys.modules[target.__module__].__file__)
        report=dict(scope='Native timers on actual normal GUI Labbot/MuJoCo/hospital v4 stationary loop. Real launch initialization and unchanged sensing/physics/controllers; timing callback overhead retained. No mission or whole-RTF qualification.',
            engine_version=mujoco.__version__,timer_unit='seconds',timer_callback='time.perf_counter',
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            hook_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),argv=sys.argv,
            model_counts=dict(nbody=model.nbody,ngeom=model.ngeom,nmesh=model.nmesh,nflex=model.nflex,
                              nflexvert=model.nflexvert,nflexelem=model.nflexelem),
            flex_rigid_count=int(model.flex_rigid.sum()),
            world_bound_flex_vertices=int((model.flex_vertbodyid==0).sum()),
            initial_state=dict(qpos=data.qpos.tolist(),qvel=data.qvel.tolist(),ctrl=data.ctrl.tolist()),
            spawn_parameters={k:node.get_parameter(k).value for k in ('spawn_x','spawn_y','spawn_z','spawn_yaw')},
            sensor_parameters={k:node.get_parameter(k).value for k in
                ('scan_rate','scan_samples','camera_rate','camera_width','camera_height')},
            start_sim_s=float(data.time),viewer_present=node._viewer is not None)
        callback=mujoco.get_mjcb_time()
        mujoco.set_mjcb_time(time.perf_counter)
        start=time.perf_counter()
        try:
            return original_run(thread)
        finally:
            report.update(wall_s=time.perf_counter()-start,end_sim_s=float(data.time))
            report['timers']={}
            for name,i in timers.items():
                elapsed=float(data.timer[i].duration)-before[name][0]
                calls=int(data.timer[i].number)-before[name][1]
                report['timers'][name]=dict(total_s=elapsed,calls=calls,
                    mean_ms=elapsed*1000/calls if calls else None)
            report['final_body']=dict(name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,node._body_id),
                                      position=data.xpos[node._body_id].tolist())
            mujoco.set_mjcb_time(callback)
            (folder/'native-loop.json').write_text(json.dumps(report,indent=2)+'\n')

    threading.Thread.run=observed_run
