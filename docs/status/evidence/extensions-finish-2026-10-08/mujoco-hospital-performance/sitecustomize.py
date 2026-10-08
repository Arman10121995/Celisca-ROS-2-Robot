"""Opt-in timing observation of the unchanged MuJoCo physics thread.

Put this directory first on PYTHONPATH for a single bounded diagnostic and set
ROBOT_LAB_PROFILE_DIRECTORY to a fresh SSD directory. No controller, command,
rate, collision, sensor or model parameter is changed. Profiling overhead means
these timings are a separate stage from the uninstrumented RTF measurement.
"""
import cProfile
import hashlib
import json
import os
from pathlib import Path
import pstats
import sys
import threading
import time


if (Path(sys.argv[0]).name == 'mujoco_spawner'
        and os.environ.get('ROBOT_LAB_PROFILE_DIRECTORY')):
    original_run = threading.Thread.run

    def observed_run(thread):
        target = getattr(thread, '_target', None)
        if (getattr(target, '__name__', '') != '_loop'
                or getattr(target, '__module__', '') != 'robot_lab_mujoco.mujoco_spawner'):
            return original_run(thread)
        folder = Path(os.environ['ROBOT_LAB_PROFILE_DIRECTORY'])
        assert folder.is_dir() and folder.stat().st_dev != Path('/').stat().st_dev
        import mujoco
        node = target.__self__
        model = node._model
        model_path = folder/'observed-scene.xml'
        mujoco.mj_saveLastXML(str(model_path), model)
        source = Path(sys.modules[target.__module__].__file__)
        scope = dict(scope='Instrumented stationary normal GUI Labbot/MuJoCo/hospital v4 physics loop; profiling overhead retained. No speed fix or navigation mission.',
            argv=sys.argv, source_path=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            hook_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            engine_version=mujoco.__version__, timestep=model.opt.timestep,
            model_counts=dict(nbody=model.nbody, ngeom=model.ngeom, nmesh=model.nmesh, nflex=model.nflex),
            scene=dict(path=str(model_path), bytes=model_path.stat().st_size, sha256=hashlib.sha256(model_path.read_bytes()).hexdigest()),
            sensor_parameters={name:node.get_parameter(name).value for name in
                ('publish_rate','scan_rate','scan_samples','camera_rate','camera_width','camera_height')},
            viewer_present=node._viewer is not None, start_sim_s=node._data.time)
        profiler = cProfile.Profile()
        wall_start = time.monotonic()
        try:
            return profiler.runcall(original_run, thread)
        finally:
            scope.update(wall_s=time.monotonic()-wall_start, end_sim_s=node._data.time)
            profiler.dump_stats(str(folder/'physics-loop.pstats'))
            stats = pstats.Stats(profiler)
            rows=[]
            for (filename,line,function),(primitive,calls,self_s,cumulative_s,callers) in stats.stats.items():
                rows.append(dict(filename=filename,line=line,function=function,calls=calls,
                                 self_s=self_s,cumulative_s=cumulative_s))
            scope['timings']=sorted(rows,key=lambda r:r['cumulative_s'],reverse=True)
            (folder/'profile.json').write_text(json.dumps(scope,indent=2)+'\n')

    threading.Thread.run = observed_run
