#!/usr/bin/env python3
"""Isaac Sim marine plant for the native ROV force interface (ROADMAP R10.4).

The ROS side of ``marine_isaac_runtime``: the same topics as the MuJoCo
and PyBullet plants (``marine_mujoco_plant.run_plant``), so the unchanged
``marine_controller`` runs on Isaac PhysX. Each period it sends the
thruster forces and the current to the Isaac child, which steps the
ported water forces, and reads back the pose.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np

from robot_lab_utils.marine_mujoco_plant import WaterForces, plant_arguments, run_plant


STARTUP_TIMEOUT_S = 180.   # Isaac Sim boot
STEP_TIMEOUT_S = 10.


class MarineIsaacPlant(WaterForces):
    engine = 'isaac'
    scope = 'Isaac PhysX (CPU) rigid body with ported Fortress buoyancy/hydrodynamics/thruster forces; seabed only'

    def __init__(self, description, spawn, surface_z, bottom_z, density=1025., isaac_python=None):
        from robot_lab_utils.isaac_env import find_isaac_python, isaac_preload_libraries
        self._load(description, surface_z, bottom_z, density)
        python = isaac_python or find_isaac_python()
        if not python or not Path(python).exists():
            raise RuntimeError('No Isaac Sim Python found (set ISAAC_PYTHON)')
        env = dict(os.environ, ACCEPT_EULA='YES', OMNI_KIT_ACCEPT_EULA='YES')
        env['LD_PRELOAD'] = isaac_preload_libraries(python, env.get('LD_PRELOAD', ''))
        fifo = tempfile.mkstemp(prefix='marine_isaac_', dir=tempfile.gettempdir())[1]
        os.unlink(fifo)
        os.mkfifo(fifo)
        env['ISAAC_EVENT_FIFO'] = fifo
        # The installed launcher script lives outside the package: locate the
        # runtime and the package's site directory from the module itself.
        import importlib.util
        import robot_lab_utils
        runtime = importlib.util.find_spec('robot_lab_utils.marine_isaac_runtime').origin
        # Kit's console output (and, on this Jetson, GPU PhysX's failed CUDA
        # module loads: the CPU solver runs) goes to a log, not the console.
        log_dir = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', tempfile.gettempdir()))
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log = open(log_dir / ('marine_isaac_runtime_%d.log' % os.getpid()), 'w')
        self.process = subprocess.Popen([python, '-u', runtime], stdin=subprocess.PIPE,
                                        stdout=self.log, stderr=subprocess.STDOUT, env=env, text=True)
        package_root = str(Path(robot_lab_utils.__file__).resolve().parent.parent)   # its site directory
        self.process.stdin.write(json.dumps(dict(description=description, spawn=[float(v) for v in spawn],
                                                 surface_z=float(surface_z), bottom_z=float(bottom_z),
                                                 density=float(density), pythonpath=[package_root])) + '\n')
        self.process.stdin.flush()
        # Non-blocking: a child that dies before opening the FIFO must not hang us.
        self.events = os.fdopen(os.open(fifo, os.O_RDONLY | os.O_NONBLOCK))
        self.buffer = ''
        ready = self._event(timeout=STARTUP_TIMEOUT_S)
        if ready.get('event') != 'ready':
            raise RuntimeError('Isaac marine runtime failed: %s' % ready)
        self.time = 0.
        self.position = np.asarray(spawn[:3], dtype=float)
        yaw = float(spawn[3])
        self.quaternion = np.array([0., 0., np.sin(yaw / 2), np.cos(yaw / 2)])

    def _event(self, timeout=STEP_TIMEOUT_S):
        import select
        import time
        deadline = time.monotonic() + timeout
        while '\n' not in self.buffer:
            if self.process.poll() is not None:
                raise RuntimeError('Isaac marine runtime exited (%s); see %s' % (self.process.returncode, self.log.name))
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError('Isaac marine runtime did not answer within %.0f s' % timeout)
            ready, _, _ = select.select([self.events], [], [], min(remaining, .5))
            if ready:
                self.buffer += self.events.read() or ''
        line, self.buffer = self.buffer.split('\n', 1)
        return json.loads(line)

    def pose(self):
        return self.position.copy(), self.quaternion.copy()

    def step(self, dt):
        self.process.stdin.write(json.dumps(dict(forces=self.forces.tolist(), current=list(map(float, self.current)),
                                                 dt=float(dt))) + '\n')
        self.process.stdin.flush()
        state = self._event()
        if state.get('event') != 'state':
            raise RuntimeError('Isaac marine runtime: %s' % state)
        self.time = float(state['t'])
        self.position = np.asarray(state['position'], dtype=float)
        self.quaternion = np.asarray(state['quaternion_xyzw'], dtype=float)
        self.wrench = np.asarray(state['wrench'], dtype=float)
        return self.time

    def close(self):
        try:
            self.process.stdin.write(json.dumps(dict(stop=True)) + '\n')
            self.process.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass
        self.process.wait(timeout=20)


def main(args=None):
    import rclpy
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    rclpy.init(args=args)
    plant = MarineIsaacPlant(*plant_arguments())
    try:
        run_plant(plant, 'marine_isaac_plant')
    finally:
        plant.close()


if __name__ == '__main__':
    main()
