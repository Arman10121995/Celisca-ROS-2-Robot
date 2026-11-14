#!/usr/bin/env python3
"""Does sim_seed make Gazebo's sensor noise repeat? (ROADMAP R7, simulator-applied seeds)

Runs a headless Gazebo world holding one noisy IMU, once per requested seed
(``none`` = no seed system), with the world built by the same
``world_with_seed`` the launch uses. Each run starts paused, an ``ign topic``
echo subscribes, the world is unpaused, and the IMU samples are keyed by their
simulation stamp. Runs are then compared on the stamps they share.

  python3 scripts/gazebo_seed_check.py --seeds 42 42 7 none none --out result.json
"""
import argparse
import importlib.util
import json
import os
import re
import signal
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD = '''<?xml version="1.0"?>
<sdf version="1.9"><world name="seed_check">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
  <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
  <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
  <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
  <model name="box"><static>true</static><link name="link">
    <sensor name="imu" type="imu"><always_on>1</always_on><update_rate>100</update_rate><topic>seed_check/imu</topic>
      <imu>%s</imu></sensor></link></model>
</world></sdf>
'''
AXIS = '<%s><noise type="gaussian"><mean>0</mean><stddev>%g</stddev><bias_mean>%g</bias_mean><bias_stddev>%g</bias_stddev></noise></%s>'


def world_text():
    def block(kind, std, bias):
        return '<%s>%s</%s>' % (kind, ''.join(AXIS % (a, std, 0., bias, a) for a in 'xyz'), kind)
    return WORLD % (block('angular_velocity', .01, .005) + block('linear_acceleration', .05, .02))


def load_launch():
    path = ROOT / 'src/robot_lab_description/launch/gazebo.launch.py'
    spec = importlib.util.spec_from_file_location('robot_lab_gazebo_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse(text):
    """Text-format Imu messages -> {stamp_ns: (wx, wy, wz, ax, ay, az)}."""
    samples = {}
    for message in re.split(r'\n(?=header \{)', text):
        stamp = re.search(r'stamp \{\s*(?:sec: (\d+))?\s*(?:nsec: (\d+))?', message)
        vectors = [re.search(r'%s \{([^}]*)\}' % k, message) for k in ('angular_velocity', 'linear_acceleration')]
        if not stamp or not all(vectors):
            continue
        values = []
        for vector in vectors:
            for axis in 'xyz':
                m = re.search(r'\b%s: (\S+)' % axis, vector.group(1))
                values.append(float(m.group(1)) if m else 0.)
        samples[int(stamp.group(1) or 0) * 10 ** 9 + int(stamp.group(2) or 0)] = tuple(values)
    return samples


def run(world, count, env):
    server = subprocess.Popen(['ign', 'gazebo', '-s', world], env=env, stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE, text=True, start_new_session=True)
    try:
        time.sleep(4.)
        echo = subprocess.Popen(['ign', 'topic', '-e', '-t', '/seed_check/imu', '-n', str(count)], env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
        time.sleep(1.5)
        subprocess.run(['ign', 'service', '-s', '/world/seed_check/control', '--reqtype', 'ignition.msgs.WorldControl',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', 'pause: false'],
                       env=env, capture_output=True, timeout=10)
        try:
            out, _ = echo.communicate(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(echo.pid, signal.SIGTERM)
            out, _ = echo.communicate()
        return parse(out)
    finally:
        os.killpg(server.pid, signal.SIGTERM)
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL)
            server.wait()


def compare(a, b):
    shared = sorted(set(a) & set(b))
    identical = sum(a[s] == b[s] for s in shared)
    return dict(shared=len(shared), identical=identical)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--seeds', nargs='+', default=['42', '42', '7', 'none', 'none'])
    parser.add_argument('--count', type=int, default=300)
    parser.add_argument('--out', default='')
    options = parser.parse_args()
    env = dict(os.environ, IGN_PARTITION='seed_check_%d' % os.getpid())
    directory = Path(tempfile.mkdtemp(prefix='robot_lab_seed_check_'))
    base = directory / 'seed_check.sdf'
    base.write_text(world_text())
    launch = load_launch()
    runs = []
    for seed in options.seeds:
        world = str(base) if seed == 'none' else launch.world_with_seed(str(base), int(seed))
        samples = run(world, options.count, env)
        runs.append(dict(seed=seed, samples=samples))
        print('seed %s: %d samples' % (seed, len(samples)))
    pairs = []
    for i in range(len(runs)):
        for j in range(i + 1, len(runs)):
            pairs.append(dict(a=runs[i]['seed'], b=runs[j]['seed'], **compare(runs[i]['samples'], runs[j]['samples'])))
    for pair in pairs:
        print('%(a)s vs %(b)s: %(identical)d of %(shared)d shared stamps identical' % pair)
    if options.out:
        first = {r['seed']: [list(v) for _, v in sorted(r['samples'].items())[:3]] for r in runs}
        Path(options.out).write_text(json.dumps(dict(seeds=options.seeds, samples=[len(r['samples']) for r in runs],
                                                     pairs=pairs, first_samples=first), indent=2) + '\n')


if __name__ == '__main__':
    main()
