#!/usr/bin/env python3
"""Does a scripted Gazebo actor push what it walks into? (ROADMAP R6.2, actor_collisions)

A person actor walks a straight 6 m line through a free 0.3 m box. The world
runs headless twice through the launch's own world copy: without and with
``world_with_actor_colliders``. The box pose comes from the world's
dynamic pose topic. With colliders the box should be pushed off the line; without, the
actor walks through it.

  python3 scripts/gazebo_actor_collision_check.py --out result.json
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
<sdf version="1.9"><world name="actor_check">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
  <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
  <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal>
    <size>40 40</size></plane></geometry></collision></link></model>
  <model name="box"><pose>0 0 0.15 0 0 0</pose>
    <link name="l"><inertial><mass>2</mass><inertia><ixx>0.03</ixx><iyy>0.03</iyy><izz>0.03</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision></link></model>
  <actor name="person_00"><skin><filename>%s</filename></skin>
    <script><loop>false</loop><trajectory id="0" type="walk">
      <waypoint><time>0</time><pose>-3 0.05 1 0 0 0</pose></waypoint>
      <waypoint><time>6</time><pose>3 0.05 1 0 0 0</pose></waypoint>
    </trajectory></script></actor>
</world></sdf>
'''


def load_launch():
    path = ROOT / 'src/robot_lab_description/launch/gazebo.launch.py'
    spec = importlib.util.spec_from_file_location('robot_lab_gazebo_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def box_track(world, seconds, env):
    server = subprocess.Popen(['ign', 'gazebo', '-s', '-r', world], env=env, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        time.sleep(2.)
        echo = subprocess.run(['ign', 'topic', '-e', '-t', '/world/actor_check/dynamic_pose/info', '-d', str(seconds)],
                              env=env, capture_output=True, text=True, timeout=seconds + 30)
    finally:
        os.killpg(server.pid, signal.SIGTERM)
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL)
            server.wait()
    track = []
    # Pose_V: one "pose { name: ... position { x y z } ... }" per dynamic entity.
    for block in re.split(r'\n(?=pose \{)', echo.stdout):
        if re.search(r'name: "box"', block):
            m = re.search(r'position \{\s*(?:x: (\S+))?\s*(?:y: (\S+))?', block)
            track.append((float(m.group(1) or 0.), float(m.group(2) or 0.)) if m else (0., 0.))
    return track


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--out', default='')
    options = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    skin = Path(get_package_share_directory('robot_lab_description')) / 'meshes/MaleVisitorWalkDownloaded.dae'
    env = dict(os.environ, IGN_PARTITION='actor_check_%d' % os.getpid())
    directory = Path(tempfile.mkdtemp(prefix='robot_lab_actor_check_'))
    base = directory / 'actor_check.sdf'
    base.write_text(WORLD % skin)
    launch = load_launch()
    with_colliders, added = launch.world_with_actor_colliders(str(base))
    result = dict(colliders=added)
    for label, world in (('without', str(base)), ('with', with_colliders)):
        track = box_track(world, options.seconds, env)
        final = track[-1] if track else None
        moved = (final[0] ** 2 + final[1] ** 2) ** .5 if final else None
        result[label] = dict(samples=len(track), final_xy=final, displacement_m=moved)
        print(label, 'samples', len(track), 'final', final, 'moved', moved)
    if options.out:
        Path(options.out).write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
