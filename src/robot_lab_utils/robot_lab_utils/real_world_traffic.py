"""People and traffic for an imported real-world area, from Eclipse SUMO (R6.8).

SUMO (EPL-2.0, ``pip install eclipse-sumo``) builds a road network with
sidewalks and crossings from the same OpenStreetMap data as the world
(``netconvert``), draws random vehicle and pedestrian trips (``randomTrips``)
and simulates them; its floating-car output (``--fcd-output.geo``) gives every
person and vehicle a longitude/latitude track. Those tracks become Gazebo
``<actor>`` scripts: people use the walking mesh the lab's other actor worlds
use, vehicles a car-sized box. Actors are Gazebo-only scenery: they follow
their recorded tracks, do not react to the robot, and the other simulators
skip them (their dynamics are recorded in the manifest).

The SUMO outputs are saved with the world (``inputs/traffic_fcd.xml``), so a
regeneration replays the same traffic without running SUMO.
"""
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

PERSON_SKIN = 'package://robot_lab_description/meshes/MaleVisitorWalkDownloaded.dae'
MAX_PEOPLE, MAX_VEHICLES = 40, 40
SEED = 42


def sumo_home():
    try:
        import sumo
        return Path(sumo.SUMO_HOME)
    except ImportError:
        home = os.environ.get('SUMO_HOME')
        if home and Path(home).is_dir():
            return Path(home)
    raise ValueError('People and traffic need Eclipse SUMO: pip install --user eclipse-sumo')


def _binary(name):
    for candidate in (shutil.which(name), str(Path(sys.executable).parent / name),
                      str(Path.home() / '.local' / 'bin' / name)):
        if candidate and Path(candidate).is_file():
            return candidate
    raise ValueError('%s not found; install Eclipse SUMO (pip install --user eclipse-sumo)' % name)


def osm_xml(osm):
    """Roads and their nodes as OSM XML for netconvert."""
    root = ET.Element('osm', version='0.6', generator='robot_lab real_world_traffic')
    ways = [e for e in osm['elements'] if e['type'] == 'way' and 'highway' in (e.get('tags') or {})]
    used = {node for way in ways for node in way.get('nodes', [])}
    for element in osm['elements']:
        if element['type'] == 'node' and element['id'] in used:
            ET.SubElement(root, 'node', id=str(element['id']), lat='%.7f' % element['lat'],
                          lon='%.7f' % element['lon'], version='1')
    for way in ways:
        node = ET.SubElement(root, 'way', id=str(way['id']), version='1')
        for reference in way.get('nodes', []):
            ET.SubElement(node, 'nd', ref=str(reference))
        for key, value in sorted((way.get('tags') or {}).items()):
            ET.SubElement(node, 'tag', k=key, v=value)
    return ET.tostring(root, encoding='unicode')


def run_sumo(osm, spec, work, progress=print):
    """netconvert + randomTrips + sumo in ``work``; returns the FCD XML text."""
    home = sumo_home()
    work = Path(work)
    (work / 'roads.osm').write_text(osm_xml(osm))
    run = lambda command: subprocess.run(command, cwd=work, check=True, capture_output=True, text=True, timeout=900)
    progress('Traffic: SUMO road network with sidewalks and crossings')
    run([_binary('netconvert'), '--osm-files', 'roads.osm', '-o', 'net.net.xml', '--geometry.remove',
         '--ramps.guess', '--junctions.join', '--tls.guess-signals', '--tls.discard-simple', '--tls.join',
         '--sidewalks.guess', '--crossings.guess', '--remove-edges.isolated', '--no-turnarounds',
         '--seed', str(SEED), '--no-warnings'])
    duration = int(spec['traffic_seconds'])
    tools = home / 'tools'
    routes = []
    if spec['traffic']:
        progress('Traffic: random vehicle trips')
        run([sys.executable, str(tools / 'randomTrips.py'), '-n', 'net.net.xml', '-o', 'vehicles.trips.xml',
             '-r', 'vehicles.rou.xml', '-e', str(duration), '-p', '%.3f' % (60. / spec['vehicles_per_minute']),
             '--seed', str(SEED), '--validate', '--fringe-factor', '5', '--min-distance', '50'])
        routes.append('vehicles.rou.xml')
    if spec['people']:
        progress('Traffic: random pedestrian walks')
        run([sys.executable, str(tools / 'randomTrips.py'), '-n', 'net.net.xml', '-o', 'people.rou.xml',
             '--pedestrians', '-e', str(duration), '-p', '%.3f' % (60. / spec['people_per_minute']),
             '--seed', str(SEED + 1), '--max-distance', '600', '--prefix', 'person'])
        routes.append('people.rou.xml')
    progress('Traffic: simulating %d s' % duration)
    run([_binary('sumo'), '-n', 'net.net.xml', '-r', ','.join(routes), '--end', str(duration),
         '--step-length', '0.5', '--fcd-output', 'fcd.xml', '--fcd-output.geo', 'true', '--seed', str(SEED),
         '--no-step-log', 'true', '--ignore-route-errors', 'true', '--no-warnings', 'true'])
    return (work / 'fcd.xml').read_text()


def tracks(fcd_text):
    """{(kind, id): [(time, lon, lat, angle_deg)]} from SUMO floating-car output."""
    found = {}
    for step in ET.fromstring(fcd_text).iter('timestep'):
        time_s = float(step.get('time'))
        for element in step:
            kind = 'person' if element.tag == 'person' else 'vehicle'
            found.setdefault((kind, element.get('id')), []).append(
                (time_s, float(element.get('x')), float(element.get('y')), float(element.get('angle', 0))))
    return found


def _actor(name, skin, scale, samples, start, loop_time, z_offset=0.):
    waypoints = ''.join(
        '            <waypoint><time>%.2f</time><pose>%.3f %.3f %.3f 0 0 %.4f</pose></waypoint>\n'
        % (t - start, x, y, z + z_offset, yaw) for t, x, y, z, yaw in samples)
    animation = ('      <animation name="walk"><filename>%s</filename><interpolate_x>false</interpolate_x>'
                 '</animation>\n' % skin)      # Gazebo binds the "walk" trajectory to this animation
    return ('    <actor name="%s">\n      <skin><filename>%s</filename><scale>%g</scale></skin>\n%s'
            '      <script>\n        <loop>true</loop>\n        <delay_start>%.2f</delay_start>\n'
            '        <auto_start>true</auto_start>\n        <trajectory id="0" type="walk" tension="0.4">\n%s'
            '        </trajectory>\n      </script>\n    </actor>\n'
            % (name, skin, scale, animation, start, waypoints))


def traffic_actors(spec, osm, projection, ground, folder=None, progress=print):
    """(actor SDF text, manifest summary, {relative path: bytes}) for the requested people/traffic."""
    saved = Path(folder) / 'inputs' / 'traffic_fcd.xml' if folder else None
    if saved and saved.is_file():
        fcd = saved.read_text()
    else:
        with tempfile.TemporaryDirectory(prefix='robot_lab_sumo_') as work:
            fcd = run_sumo(osm, spec, work, progress=progress)
    half = spec['size_m'] / 2 - 1.
    actors, counts = [], dict(person=0, vehicle=0)
    limits = dict(person=MAX_PEOPLE, vehicle=MAX_VEHICLES)
    car_skin = 'vehicle_box.dae'
    for (kind, identity), samples in sorted(tracks(fcd).items(), key=lambda item: item[1][0][0]):
        if counts[kind] >= limits[kind] or len(samples) < 4:
            continue
        times = np.array([s[0] for s in samples])
        x, y = projection.forward_many([s[2] for s in samples], [s[1] for s in samples])
        inside = (np.abs(x) < half) & (np.abs(y) < half)
        if inside.sum() < 4:
            continue
        keep = np.nonzero(inside)[0]
        keep = keep[np.append(np.diff(times[keep]) >= 0, True)]
        stride = max(1, int(round(1. / max(np.median(np.diff(times[keep])) if len(keep) > 1 else .5, 1e-3))))
        keep = np.unique(np.append(keep[::stride], keep[-1]))
        rows = [(float(times[i]), float(x[i]), float(y[i]), float(ground(x[i], y[i])),
                 math.radians(90. - samples[i][3])) for i in keep]
        counts[kind] += 1
        if kind == 'person':
            actors.append(_actor('person_%02d' % counts[kind], PERSON_SKIN, 1., rows, rows[0][0], 0.))
        else:
            actors.append(_actor('vehicle_%02d' % counts[kind], car_skin, 1., rows, rows[0][0], 0., z_offset=.0))
    files = {'inputs/traffic_fcd.xml': fcd.encode()}
    if counts['vehicle']:
        files['worlds/' + car_skin] = vehicle_box_dae()
    summary = dict(source='Eclipse SUMO %s (EPL-2.0), seed %d, %d s, step 0.5 s' % (sumo_version(), SEED,
                                                                                  spec['traffic_seconds']),
                   people=counts['person'], vehicles=counts['vehicle'],
                   scope='Gazebo actors replaying recorded SUMO tracks; they do not react to robots; '
                         'MuJoCo, PyBullet and Isaac skip actors.')
    return ''.join(actors), summary, files


def sumo_version():
    try:
        out = subprocess.run([_binary('sumo'), '--version'], capture_output=True, text=True, timeout=30).stdout
        return out.split('\n', 1)[0].split()[-1]
    except Exception:
        return 'unknown'


def vehicle_box_dae():
    """A 4.4 × 1.8 × 1.5 m car-sized box as a rigged Collada actor skin.

    Gazebo renders an actor only from a skinned mesh with a skeleton ("Mesh skeleton ... not found" for a
    plain mesh, 2026-11-13): the box is bound to one root bone, with a static two-key "walk" animation
    that the actor's trajectory drives.
    """
    hx, hy, h = 2.2, .9, 1.5
    corners = [(x, y, z) for z in (0., h) for y in (-hy, hy) for x in (-hx, hx)]
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
             (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    positions = ' '.join('%g %g %g' % c for c in corners)
    triangles = ' '.join('%d' % i for face in faces for i in face)
    identity = '1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1'
    text = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="box" name="box"><mesh>
      <source id="box-pos"><float_array id="box-pos-array" count="24">{positions}</float_array>
        <technique_common><accessor source="#box-pos-array" count="8" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common></source>
      <vertices id="box-vtx"><input semantic="POSITION" source="#box-pos"/></vertices>
      <triangles count="12"><input semantic="VERTEX" source="#box-vtx" offset="0"/><p>{triangles}</p></triangles>
    </mesh></geometry>
  </library_geometries>
  <library_controllers>
    <controller id="box-skin"><skin source="#box">
      <bind_shape_matrix>{identity}</bind_shape_matrix>
      <source id="box-joints"><Name_array id="box-joints-array" count="1">Root</Name_array>
        <technique_common><accessor source="#box-joints-array" count="1"><param name="JOINT" type="name"/></accessor></technique_common></source>
      <source id="box-binds"><float_array id="box-binds-array" count="16">{identity}</float_array>
        <technique_common><accessor source="#box-binds-array" count="1" stride="16"><param name="TRANSFORM" type="float4x4"/></accessor></technique_common></source>
      <source id="box-weights"><float_array id="box-weights-array" count="1">1</float_array>
        <technique_common><accessor source="#box-weights-array" count="1"><param name="WEIGHT" type="float"/></accessor></technique_common></source>
      <joints><input semantic="JOINT" source="#box-joints"/><input semantic="INV_BIND_MATRIX" source="#box-binds"/></joints>
      <vertex_weights count="8"><input semantic="JOINT" source="#box-joints" offset="0"/><input semantic="WEIGHT" source="#box-weights" offset="1"/>
        <vcount>{' '.join(['1'] * 8)}</vcount><v>{' '.join(['0 0'] * 8)}</v></vertex_weights>
    </skin></controller>
  </library_controllers>
  <library_animations>
    <animation id="Root-anim">
      <source id="Root-time"><float_array id="Root-time-array" count="2">0 1</float_array>
        <technique_common><accessor source="#Root-time-array" count="2"><param name="TIME" type="float"/></accessor></technique_common></source>
      <source id="Root-pose"><float_array id="Root-pose-array" count="32">{identity} {identity}</float_array>
        <technique_common><accessor source="#Root-pose-array" count="2" stride="16"><param name="TRANSFORM" type="float4x4"/></accessor></technique_common></source>
      <source id="Root-interp"><Name_array id="Root-interp-array" count="2">LINEAR LINEAR</Name_array>
        <technique_common><accessor source="#Root-interp-array" count="2"><param name="INTERPOLATION" type="name"/></accessor></technique_common></source>
      <sampler id="Root-sampler"><input semantic="INPUT" source="#Root-time"/><input semantic="OUTPUT" source="#Root-pose"/>
        <input semantic="INTERPOLATION" source="#Root-interp"/></sampler>
      <channel source="#Root-sampler" target="Root/transform"/>
    </animation>
  </library_animations>
  <library_visual_scenes>
    <visual_scene id="scene">
      <node id="Root" name="Root" sid="Root" type="JOINT"><matrix sid="transform">{identity}</matrix></node>
      <node id="vehicle" name="vehicle"><instance_controller url="#box-skin"><skeleton>#Root</skeleton></instance_controller></node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""
    return text.encode()
