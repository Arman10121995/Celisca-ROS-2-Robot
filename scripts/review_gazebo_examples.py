#!/usr/bin/env python3
"""Behaviour and resource review of the Gazebo Fortress example worlds (ROADMAP R6.6).

Every SDF under ``examples/worlds`` of the pinned gazebo_examples checkout
is read and classified from its own content:

- **resources:** included model URIs (Fuel downloads, ``model://``, files)
  and mesh URIs, with whether each resolves locally;
- **behaviour:** system plugins (world and model level), models with joints
  driven by a controller plugin (a robot demo), actors, sensors, lights;
- **load:** an optional headless Gazebo run (``ign gazebo -s -r
  --iterations N``) records whether the world loads and the errors it prints.

The class decides how the lab may use it: ``environment`` (a scene:
installable as a world), ``robot_demo``, ``actor_demo``, ``sensor_demo`` or
``plugin_demo`` (feature fixtures, not maps). ``needs_fuel`` marks worlds
whose models come from Gazebo Fuel; their licences are recorded per model by
``scripts/fuel_models.py`` before any is installed.

  python3 scripts/review_gazebo_examples.py --load --json review.json --markdown review.md
"""
import argparse
import json
import os
import re
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

DRIVERS = re.compile(r'(diff.?drive|ackermann|joint.?(position|trajectory|controller)|velocity.?control|'
                     r'multicopter|thruster|tracked|mecanum|apply.?joint.?force|lift.?drag|wheel|odometry)', re.I)
# Systems a plain world needs; anything else is a feature this example demonstrates.
BASIC_SYSTEMS = {'physics', 'user-commands', 'usercommands', 'scene-broadcaster', 'scenebroadcaster',
                 'sensors', 'contact', 'imu', 'forcetorque'}
BENIGN = re.compile(r'libEGL warning|MESA-LOADER|egl: failed to create dri2 screen')
ERROR = re.compile(r'\[(Err|err)\]|Error|error while|Unable to find|Failed to', re.I)


def plugin_name(plugin):
    name = (plugin.get('filename') or plugin.get('name') or '').lower()
    return re.sub(r'^(lib)?(ignition-gazebo-|gz-sim-|ignition-gazebo\d*-)', '', name).replace('-system', '').replace('.so', '')


def resolve(uri, base, models_roots):
    if uri.startswith(('http://', 'https://')):
        return 'fuel', None
    if uri.startswith('model://'):
        name, _, rest = uri[8:].partition('/')
        for root in models_roots:
            candidate = Path(root) / name / rest if rest else Path(root) / name
            if candidate.exists():
                return 'model', candidate
        return 'model', None
    if uri.startswith('file://'):
        path = Path(uri[7:])
        return 'file', path if path.exists() else None
    path = (Path(base) / uri).resolve()
    return 'file', path if path.exists() else None


def parse_sdf(path):
    """SDF the way Gazebo's parser tolerates it: comments may contain '--', attributes a gz/ignition prefix."""
    text = Path(path).read_text(errors='replace')
    if not text.strip():
        raise ValueError('empty file')
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    text = re.sub(r'(\s)(?:ignition|gz):([A-Za-z_][\w.-]*=)', r'\1\2', text)
    return ET.fromstring(text)


def fuel_model_sdf(uri):
    """The cached model.sdf (Gazebo's Fuel cache) for a Fuel model URI, or None."""
    import urllib.parse
    parts = [urllib.parse.unquote(p) for p in urllib.parse.urlparse(uri).path.split('/') if p]
    if parts and parts[0] == '1.0':
        parts = parts[1:]
    if len(parts) < 3:
        return None
    # Gazebo caches each Fuel host separately (the old fuel.ignitionrobotics.org included).
    hosts = [urllib.parse.urlparse(uri).netloc, 'fuel.gazebosim.org', 'fuel.ignitionrobotics.org']
    versions = []
    for host in hosts:
        root = Path.home() / '.ignition/fuel' / host / parts[0].lower() / 'models' / parts[2].lower()
        if root.is_dir():
            versions = sorted((p for p in root.glob('*') if p.name.isdigit()), key=lambda p: int(p.name))
            if versions:
                break
    wanted = [p for p in versions if len(parts) > 3 and p.name == parts[3]]
    chosen = (wanted or versions[-1:] or [None])[0]
    return chosen / 'model.sdf' if chosen and (chosen / 'model.sdf').is_file() else None


def review(path, models_roots):
    root = parse_sdf(path)
    world = root.find('world')
    if world is None:
        return dict(file=Path(path).name, kind='not_a_world')
    # Models a world includes from Fuel count as its own: open the cached copies.
    inspected, unreviewed = [], []
    for include in world.findall('include'):
        uri = (include.findtext('uri') or '').strip()
        if uri.startswith(('http://', 'https://')) and '/files/' not in uri:
            sdf = fuel_model_sdf(uri)
            if sdf is None:
                unreviewed.append(uri)
                continue
            try:
                included = parse_sdf(sdf)
            except (ET.ParseError, ValueError):
                unreviewed.append(uri)
                continue
            for element in list(included):
                if element.tag in ('model', 'actor'):
                    # The included model's relative files live in its own directory.
                    for node in element.iter('uri'):
                        value = (node.text or '').strip()
                        if value and '://' not in value and not value.startswith('/'):
                            node.text = str(sdf.parent / value)
                    # Plugins written on the <include> drive the included model.
                    for plugin in include.findall('plugin'):
                        element.append(plugin)
                    world.append(element)
                    inspected.append(uri)
    world_plugins = sorted({plugin_name(p) for p in world.findall('plugin')})
    model_plugins, driven, sensors = set(), [], set()
    for model in world.iter('model'):
        names = {plugin_name(p) for p in model.findall('plugin')}
        model_plugins |= names
        if model.findall('joint') and any(DRIVERS.search(n) for n in names):
            driven.append(model.get('name'))
    for sensor in world.iter('sensor'):
        sensors.add(sensor.get('type'))
    includes, unresolved, fuel = [], [], []
    for uri in [u.text.strip() for u in world.iter('uri') if u.text and u.text.strip()]:
        kind, target = resolve(uri, Path(path).parent, models_roots)
        if kind == 'fuel':
            fuel.append(uri)
        elif target is None and not uri.startswith(('__', 'file://media/')):   # media/ is Gazebo's own
            unresolved.append(uri)
        includes.append(uri)
    actors = [a.get('name') for a in world.iter('actor')]
    extra = sorted(n for n in set(world_plugins) | model_plugins if n and n not in BASIC_SYSTEMS)
    if driven:
        kind = 'robot_demo'
    elif actors:
        kind = 'actor_demo'
    elif unresolved:
        kind = 'unresolved_resources'
    elif sensors - {None}:
        kind = 'sensor_demo'
    elif extra:
        kind = 'plugin_demo'
    else:
        kind = 'environment'
    return dict(file=Path(path).name, world=world.get('name'), kind=kind, needs_fuel=bool(fuel),
                fuel_models_inspected=len(set(inspected)), fuel_models_unreviewed=sorted(set(unreviewed)),
                models=len(list(world.iter('model'))),
                world_plugins=world_plugins, extra_plugins=extra, driven_models=driven, actors=len(actors),
                sensors=sorted(s for s in sensors if s), fuel_uris=sorted(set(fuel)), unresolved=sorted(set(unresolved)),
                lights=len(world.findall('light')))


def load(path, iterations=200, timeout=40):
    """Headless load in its own transport partition; (ok, seconds, first errors)."""
    env = dict(os.environ, IGN_PARTITION='review_%d_%d' % (os.getpid(), time.monotonic_ns() % 100000))
    started = time.monotonic()
    with tempfile.TemporaryFile('w+') as log:
        process = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '--iterations', str(iterations), str(path)],
                                   env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            code = 'timeout'
        log.seek(0)
        lines = [re.sub(r'\x1b\[[0-9;]*m', '', l).strip() for l in log.read().splitlines()]
    # The Jetson's EGL prints harmless driver-probe warnings for every rendering sensor.
    errors = [l for l in lines if ERROR.search(l) and not BENIGN.search(l)][:5]
    return dict(exit=code, ok=code == 0 and not errors, seconds=round(time.monotonic() - started, 1), errors=errors)


def markdown(rows):
    counts = {}
    for r in rows:
        counts[r['kind']] = counts.get(r['kind'], 0) + 1
    lines = ['# Gazebo Fortress example worlds: behaviour and resource review', '',
             'Generated by `scripts/review_gazebo_examples.py` from each SDF and a headless load.', '',
             'Classes: ' + ', '.join('%s %d' % kv for kv in sorted(counts.items())) + '.', '',
             '| World | Class | Feature plugins | Driven models | Actors | Sensors | Fuel models | Load |',
             '|---|---|---|---|---|---|---|---|']
    for r in rows:
        load_text = '-' if 'load' not in r else ('ok %.0f s' % r['load']['seconds'] if r['load']['ok'] else
                                                 'FAIL (%s) %s' % (r['load']['exit'], (r['load']['errors'] or [''])[0][:80]))
        lines.append('| `%s` | %s | %s | %s | %d | %s | %d | %s |' % (
            r['file'], r['kind'], ', '.join(r.get('extra_plugins', []))[:90] or '-', ', '.join(r.get('driven_models', [])) or '-',
            r.get('actors', 0), ', '.join(r.get('sensors', [])) or '-', len(r.get('fuel_uris', [])),
            load_text.replace('|', '/')))
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--root', default='')
    parser.add_argument('--load', action='store_true')
    parser.add_argument('--json', default='')
    parser.add_argument('--markdown', default='')
    options = parser.parse_args()
    if options.root:
        root = Path(options.root)
    else:
        store = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'external_assets'
        root = sorted((store / 'sources' / 'gazebo_examples').glob('*/examples/worlds'))[-1]
    models_roots = [p for p in os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '').split(':') if p] + [str(root.parent / 'models')]
    rows = []
    for path in sorted(root.glob('*.sdf')):
        try:
            row = review(path, models_roots)
        except (ET.ParseError, ValueError) as exc:
            row = dict(file=path.name, kind='empty' if str(exc) == 'empty file' else 'unparsable', error=str(exc))
        if options.load and row['kind'] not in ('unparsable', 'not_a_world', 'empty'):
            row['load'] = load(path)
        rows.append(row)
        print(row['file'], row['kind'], row.get('load', {}).get('ok', ''), flush=True)
    if options.json:
        Path(options.json).write_text(json.dumps(dict(root=str(root), worlds=rows), indent=1) + '\n')
    if options.markdown:
        Path(options.markdown).write_text(markdown(rows))


if __name__ == '__main__':
    main()
