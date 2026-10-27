"""Distinct PX4 identities and bounded 3D group kernels (no plant pose writes)."""
import math
import numpy as np


def identities(count):
    if type(count) is not int or not 2 <= count <= 4:
        raise ValueError('Aerial sessions support 2–4 complete X500/FCU instances')
    return [dict(id='air'+str(i+1),instance=i,system_id=i+1,port=14580+i,
                 entity='x500_'+str(i),namespace='/swarm/air'+str(i+1)) for i in range(count)]


def prepare_session(session_file, map_override='', runtime_root=None, px4_root=None):
    """Resolve installed worlds and actual model envelopes before spawning."""
    import hashlib
    import json
    import os
    from pathlib import Path
    import tempfile
    import yaml
    from ament_index_python.packages import get_package_share_directory as share
    from robot_lab_utils.installed_assets import merge_installed_profiles
    from robot_lab_utils.fleet_session import SpawnGrid
    from robot_lab_utils.aerial_planning import StaticScene
    from robot_lab_utils.aerial_takeoff import check_takeoff, x500_envelope
    requested = yaml.safe_load(Path(session_file).read_text()) or {}
    raw_members = requested.get('members')
    if not isinstance(raw_members, list):
        raise ValueError('Aerial session needs 2–4 member records')
    members = identities(len(raw_members))
    selected = map_override or requested.get('map', 'nav_empty')
    maps = merge_installed_profiles(yaml.safe_load((Path(share('robot_lab_bringup'))/
        'config/sim_maps.yaml').read_text()), 'maps')['maps']
    if selected not in maps:
        raise ValueError('Choose an installed world: '+str(selected))
    profile = maps[selected]
    gz = profile.get('gazebo', {})
    def resolve(package, value):
        value = Path(value)
        return value if value.is_absolute() else Path(share(package))/value
    world = resolve(gz.get('world_package', 'robot_lab_maps'), gz['world_path'])
    if not world.is_file():
        raise ValueError('Selected world is unavailable: '+str(world))
    root = Path(px4_root or os.environ.get('PX4_ROOT', '/workspace/molar/px4/PX4-Autopilot'))
    envelope = x500_envelope(root/'Tools/simulation/gz/models')
    scene = StaticScene(world)
    metadata = profile.get('map', {})
    grid = (SpawnGrid(resolve(metadata.get('package', 'robot_lab_maps'), metadata['path']))
            if metadata.get('has_2d_map') else None)
    seed = profile.get('spawn', {})
    sx, sy = float(seed.get('x', 0.)), float(seed.get('y', 0.))
    spacing = float(requested.get('spacing', 3.))
    if not math.isfinite(spacing) or not 3. <= spacing <= 8.:
        raise ValueError('Spawn spacing must be 3–8 m')
    offsets = sorted(((x, y) for x in range(-3, 4) for y in range(-3, 4)),
                     key=lambda p: (p[0]**2+p[1]**2, p[0], p[1]))
    placed = []
    for item, supplied in zip(members, raw_members):
        if not isinstance(supplied, dict) or supplied.get('id', item['id']) != item['id']:
            raise ValueError('Aerial member ids must be air1, air2, air3, air4 in order')
        spawn = supplied.get('spawn') or {}
        if ('x' in spawn) != ('y' in spawn):
            raise ValueError('Supply both spawn X and Y, or leave both automatic')
        explicit = 'x' in spawn
        candidates = [(float(spawn['x']), float(spawn['y']))] if explicit else [
            (sx+x*spacing, sy+y*spacing) for x, y in offsets]
        z = float(spawn.get('z', requested.get('spawn_z', 0.)))
        if not math.isfinite(z) or abs(z)>500.:
            raise ValueError('Spawn Z must be finite and within the world height range')
        last_error = 'No separated free spawn fits the chosen world'
        for x, y in candidates:
            if not all(math.isfinite(v) for v in (x, y)):
                raise ValueError('Finite spawn XYZ required')
            if any(math.dist((x, y, z), (p['x'], p['y'], p['z'])) < spacing-.01 for p in placed):
                continue
            if grid is not None and not grid.clear(x, y, envelope['upper'][0]+.08):
                continue
            try:
                check = check_takeoff(scene, [x, y, z+.24], envelope, desired_rise=.8,
                                      minimum_rise=.8, timeout=3.)
            except (ValueError, TimeoutError) as exc:
                last_error = str(exc)
                continue
            item['spawn'] = dict(x=x, y=y, z=z)
            item['spawn_geometry_check'] = check
            placed.append(item['spawn'])
            break
        else:
            raise ValueError(item['id']+': '+last_error+'. Edit explicit member XYZ for this world.')
    runtime = Path(runtime_root or os.environ.get('ROBOT_LAB_RUNTIME_ROOT',
        '/workspace/molar/robot_lab_runtime'))
    base = runtime/'aerial_sessions'
    base.mkdir(parents=True, exist_ok=True)
    if base.stat().st_dev == Path('/').stat().st_dev:
        raise ValueError('Aerial sessions and logs must stay on the mounted workspace SSD')
    folder = Path(tempfile.mkdtemp(prefix='swarm-', dir=base))
    session = dict(schema_version=1, session_id=folder.name, directory=str(folder), map=selected,
        source_world=str(world), world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        members=members, spacing=spacing, source_session=str(Path(session_file).resolve()),
        scope='Implemented experimental native X500 multi-FCU session; actual flight qualification is separate')
    session['resolved_file'] = str(folder/'session.json')
    Path(session['resolved_file']).write_text(json.dumps(session, indent=2)+'\n')
    return session


def velocities(mode, positions, spacing=3., anchor=None, maximum=.4):
    if mode not in ('formation','rendezvous','dispersion','flocking') or not 2 <= len(positions) <= 4:
        raise ValueError('Select a supported 3D swarm kernel and 2–4 fresh members')
    if not math.isfinite(spacing) or not 2. <= spacing <= 8.:
        raise ValueError('Aerial spacing must be 2–8 m')
    names=sorted(positions);p=np.asarray([positions[n] for n in names],dtype=float)
    if p.shape!=(len(names),3) or not np.isfinite(p).all():raise ValueError('Fresh finite XYZ positions required')
    centre=np.mean(p,axis=0);anchor=centre if anchor is None else np.asarray(anchor,dtype=float)
    if anchor.shape!=(3,) or not np.isfinite(anchor).all():raise ValueError('Finite XYZ anchor required')
    result={}
    for index,name in enumerate(names):
        if mode=='formation':
            target=anchor+np.array([0.,(index-(len(names)-1)/2)*spacing,0.])
            v=.5*(target-p[index])
        elif mode=='rendezvous':v=.3*(centre-p[index])
        elif mode=='dispersion':v=np.zeros(3)
        else:v=.15*(centre-p[index])+np.array([.2,0.,0.])
        for other in range(len(names)):
            if other==index:continue
            delta=p[index]-p[other];distance=float(np.linalg.norm(delta))
            if distance<1e-6:raise ValueError('Coincident aerial estimates; Hold before restarting')
            if distance<spacing:
                v+=(spacing-distance)*delta/distance*.5
        speed=float(np.linalg.norm(v))
        if speed>maximum:v*=maximum/speed
        result[name]=v.tolist()
    return result
