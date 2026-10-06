"""Cap a compliant robot's physics step without replacing its selected world."""
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def cap_physics_step(world_file, maximum_step, runtime_root=None):
    source = Path(world_file).resolve()
    maximum = float(maximum_step)
    if not math.isfinite(maximum) or not 0 < maximum <= .02:
        raise ValueError('Gazebo physics maximum step must be finite and within (0, 0.02] seconds')
    data = source.read_bytes()
    root = ET.fromstring(data)
    worlds = root.findall('world')
    if len(worlds) != 1:
        raise ValueError('Physics preparation requires one selected SDF world')
    world = worlds[0]
    profiles = world.findall('physics')
    physics = next((p for p in profiles if p.get('default', '').lower() in ('true', '1')),
                   profiles[0] if profiles else None)
    step = physics.find('max_step_size') if physics is not None else None
    # SDF's default is 1 ms, which already meets this controller's bound.
    original = float(step.text) if step is not None else .001
    if not math.isfinite(original) or original <= 0:
        raise ValueError('Selected world physics step must be finite and positive')
    if original <= maximum:
        return str(source)
    if physics is None:
        physics = ET.SubElement(world, 'physics', name='robot_lab_physics', type='ode')
    if step is None:
        step = ET.SubElement(physics, 'max_step_size')
    step.text = format(maximum, '.12g')
    # A cached derivative lives away from the source. Preserve local mesh,
    # include, heightmap and texture resources through absolute file URIs.
    resources = []
    for element in world.iter():
        if element.tag not in ('uri', 'image', 'diffuse', 'normal', 'albedo_map', 'normal_map'):
            continue
        text = (element.text or '').strip()
        if not text or '://' in text or Path(text).is_absolute():
            continue
        local = source.parent/text
        if local.exists():
            element.text = local.resolve().as_uri()
            resources.append(dict(original=text, resolved=element.text))
    payload = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    identity = hashlib.sha256(data+str(source).encode()+payload).hexdigest()[:20]
    base = Path(runtime_root or os.environ.get('ROBOT_LAB_RUNTIME_ROOT',
                '/workspace/molar/robot_lab_runtime'))/'gazebo_worlds'
    folder = base/(source.stem+'-'+identity)
    folder.mkdir(parents=True, exist_ok=True)
    target = folder/source.name
    if not target.is_file() or target.read_bytes() != payload:
        temporary = folder/(source.name+'.partial')
        temporary.write_bytes(payload)
        temporary.replace(target)
    (folder/'manifest.json').write_text(json.dumps(dict(source=str(source),
        source_sha256=hashlib.sha256(data).hexdigest(), world_name=world.get('name'),
        source_max_step_s=original, effective_max_step_s=maximum,
        output_sha256=hashlib.sha256(payload).hexdigest(), resources=resources,
        scope='Same selected world geometry/plugins; smaller physics integration step'), indent=2)+'\n')
    return str(target)
