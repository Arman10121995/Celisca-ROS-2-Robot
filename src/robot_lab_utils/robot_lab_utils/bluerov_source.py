"""Exact-frame URDF derivative of the pinned BlueROV2 Heavy SDF plant.

Only the inspected SDF 1.6 topology is accepted. Source mesh scales, collision
volume, inertias, propeller frames and water coefficients remain unchanged.
"""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def heavy_description(source_file, model_name):
    source_file=Path(source_file)
    sdf=ET.parse(source_file).getroot()
    model=sdf.find('model')
    if sdf.get('version')!='1.6' or model is None or model.get('name')!='bluerov2_heavy':
        raise ValueError('Expected the reviewed BlueROV2 Heavy SDF 1.6 source')
    root=ET.Element('robot',name=model_name)
    def pose(node):
        element=node.find('pose')
        if element is not None and element.attrib:raise ValueError('Unreviewed source pose frame')
        values=(element.text if element is not None else '0 0 0 0 0 0').split()
        if len(values)!=6 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError('Finite six-coordinate source pose required')
        return dict(xyz=' '.join(values[:3]),rpy=' '.join(values[3:]))
    if any(float(v) for v in ' '.join(pose(model).values()).split()):
        raise ValueError('Expected a zero source model pose')
    links={link.get('name'):link for link in model.findall('link')}
    if set(links)!={'base_link',*('thruster'+str(i) for i in range(1,9))}:
        raise ValueError('Expected complete hull and eight source propellers')
    meshes={}
    for name,link in links.items():
        target=ET.SubElement(root,'link',name=name)
        if name=='base_link' and any(float(v) for v in ' '.join(pose(link).values()).split()):
            raise ValueError('Expected the source hull at the model origin')
        original=link.find('inertial')
        inertial=ET.SubElement(target,'inertial')
        ET.SubElement(inertial,'origin',**pose(original))
        ET.SubElement(inertial,'mass',value=original.findtext('mass'))
        ET.SubElement(inertial,'inertia',**{n.tag:n.text.strip() for n in original.find('inertia')})
        for kind in ('collision','visual'):
            for item in link.findall(kind):
                output=ET.SubElement(target,kind,name=item.get('name'))
                ET.SubElement(output,'origin',**pose(item))
                geometry=ET.SubElement(output,'geometry')
                box=item.find('geometry/box')
                mesh=item.find('geometry/mesh')
                if box is not None:ET.SubElement(geometry,'box',size=box.findtext('size').strip())
                elif mesh is not None:
                    uri=mesh.findtext('uri')
                    if not uri.startswith('model://bluerov2_heavy/'):
                        raise ValueError('Unreviewed source mesh URI: '+uri)
                    path=(source_file.parent/uri.removeprefix('model://bluerov2_heavy/')).resolve()
                    if not path.is_relative_to(source_file.parent.resolve()) or not path.is_file():
                        raise ValueError('Missing source mesh: '+uri)
                    ET.SubElement(geometry,'mesh',filename=str(path),scale=mesh.findtext('scale','1 1 1').strip())
                    meshes[uri]=str(path)
                else:raise ValueError('Unsupported source geometry; do not replace it with a primitive')
        # Source IMU remains FRD for ArduSub; never relabel it as an ENU sensor.
        for sensor in link.findall('sensor'):
            block=ET.SubElement(root,'gazebo',reference=name)
            copied=copy.deepcopy(sensor)
            ET.SubElement(copied,'topic').text='/marine/source_frd_imu'
            block.append(copied)
    controls={n.findtext('jointName'):n for p in model.findall('plugin') for n in p.findall('control')}
    bounds={}
    for joint in model.findall('joint'):
        name=joint.get('name');child=joint.findtext('child')
        if joint.get('type')!='revolute' or joint.findtext('parent')!='base_link' or child not in links or joint.find('pose') is not None:
            raise ValueError('Unreviewed source propeller topology/frame')
        target=ET.SubElement(root,'joint',name=name,type='continuous')
        ET.SubElement(target,'parent',link='base_link');ET.SubElement(target,'child',link=child)
        ET.SubElement(target,'origin',**pose(links[child]))
        axis=joint.find('axis/xyz')
        if axis.attrib:raise ValueError('Unreviewed source joint axis frame')
        ET.SubElement(target,'axis',xyz=axis.text.strip())
        control=controls[name]
        if control.findtext('type')!='COMMAND':raise ValueError('Expected normalized native-force control')
        offset,multiplier=(float(control.findtext(k)) for k in ('offset','multiplier'))
        bounds[name]=sorted([offset*multiplier,(1.+offset)*multiplier])
    for plugin in model.findall('plugin'):
        kind=plugin.get('name','').split('::')[-1]
        if kind not in ('Thruster','Hydrodynamics'):continue
        copied=copy.deepcopy(plugin)
        copied.set('name','ignition::gazebo::systems::'+kind)
        copied.set('filename','ignition-gazebo-'+kind.lower()+'-system')
        if kind=='Thruster':
            copied.find('namespace').text=model_name
            for tag,value in zip(('min_thrust_cmd','max_thrust_cmd'),bounds[copied.findtext('joint_name')]):
                ET.SubElement(copied,tag).text=str(value)
        ET.SubElement(root,'gazebo').append(copied)
    return root,dict(meshes=meshes,force_bounds_N=bounds,
                     source_imu='Original FRD sensor retained on /marine/source_frd_imu; separate lab ENU IMU added',
                     actuation='ArduSub control replaced by existing lab force allocator; no firmware parity claim')
