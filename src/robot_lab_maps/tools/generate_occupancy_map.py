#!/usr/bin/env python3
"""Generate a static occupancy slice using the pinned Fortress map plugin.

Meshes are sliced at their actual world pose/scale into a collision mask;
the upstream plugin's invented 1 m mesh bounds are never used. Its AABB cell
checks remain conservative. Heightfields need the R6.7 terrain converter.
"""
import os
import signal
import sys

# A fresh exec child binds its lifetime to the GUI-owned generator process.
if len(sys.argv)>2 and sys.argv[1]=='--child':
    import ctypes
    if ctypes.CDLL(None).prctl(1,signal.SIGTERM,0,0,0)!=0 or os.getppid()!=int(sys.argv[2]):
        raise SystemExit(1)
    os.execvp(sys.argv[3],sys.argv[3:])

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

# ros2 run adds a CLI parent between the GUI and this generator. If that
# parent is terminated by Stop Generation or timeout, terminate here too.
# Gazebo's own lifetime is bound to this process by the --child entry above.
import ctypes
generator_parent=os.getppid()
if ctypes.CDLL(None).prctl(1,signal.SIGTERM,0,0,0)!=0 or os.getppid()!=generator_parent:
    raise SystemExit('Cannot bind the map generator to its parent process')

import numpy as np
import trimesh
import yaml
from PIL import Image,ImageDraw
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from robot_lab_utils.sdf_world import extract_static_shapes,uri_resolver,_rotate
from robot_lab_utils.mesh_assets import _load_indexed_mesh


def slice_mesh(shape,height,resolution):
    vertices,faces=_load_indexed_mesh(shape['mesh'])
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=False)
    mesh.apply_scale(shape['scale'])
    transform=trimesh.transformations.quaternion_matrix(shape['orientation'])
    transform[:3,3]=shape['position']
    mesh.apply_transform(transform)
    section=mesh.section(plane_origin=[0,0,height],plane_normal=[0,0,1])
    if section is None:
        return [],mesh.bounds
    return list(section.discrete),mesh.bounds


def prepare_mapping_world(source,destination,output,resolution,height,seed):
    maps=Path(get_package_share_directory('robot_lab_maps'))
    dirs=[str(p) for p in maps.glob('maps/*/models')]+[str(source.parent),str(source.parent.parent/'models')]
    shapes,skipped=extract_static_shapes(str(source),uri_resolver(dirs,get_package_share_directory),collision_only=True)
    rejected=[s for s in skipped if not s.startswith('actor ')]
    if rejected:
        raise ValueError('World geometry cannot be projected: '+'; '.join(rejected))
    projected=[];bounds=[];mesh_files={};mesh_paths=[]
    for shape in shapes:
        kind=shape['type']
        if kind=='plane':
            continue
        if kind=='mesh':
            paths,box=slice_mesh(shape,height,resolution)
            mesh_paths.extend(paths);bounds.extend(box)
            path=Path(shape['mesh']);mesh_files[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        else:
            projected.append(shape)
            size=shape['size']
            extent=np.array(size)/2 if kind=='box' else np.full(3,size[0])
            if kind=='cylinder':extent[2]=size[1]/2
            for signs in itertools.product((-1,1),repeat=3):
                bounds.append(np.array(shape['position'])+_rotate(extent*np.array(signs),shape['orientation']))
    reach=np.max(np.abs(np.asarray(bounds)[:,:2]),axis=0)+2 if bounds else np.array([10.,10.])
    sizes=np.ceil(2*reach/resolution)*resolution
    if np.prod(sizes/resolution)>4_000_000:
        raise ValueError('More than 4 million grid cells; choose a coarser resolution')
    width,height_cells=(int(v/resolution) for v in sizes)
    mask=Image.new('L',(width,height_cells));draw=ImageDraw.Draw(mask)
    for path in mesh_paths:
        pixels=[((p[0]+sizes[0]/2)/resolution,(p[1]+sizes[1]/2)/resolution) for p in path]
        draw.line(pixels,fill=255,width=2)
    mask_path=destination.with_suffix('.collision-mask')
    mask_path.write_bytes(f'{width} {height_cells}\n'.encode()+mask.tobytes())
    root=ET.Element('sdf',version='1.7');world=ET.SubElement(root,'world',name='robot_lab_map_generation')
    ET.SubElement(world,'plugin',filename='ignition-gazebo-physics-system',name='ignition::gazebo::systems::Physics')
    ET.SubElement(world,'plugin',filename='ignition-gazebo-scene-broadcaster-system',name='ignition::gazebo::systems::SceneBroadcaster')
    model=ET.SubElement(world,'model',name='static_projection');ET.SubElement(model,'static').text='true'
    link=ET.SubElement(model,'link',name='geometry')
    for index,shape in enumerate(projected):
        collision=ET.SubElement(link,'collision',name=f'collision_{index}')
        rpy=trimesh.transformations.euler_from_quaternion(shape['orientation'])
        ET.SubElement(collision,'pose').text=' '.join(str(v) for v in (*shape['position'],*rpy))
        geometry=ET.SubElement(collision,'geometry');kind=shape['type'];node=ET.SubElement(geometry,kind)
        if kind=='box':ET.SubElement(node,'size').text=' '.join(str(v) for v in shape['size'])
        else:
            ET.SubElement(node,'radius').text=str(shape['size'][0])
            if kind=='cylinder':ET.SubElement(node,'length').text=str(shape['size'][1])
    library=Path(get_package_prefix('robot_lab_maps'))/'lib/librobot_lab_2dmap_system.so'
    if not library.is_file():
        raise RuntimeError('Build robot_lab_maps with the ignition-gazebo6 development SDK to install its map plugin')
    plugin=ET.SubElement(world,'plugin',filename=str(library),name='ignition::gazebo::systems::OccupancyMapFromWorld')
    for name,value in [('map_resolution',resolution),('map_height',height),('map_size_x',sizes[0]),
                       ('map_size_y',sizes[1]),('init_robot_x',seed[0]),('init_robot_y',seed[1]),('output_path',output),
                       ('collision_mask_path',mask_path)]:
        ET.SubElement(plugin,name).text=str(value)
    ET.ElementTree(root).write(destination,encoding='unicode',xml_declaration=True)
    return {'source_world':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'mesh_sha256':mesh_files,'mesh_slice_paths':len(mesh_paths),
            'mesh_mask_cells':int(np.sum(np.asarray(mask)>0)),'collision_count':len(projected),
            'resolution_m':resolution,'slice_height_m':height,'seed_xy':seed,'skipped_dynamic_actors':skipped,
            'engine':'robotics-upo Fortress 317a17d4dc8004e14299d767278f0ecbcb857819 + Robot Lab mesh collision mask',
            'limits':'Static height slice, conservative AABB checks; preview is not navigation qualification'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--resolution',type=float,default=.1)
    parser.add_argument('--height',type=float,default=.3)
    parser.add_argument('--seed-x',type=float,default=0)
    parser.add_argument('--seed-y',type=float,default=0)
    parser.add_argument('--timeout',type=float,default=180)
    args=parser.parse_args()
    if not all(math.isfinite(v) for v in [args.resolution,args.height,args.seed_x,args.seed_y,args.timeout]) or args.resolution<=0 or args.timeout<=0:
        parser.error('Resolution/timeout must be positive and all coordinates finite')
    source=args.world.resolve(strict=True)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    destination=args.output_dir.resolve()
    workspace=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))
    if destination.stat().st_dev!=workspace.stat().st_dev:
        parser.error('Generated maps must be on the workspace SSD')
    run=destination/(source.stem+'-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    run.mkdir();output=run/source.stem
    report=prepare_mapping_world(source,run/'mapping.sdf',output,args.resolution,args.height,[args.seed_x,args.seed_y])
    environment=dict(os.environ,IGN_PARTITION='mapgen_'+uuid.uuid4().hex)
    environment['GZ_PARTITION']=environment['IGN_PARTITION']
    process=None
    def terminate(_sig,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    try:
        with (run/'gazebo.log').open('w') as log:
            process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--child',str(os.getpid()),
                'ign','gazebo','-s','-r',str(run/'mapping.sdf')],env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+args.timeout;triggered=False
            while time.monotonic()<deadline and process.poll() is None:
                if not triggered:
                    response=subprocess.run(['ign','service','-s','/gazebo_2Dmap_plugin/generate_map',
                        '--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Empty','--timeout','1000','--req',''],
                        env=environment,capture_output=True,text=True,timeout=4)
                    triggered=response.returncode==0
                if output.with_suffix('.yaml').is_file() and output.with_suffix('.pgm').is_file():break
                time.sleep(.5)
            if not output.with_suffix('.pgm').is_file():raise RuntimeError('Generation did not create a map; inspect '+str(run/'gazebo.log'))
            metadata=yaml.safe_load(output.with_suffix('.yaml').read_text())
            image=np.asarray(Image.open(output.with_suffix('.pgm')))
            report.update(output_yaml=str(output.with_suffix('.yaml')),free_cells=int(np.sum(image==254)),
                          occupied_cells=int(np.sum(image==0)),unknown_cells=int(np.sum(image==205)),shape=list(image.shape),origin=metadata['origin'])
            if report['free_cells']<10:raise RuntimeError('Seed is blocked or the generated map has no free region')
            report['artifact_sha256']={suffix:hashlib.sha256(output.with_suffix(suffix).read_bytes()).hexdigest() for suffix in ('.yaml','.pgm')}
            output.with_suffix('.generation.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2),flush=True)
    except KeyboardInterrupt:
        return 130
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
