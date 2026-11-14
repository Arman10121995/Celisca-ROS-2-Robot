#!/usr/bin/env python3
"""Install the declared pick/transport/drop bench fixture in the normal GUI maps."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.mobile_manipulation_scene import NAME,BENCHES,OBJECT,sdf,mjcf,occupancy
from provision_extension_assets import write_yaml


def main():
    store=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'external_assets'
    store.mkdir(parents=True,exist_ok=True)
    if store.stat().st_dev==Path('/').stat().st_dev:raise ValueError('Use the mounted SSD')
    import cv2
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker=(cv2.aruco.generateImageMarker(dictionary,7,200) if hasattr(cv2.aruco,'generateImageMarker') else
            cv2.aruco.drawMarker(dictionary,7,200))
    marker=cv2.copyMakeBorder(marker,20,20,20,20,cv2.BORDER_CONSTANT,value=255)
    ok,png=cv2.imencode('.png',marker)
    if not ok:raise RuntimeError('Marker image encoding failed')
    marker_bytes=png.tobytes()
    marker_binding=dict(schema='robot_lab.object_markers.v1',dictionary='DICT_4X4_50',markers=[dict(
        id=7,object='environment_pickup_cube',marker_size_m=.024,object_size_m=[.03,.03,.03],
        object_from_marker=[0.,0.,.015075,1.,0.,0.,0.])])
    document=sdf();native=mjcf('../assets/pickup_marker.png');grid=occupancy()
    revision=hashlib.sha256(document.encode()+native.encode()+grid+marker_bytes+json.dumps(marker_binding,sort_keys=True).encode()).hexdigest()
    folder=store/'worlds'/NAME/revision
    for directory in ('worlds','mjcf','maps','assets'):(folder/directory).mkdir(parents=True,exist_ok=True)
    (folder/'assets/pickup_marker.png').write_bytes(marker_bytes)
    write_yaml(folder/'object_markers.yaml',marker_binding)
    (folder/'worlds'/f'{NAME}.world').write_text(document)
    (folder/'mjcf'/f'{NAME}.xml').write_text(native)
    (folder/'maps/map.pgm').write_bytes(grid)
    write_yaml(folder/'maps/map.yaml',dict(image='map.pgm',resolution=.05,origin=[-2.,-2.,0.],negate=0,occupied_thresh=.65,free_thresh=.196))
    profile=dict(gazebo=dict(world_name=NAME,world_package='robot_lab_maps',world_path=str(folder/'worlds'/f'{NAME}.world')),
        map=dict(has_2d_map=True,package='robot_lab_maps',path=str(folder/'maps/map.yaml')),
        source_id='robot_lab_task_fixture',source_entry=NAME,object_markers=str(folder/'object_markers.yaml'),
        spawn=dict(x='0.',y='0.',z='0.',yaw='0.'),initial_pose=dict(x='0.',y='0.',yaw='0.'),
        notes='Declared lab pickup/delivery benches, 35 g free 30 mm cube, original source robot unchanged. '
              'MuJoCo native tasks preserve the free object; Gazebo has a dynamic SDF cube. '
              'Alternate world importers may flatten the cube to static geometry; no manipulation acceptance there. '
              'Generated occupancy includes benches, excludes the movable cube; local sensing is required. '
              'Task fixture parameters are lab assumptions, not hardware specifications.')
    entity=dict(id=NAME,version='1.0.0',name='Mobile manipulation lab',status='available',dimension='3D',
        simulator='gazebo',simulators=['gazebo','mujoco','pybullet','isaac'],
        world_file=profile['gazebo']['world_path'],occupancy_yaml=profile['map']['path'],
        occupancy_map=str(folder/'maps/map.pgm'),supported_robot_classes=['mobile','manipulator','hybrid'],
        source=dict(repository='Robot Lab declared task fixture',license='MIT'),
        dynamics=dict(static_obstacles=True,dynamic_obstacles=True),tags=['lab','manipulation','free_object'],
        description=profile['notes'])
    (store/'locks').mkdir(exist_ok=True)
    with (store/'locks/provision.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        installed=store/'installed';installed.mkdir(exist_ok=True)
        path=installed/'maps.yaml';data=yaml.safe_load(path.read_text()) if path.exists() else {}
        data=data or {};data.setdefault('maps',{})[NAME]=profile;write_yaml(path,data)
        path=installed/'registry_maps.yaml';data=yaml.safe_load(path.read_text()) if path.exists() else []
        entries={e['id']:e for e in data or []};entries[NAME]=entity;write_yaml(path,list(entries.values()))
    receipt=dict(id=NAME,revision=revision,world=str(folder),benches=BENCHES,free_object=OBJECT,
                 qualification='Installed implementation fixture; no robot mission receipt')
    (folder/'LICENSE').write_bytes((ROOT/'LICENSE').read_bytes())
    (folder/'integration.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':main()
