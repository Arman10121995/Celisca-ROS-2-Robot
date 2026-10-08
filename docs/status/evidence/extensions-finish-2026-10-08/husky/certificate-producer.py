import hashlib,json,yaml,subprocess
from pathlib import Path
root=Path.cwd();raw=Path('/workspace/molar/robot_lab_runtime/extensions-finish-2026-10-07/husky/modes-current');dest=root/'docs/status/evidence/extensions-finish-2026-10-08/husky/modes-current'
rows=json.loads((raw/'batch-report.json').read_text());assert len(rows)==20 and all(r['passed'] and r['producer_returncode']==r['launch_returncode']==0 for r in rows)
profile=yaml.safe_load(Path('/workspace/molar/robot_lab_runtime/external_assets/installed/robots.yaml').read_text())['robots']['asset_husky']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
common=['scripts/extension_husky_control.py','src/robot_lab_utils/robot_lab_utils/skid_steer.py','src/robot_lab_utils/robot_lab_utils/drive_kinematics.py','src/robot_lab_utils/robot_lab_utils/robot_spawn.py','src/robot_lab_utils/robot_lab_utils/sensor_config.py','src/robot_lab_utils/robot_lab_utils/sim_frames.py','src/robot_lab_gui/robot_lab_gui/drive_control.py','src/robot_lab_gui/robot_lab_gui/launcher.py','src/robot_lab_bringup/launch/simulated_robot.launch.py','src/robot_lab_navigation/config/robots/asset_husky.yaml','src/robot_lab_navigation/launch/navigation.launch.py','src/robot_lab_navigation/config/controller_server.yaml','src/robot_lab_localization/config/robots/asset_husky.yaml','src/robot_lab_localization/config/amcl.yaml','src/robot_lab_localization/config/ekf.yaml','src/robot_lab_localization/launch/local_localization.launch.py','src/robot_lab_localization/launch/global_localization.launch.py']
map_files=[]
for world in ('nav_empty','nav_obstacle'):
 map_files+=['src/robot_lab_maps/maps/'+world+'/worlds/'+world+'.world','src/robot_lab_maps/maps/'+world+'/maps/map.yaml','src/robot_lab_maps/maps/'+world+'/maps/map.pgm']
by_backend={'gazebo':['src/robot_lab_description/launch/gazebo.launch.py','src/robot_lab_utils/robot_lab_utils/gazebo_physics_world.py'],'mujoco':['src/robot_lab_mujoco/python/robot_lab_mujoco/mujoco_spawner.py','src/robot_lab_mujoco/launch/mujoco_simulator.launch.py'],'pybullet':['src/robot_lab_pybullet/python/robot_lab_pybullet/pybullet_spawner.py','src/robot_lab_pybullet/launch/pybullet_simulator.launch.py'],'isaac':['src/robot_lab_isaac/python/robot_lab_isaac/isaac_spawner.py','src/robot_lab_isaac/python/robot_lab_isaac/isaac_runtime.py','src/robot_lab_isaac/launch/isaac_simulator.launch.py']}
published_revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
map_receipt=[]
for n in map_files:
 published=subprocess.check_output(['git','show',published_revision+':'+n])
 assert hashlib.sha256(published).hexdigest()==sha(root/n),('Published map changed',n)
 map_receipt.append(dict(path=n,sha256=sha(root/n),published_revision=published_revision,verification='Current bytes match the unchanged published blob. Original pre-trial manifests captured paths/profiles, not map-byte hashes; physical trial traces and source contracts remain unchanged.'))
(dest/'map-source-verification.json').write_text(json.dumps(map_receipt,indent=2)+'\n')
certificate=dict(source_revision='729f8aa45ccd86fa33a05e07ef698c52c451cd9c',drive=profile['drive'],sensor_config=profile['sensor_config'],executed_urdf_sha256=sha(Path(profile['xacro'])),generated_model_files=[dict(path='drive-controllers.yaml',sha256=sha(Path(profile['xacro']).parent/'drive-controllers.yaml'))],configuration_files=[dict(path=n,sha256=sha(root/n)) for n in common+map_files],configuration_files_by_simulator={backend:[dict(path=n,sha256=sha(root/n)) for n in names] for backend,names in by_backend.items()},screens=[])
for backend in by_backend:
 for mode in ('loc','slam','3d_slam','nav'):
  proofs=[]
  for row in rows:
   if row['backend']!=backend or row['mode']!=mode:continue
   folder=dest/Path(row['report']).parent.name;report=json.loads((folder/'report.json').read_text());source=json.loads((folder/'source-manifest.json').read_text())
   for n in common+by_backend[backend]:assert source['source_sha256'].get(n)==sha(root/n),(backend,mode,n)
   assert source['revision']==published_revision
   assert source['map_profile']['gazebo']['world_path'].endswith('/'+row['map']+'.world')
   assert source['executed_urdf_sha256']==certificate['executed_urdf_sha256']
   assert source['profile']['drive']==profile['drive'] and source['profile']['sensor_config']==profile['sensor_config']
   if mode=='nav':
    trace=json.loads((Path(row['report']).parent/'navigation.trace.json').read_text())['samples'];assert len(trace)>10
    expected=float(source['map_profile'].get('spawn',{}).get('z',0));report['workflow']['body_height']=dict(expected_floor_m=expected,minimum_m=min(t['z'] for t in trace),maximum_m=max(t['z'] for t in trace),samples=len(trace))
    report['derived_from']=dict(original_report_sha256=sha(folder/'report.json'),physical_trace_sha256=sha(Path(row['report']).parent/'navigation.trace.json'),physical_trace=str(Path(row['report']).parent/'navigation.trace.json'))
   path=folder/'report-qualified.json';path.write_text(json.dumps(report,indent=2)+'\n');proofs.append(dict(path=str(path.relative_to(root)),sha256=sha(path)))
  certificate['screens'].append(dict(backend=backend,mode=mode,reports=proofs))
from robot_lab_utils.asset_support import apply_recorded_modes
accepted=apply_recorded_modes(profile,certificate['source_revision'],certificate,root);assert all(set(modes)=={'display','loc','slam','3d_slam','nav'} for modes in accepted['supported_modes_by_simulator'].values())
path=root/'docs/status/asset-runtime-support.yaml';data=yaml.safe_load(path.read_text());data['robots']['asset_husky']=certificate;data['updated']='2026-10-08';path.write_text(yaml.safe_dump(data,sort_keys=False));print('Published source-checked Husky certificate from twenty actual physical mode trials.')
