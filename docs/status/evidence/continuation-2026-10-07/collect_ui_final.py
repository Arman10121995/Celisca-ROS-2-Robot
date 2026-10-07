from pathlib import Path
import hashlib,json,shutil,yaml,datetime,subprocess
repo=Path('/workspace/molar/ros_ws/bumperbot_ws')
root=Path('/workspace/molar/robot_lab_runtime/extensions-2026-10-07')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def entry(p): return dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p))
def copy_small(raw,dest,names):
 dest.mkdir(parents=True,exist_ok=True)
 for name in names:
  p=raw/name
  if p.exists(): shutil.copy2(p,dest/name)
 return [entry(p) for p in sorted(raw.rglob('*')) if p.is_file() and '__pycache__' not in p.parts]
def dump(p,data): p.write_text(json.dumps(data,indent=2)+'\n')
preview=root/'registry-preview'
raw=preview/'native-stage5'
dest=repo/'docs/status/evidence/registry-3d-2026-10-07'
r=json.loads((raw/'report.json').read_text());assert len(r['cases'])==7 and r['closed'] is True
copy_small(raw,dest,['report.json'])
for case in r['cases']:
 assert case['colors']>25 and case['orbit_changed_pixel_fraction']>.01
 assert case['launch_selection_unchanged'] and case['loader_returncode']==0
 copy_small(raw/case['asset_id'],dest/case['asset_id'],['viewport.png','gui.png','request.json','scene.json','loader.log'])
copy_small(preview,dest,['probe_embedded.py','assembly-topology.json'])
for stage in ('native-stage1','native-stage2','native-stage3','native-stage4'):
 folder=dest/'negative'/stage
 copy_small(preview/stage,folder,['report.json'])
 if (preview/(stage+'.log')).exists(): shutil.copy2(preview/(stage+'.log'),folder/'producer.log')
src_paths=['src/robot_lab_gui/robot_lab_gui/launcher.py','src/robot_lab_gui/robot_lab_gui/lab_tabs.py',
 'src/robot_lab_gui/robot_lab_gui/native_viewer.py','src/robot_lab_gui/robot_lab_gui/registry_preview.py',
 'src/robot_lab_utils/robot_lab_utils/asset_groups.py','src/robot_lab_utils/robot_lab_utils/asset_preview.py',
 'src/robot_lab_utils/robot_lab_utils/sdf_world.py','src/robot_lab_bringup/config/asset_groups.yaml']
sources=[dict(entry(repo/p),captured_at='archive time after rendering; not a pre-trial snapshot') for p in src_paths]
dump(dest/'source-hashes.json',dict(date='2026-10-07',scope='Archive-time implementation fingerprints; actual asset source hashes are in each scene/report',files=sources))
dump(dest/'manifest.json',dict(raw_directory=str(raw),producer_returncode=0,
 scope='Actual embedded rendering and camera input only; no mission qualification',artifacts=copy_small(raw,dest,[]),
 producer=entry(preview/'probe_embedded.py'),negative_directories=[str(preview/s) for s in ('native-stage1','native-stage2','native-stage3','native-stage4')],
 buffer_directories=[c['preview_directory'] for c in r['cases']],source_hashes=str((dest/'source-hashes.json').relative_to(repo))))
# Repeats use normal installed profiles and the consolidated GUI.
t3=root/'turtlebot3';mdest=repo/'docs/status/evidence/turtlebot3-modes-2026-10-07'
copy_small(t3/'normal-gui-modes',mdest/'normal-gui',['producer.py','report.json'])
navraw=t3/'normal-current/workflows/asset_turtlebot3_waffle_pi-mujoco-nav-nav_obstacle'
navdest=mdest/'normal-current-waffle-pi-mujoco-obstacle'
nav=json.loads((navraw/'report.json').read_text());assert nav['passed'] and nav['launch_returncode']==nav['check_returncode']==0
navfiles=copy_small(navraw,navdest,['producer.py','report.json','navigation.json','source-manifest.json','launch-ownership.json','check.log'])
drraw=t3/'normal-current-with-preview/asset_turtlebot3_burger-pybullet'
drdest=dest/'live-burger-pybullet'
dr=json.loads((drraw/'report.json').read_text());assert dr['passed'] and dr['launch_returncode']==0
assert dr['embedded_preview']['movement_commands_during_preview']==0 and abs(dr['embedded_preview']['neutral']['dx'])<.03
assert dr['embedded_preview']['colors']>3
drfiles=copy_small(drraw,drdest,['producer.py','report.json','source-manifest.json','gui-output.log','executed.urdf'])
buffers=Path(dr['embedded_preview']['directory'])
copy_small(buffers,drdest,['request.json','scene.json','loader.log'])
dump(dest/'live-plant-manifest.json',dict(raw_directory=str(drraw),producer_returncode=0,artifacts=drfiles,
 scope='Actual normal GUI embedded preview while PyBullet runs; independent neutral body truth, no preview movement commands, then original Drive/Stop/watchdog and LDS/joint/TF acceptance'))
manifest=json.loads((mdest/'manifest.json').read_text())
manifest['normal_current_repeats']=[dict(raw_directory=str(navraw),producer_returncode=0,artifacts=navfiles)]
manifest['normal_gui_selections']=dict(report=str((mdest/'normal-gui/report.json').relative_to(repo)),sha256=sha(mdest/'normal-gui/report.json'),producer=str((mdest/'normal-gui/producer.py').relative_to(repo)),scope='48 actual normal Tk mode/algorithm/default/autofill selections; no new mission implied')
dump(mdest/'manifest.json',manifest)
# Fresh physical Panda proof refreshes only contracts measured by that trial.
pr=root/'panda-planning/physical-gui-2-grouped';pd=repo/'docs/status/evidence/panda-cartesian-grouped-gui-2026-10-07'
p=json.loads((pr/'report.json').read_text());assert p['passed'] and p['launch_returncode']==0
pf=copy_small(pr,pd,['producer.py','report.json','source-manifest.json','sdk-manifest.json','ownership.json'])
source=json.loads((pr/'source-manifest.json').read_text())
supportfile=repo/'docs/status/asset-runtime-support.yaml';support=yaml.safe_load(supportfile.read_text())
certificate=support['arm_planning']['menagerie_franka_emika_panda']
for row in certificate['configuration_files']:
 if row['path'] not in source['source_sha256']:
  assert row['path']=='src/robot_lab_maps/maps/nav_empty/worlds/nav_empty.world'
  committed=subprocess.check_output(['git','show',source['git_head']+':'+row['path']],cwd=repo)
  installed=repo/'install/robot_lab_maps/share/robot_lab_maps/maps/nav_empty/worlds/nav_empty.world'
  assert hashlib.sha256(committed).hexdigest()==row['sha256']==sha(repo/row['path'])==sha(installed)
  continue
 assert sha(repo/row['path'])==source['source_sha256'][row['path']],row['path']+' changed after the trial'
 row['sha256']=source['source_sha256'][row['path']]
certificate['report']=dict(path=str((pd/'report.json').relative_to(repo)),sha256=sha(pd/'report.json'))
supportfile.write_text(yaml.safe_dump(support,sort_keys=False))
dump(pd/'manifest.json',dict(date='2026-10-07',run_directory=str(pr),source_stage='18ecdd2 plus pre-trial consolidated GUI/shared SDF visual reader; exact hashes in source-manifest.json',producer_returncode=0,experimental_override='none; normal installed profile',artifacts=pf,source_snapshot_directory=str(pr/'source-snapshot'),previous_trial_manifest='docs/status/evidence/panda-cartesian-2026-10-07/manifest.json',world_hash_provenance='Recorded HEAD world bytes checked against source and installed world at collection; omitted from original pre-trial source sweep',scope='Physical MuJoCo/native Panda/MoveIt GUI regression only; native viewer tests are separate'))
idxfile=repo/'docs/status/runtime-evidence-index.yaml';idx=yaml.safe_load(idxfile.read_text())
for original,folder,filename,scope in [
 (next(row for row in idx['records'] if row['robot_id']=='menagerie_franka_emika_panda' and row['date']=='2026-10-07'),pd,'report.json','Fresh actual normal consolidated-GUI Panda physical Plan/Execute after grouping and the shared SDF visual-reader change; two independent physical TCP offsets and all original rejection/invalidation/interruption/reset/monitor/cleanup checks.'),
 (next(row for row in idx['records'] if row['robot_id']=='asset_turtlebot3_waffle_pi' and row['simulator']=='mujoco' and row['environment_id']=='nav_obstacle'),navdest,'navigation.json','Actual normal-installed consolidated GUI Waffle Pi MuJoCo obstacle navigation repeat with current 0.65 m inflation; independent body terminal and swept-clearance limits unchanged.')]:
 row=dict(original);row.update(report=str((folder/filename).relative_to(repo)),sha256=sha(folder/filename),source_hashes=str((folder/'source-manifest.json').relative_to(repo)),source_hashes_sha256=sha(folder/'source-manifest.json'),scope=scope)
 idx['records']=[x for x in idx['records'] if x['report']!=row['report']];idx['records'].append(row)
idxfile.write_text(yaml.safe_dump(idx,sort_keys=False))
print('Archived seven embedded scenes, live-plant preview/Drive, normal mode selections/navigation and fresh Panda regression.')
