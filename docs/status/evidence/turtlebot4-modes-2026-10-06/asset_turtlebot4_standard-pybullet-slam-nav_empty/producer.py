"""Actual Tk selection/autofill/Run/Save/Stop and independently measured ROS workflow."""
import hashlib,json,os,subprocess,time
from pathlib import Path
from robot_lab_gui.launcher import SimulationLauncherGui
import robot_lab_gui.launcher as gui_module

robot=os.environ['PROBE_ROBOT'];backend=os.environ['PROBE_BACKEND'];mode=os.environ['PROBE_MODE']
world=os.environ.get('PROBE_MAP','nav_empty');root=Path.cwd()
out=Path(os.environ.get('PROBE_ROOT',str(Path(__file__).parent)))/'workflows'/(robot+'-'+backend+'-'+mode+'-'+world);out.mkdir(parents=True,exist_ok=True)
(out/'producer.py').write_bytes(Path(__file__).read_bytes())
app=SimulationLauncherGui();app.update();check=None
report=dict(robot=robot,backend=backend,mode=mode,map=world,passed=False,
 scope='Actual GUI Launch/autofill/Run/Stop; workflow motion sent through its /key_vel channel and independent engine truth. Sensor/Drive keyboard widgets separately recorded.')
last_log=0.
def pump():
 global last_log
 app.update();time.sleep(.02)
 if time.monotonic()-last_log>10:
  (out/'live-gui-output.log').write_text(app.output.get('1.0','end'))
  last_log=time.monotonic()
try:
 if os.environ.get('PROBE_WORLD_OVERRIDE'):
  setter=app._set_command
  def override_world(command):
   command=[arg for arg in command if not arg.startswith('world_path:=')]
   command.append('world_path:='+os.environ['PROBE_WORLD_OVERRIDE']);setter(command)
  app._set_command=override_world
 app.robot_var.set(robot);app.simulator_var.set(backend);app.map_var.set(world);app.mode_var.set(mode)
 app.gui_var.set('false');app._update_from_selection();app.update()
 assert app.mode_var.get()==mode,app.validation_var.get()
 assert app.start_button.instate(['!disabled']),app.validation_var.get()
 selected={k:v.get() for k,v in app.slot_vars.items()}
 expected={'loc':('localizer','amcl'),'slam':('slam_backend','slam_toolbox'),
           '3d_slam':('slam_3d_backend','rtabmap_localization')}
 if mode in expected:
  slot,value=expected[mode]
  assert value in selected.values(),(mode,selected)
 if mode=='nav':assert 'amcl' in selected.values() and 'pure_pursuit' in selected.values(),selected
 report['command']=app.command_var.get()
 if os.environ.get('PROBE_WORLD_OVERRIDE'):assert 'world_path:='+os.environ['PROBE_WORLD_OVERRIDE'] in report['command']
 report['algorithm_selection']={k:v.get() for k,v in app.slot_vars.items()}
 profile=app.robot_profiles[robot];model=Path(profile['xacro']);(out/'executed.urdf').write_bytes(model.read_bytes())
 files={}
 for package in ('robot_lab_utils','robot_lab_gui','robot_lab_bringup','robot_lab_description',
                 'robot_lab_mujoco','robot_lab_pybullet','robot_lab_isaac','robot_lab_navigation',
                 'robot_lab_localization','robot_lab_mapping','robot_lab_controller'):
  for path in (root/'src'/package).rglob('*'):
   if path.is_file() and path.suffix in ('.py','.yaml','.xml','.xacro','.cpp'):
    files[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
 (out/'source-manifest.json').write_text(json.dumps(dict(git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
  installed_sha256={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for package in ('robot_lab_utils','robot_lab_gui','robot_lab_bringup','robot_lab_isaac') for f in (root/'install'/package).rglob('*.py') if f.is_file()}, source_sha256=files,profile=profile,map_profile=app.map_profiles[world],executed_urdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
  producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
 app._start_launch();report['owned_pid']=app.process.pid
 (out/'launch-ownership.json').write_text(json.dumps(dict(pid=app.process.pid,command=report['command'],robot=robot,backend=backend))+'\n')
 if mode=='nav':
  command=['python3','scripts/sim_nav_check.py','--base','base_link','--timeout',str(600 if backend=='isaac' else 360),'--via-topic',
   '--max-position-error','0.15','--max-yaw-error-deg','5','--trace-out',str(out/'navigation.trace.json'),
   '--robot-radius','0.185','--min-route-clearance','0.02',
   '--world-file',str(root/'src/robot_lab_maps/maps'/world/'worlds'/(world+'.world'))]
  if world=='nav_obstacle':command+=['--offset-x','3.5','--offset-y','3.5','--goal-yaw-deg','0','--require-detour']
  else:command+=['--offset-x','2','--goal-yaw-deg','0']
 else:
  command=['python3','scripts/sim_workflow_check.py','--mode',mode,'--base-frame','base_link','--timeout','300',
   '--out',str(out/'workflow.json')]
 report['check_command']=command
 with (out/'check.log').open('w') as log:
  check=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
  deadline=time.monotonic()+float(os.environ.get('PROBE_BUDGET','900'))
  while check.poll() is None and time.monotonic()<deadline:
   pump();assert app.process.poll() is None,'owned launch exited during workflow'
  assert check.poll() is not None,'workflow budget expired'
  report['check_returncode']=check.returncode
 if mode=='nav':
  lines=(out/'check.log').read_text().splitlines();record=json.loads(next(l for l in reversed(lines) if l.startswith('{')))
  (out/'navigation.json').write_text(json.dumps(record,indent=2)+'\n');report['workflow']=record
 else:report['workflow']=json.loads((out/'workflow.json').read_text())
 assert check.returncode==0,report['workflow']
 if mode in ('slam','3d_slam'):
  target=out/('saved-map.db' if mode=='3d_slam' else 'saved-map')
  gui_module.filedialog.asksaveasfilename=lambda **kwargs:str(target)
  assert app.save_map_button.instate(['!disabled'])
  app._save_map();deadline=time.monotonic()+90
  outputs=[target,target.with_suffix('.pcd')] if mode=='3d_slam' else [target.with_suffix('.yaml'),target.with_suffix('.pgm')]
  while not all(p.is_file() and p.stat().st_size>0 for p in outputs) and time.monotonic()<deadline:pump()
  assert all(p.is_file() and p.stat().st_size>0 for p in outputs),'GUI map save failed'
  if mode=='3d_slam':
   import sqlite3
   with sqlite3.connect(target) as db:
    assert db.execute('pragma integrity_check').fetchone()[0]=='ok'
    nodes=db.execute('select count(*) from Node').fetchone()[0];assert nodes>1
   import io,numpy as np
   text=outputs[1].read_text();assert 'DATA ascii\n' in text
   points=np.loadtxt(io.StringIO(text.split('DATA ascii\n',1)[1]));assert len(points)>100
   assert np.isfinite(points[:,:3]).all()
   report['save_map']=dict(database_nodes=nodes,pointcloud_points=len(points))
  else:report['save_map']=dict(yaml=outputs[0].read_text(),image_bytes=outputs[1].stat().st_size)
  deadline=time.monotonic()+10
  while app._aux_processes and time.monotonic()<deadline:pump()
  assert not app._aux_processes, 'map exporter did not exit after saving'
  report['map_exporter_exited']=True
 app._stop_drive();app._stop_launch();deadline=time.monotonic()+30
 while app._launch_running and time.monotonic()<deadline:pump()
 report['launch_returncode']=app.process.poll();assert report['launch_returncode']==0
 report['passed']=True
except BaseException as exc:
 report['exception']=repr(exc)
 raise
finally:
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 if check and check.poll() is None:
  check.terminate()
  try:check.wait(timeout=10)
  except subprocess.TimeoutExpired:check.kill();check.wait()
 if app.process and app.process.poll() is None:
  app._stop_drive();app._stop_launch();deadline=time.monotonic()+30
  while app._launch_running and time.monotonic()<deadline:pump()
 report['launch_returncode']=app.process.poll() if app.process else None
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 (out/'gui-output.log').write_text(app.output.get('1.0','end'));app.destroy()
