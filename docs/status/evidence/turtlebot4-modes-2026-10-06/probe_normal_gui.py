"""Normal installed Tk selection/autofill; actual missions are separate proof."""
import json,shlex,hashlib
from pathlib import Path
from robot_lab_gui.launcher import SimulationLauncherGui
r=Path(__file__).parent
a=SimulationLauncherGui();a.update();rows=[]
try:
 for robot in ('asset_turtlebot4_standard','asset_turtlebot4_lite'):
  for backend in ('gazebo','mujoco','pybullet','isaac'):
   for mode in ('display','loc','slam','3d_slam','nav'):
    a.robot_var.set(robot);a.simulator_var.set(backend);a.map_var.set('nav_empty');a.mode_var.set(mode);a._update_from_selection();a.update()
    cmd=shlex.split(a.command_var.get());values={k:v.get() for k,v in a.slot_vars.items()}
    assert a.robot_var.get()==robot and a.simulator_var.get()==backend and a.mode_var.get()==mode
    assert a.start_button.instate(['!disabled']),a.validation_var.get()
    for arg in ('robot_model:='+robot,'simulator:='+backend,'mode:='+mode,'map_name:=nav_empty'):assert arg in cmd,(arg,cmd)
    if mode in ('loc','nav'):assert 'amcl' in values.values(),values
    if mode=='slam':assert 'slam_toolbox' in values.values(),values
    if mode=='3d_slam':assert 'rtabmap_localization' in values.values(),values
    if mode=='nav':assert 'pure_pursuit' in values.values(),values
    rows.append(dict(robot=robot,backend=backend,mode=mode,command=a.command_var.get(),algorithms=values,run_enabled=True))
finally:
 (r/'normal-gui-qualification.json').write_text(json.dumps(dict(scope='Actual normal-installed Tk selectors, compatible algorithm defaults, command and Run state; physics missions recorded separately',rows=rows,
    producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
 a._on_close()
print('Normal installed GUI selections checked:',len(rows))
