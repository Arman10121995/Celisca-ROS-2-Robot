import json,shlex
from pathlib import Path
from robot_lab_gui.launcher import SimulationLauncherGui
root=Path('/workspace/molar/robot_lab_runtime/extension-completion-20261029-vsebubus')
app=SimulationLauncherGui();app.withdraw();app.update_idletasks();result={}
try:
 for robot,backend,mode,world in [('lab_snake','mujoco','gait','nav_empty'),('lab_usv','gazebo','marine','surface_harbor'),('lab_tracked_articulated','gazebo','nav','nav_empty')]:
  app.robot_var.set(robot);app.simulator_var.set(backend);app.map_var.set(world);app.mode_var.set(mode);app._update_from_selection()
  if robot=='lab_snake':app.gait_task_var.set('slam')
  elif robot=='lab_usv':app.marine_slam_var.set(True)
  app._update_validation_and_command();command=app.command_var.get();arguments=shlex.split(command)
  result[robot]=dict(command=command,validation=app.validation_var.get(),family=app.robot_groups.family(robot))
  assert 'robot_model:='+robot in arguments,result[robot]
  assert 'mode:='+mode in arguments,command
  assert ('gait_task:=slam' if robot=='lab_snake' else 'marine_slam:=true' if robot=='lab_usv' else 'control:=native_tracked_controller') in arguments,command
  result[robot]=dict(command=command,family=app.robot_groups.family(robot))
 from robot_lab_gui.lab_tabs import format_platform_status
 import yaml
 health=format_platform_status(yaml.safe_load(Path('/workspace/molar/ros_ws/bumperbot_ws/docs/status/platform-status.yaml').read_text()))
 assert '151 robot profiles / 81 families' in health and 'amphibious_model_controller' in health
 assert 'AUV acquisition is measured, not missing' in health
 result['health_current_delivery']=True
 result['inventory']=dict(robot_profiles=len(app.robot_profiles),robot_families=len(app.robot_groups.choices()),
  world_profiles=len(app.map_profiles),world_families=len(app.map_groups.choices()))
finally:
 app._on_close();(root/'native-gui-forms.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
