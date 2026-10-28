import json,os
from pathlib import Path
from robot_lab_gui.launcher import SimulationLauncherGui
app=SimulationLauncherGui();app.withdraw();app.update_idletasks();result={}
try:
 app.robot_var.set('lab_tracked');app.simulator_var.set('gazebo');app.map_var.set('nav_empty');app.mode_var.set('nav');app._update_from_selection()
 result['tracked_command']=app.command_var.get();result['mode']=app.mode_var.get()
 assert 'robot_model:=lab_tracked' in result['tracked_command'] and 'mode:=nav' in result['tracked_command']
 assert 'control:=native_tracked_controller' in result['tracked_command']
 assert 'local_planning:=pure_pursuit' in result['tracked_command']
 app.aerial_swarm_tab.measured_reference();result['aerial_reference']=app.aerial_swarm_tab.configuration()
 assert result['aerial_reference']['estimator']['height_reference']=='rangefinder'
 result['inventory']=dict(robots=len(app.robot_profiles),families=len(app.robot_groups.choices()))
finally:
 app._on_close();Path(os.environ['LAB_EXTENSION_RUN'],'native-gui-workloads.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
