from pathlib import Path
import json,shlex
from robot_lab_gui.launcher import SimulationLauncherGui
app=SimulationLauncherGui();app.withdraw();app.update_idletasks();result={}
try:
 app.robot_var.set('lab_amphibious');app.map_var.set('amphibious_shore');app.simulator_var.set('gazebo');app.mode_var.set('marine');app._update_from_selection();app.marine_slam_var.set(True);app._update_validation_and_command()
 command=app.command_var.get();assert 'robot_model:=lab_amphibious' in shlex.split(command) and 'marine_slam:=true' in shlex.split(command),command
 result['amphibious_command']=command
 assert app._map_ok_for_mode('nav_empty');result['terrestrial_map_selectable']=True
 assert hasattr(app.fleet_tab,'infrastructure_controls');result['fleet_infrastructure_page']=True
 result['inventory']=dict(robot_profiles=len(app.robot_profiles),robot_families=len(app.robot_groups.choices()),world_profiles=len(app.map_profiles),world_families=len(app.map_groups.choices()))
finally:app._on_close()
p=Path('/workspace/molar/robot_lab_runtime/amphibious-infrastructure-20261030-mcpiqquv/gui-current.json');p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
