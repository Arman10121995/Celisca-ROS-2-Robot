import json,math,os,time
from pathlib import Path
from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.experiment_queue import ExperimentQueue
from types import SimpleNamespace
app=SimulationLauncherGui();app.withdraw();app.update_idletasks()
results={}
try:
 ids=['asset_turtlebot3_burger','asset_turtlebot3_waffle','asset_turtlebot3_waffle_pi','asset_turtlebot4_standard','asset_turtlebot4_lite','asset_husky','menagerie_franka_emika_panda']
 results['restored_ids_present']=all(i in app.robot_profiles for i in ids)
 results['inventory']=dict(robot_profiles=len(app.robot_profiles),robot_families=len(app.robot_groups.choices()),maps=len(app.map_profiles),map_families=len(app.map_groups.choices()))
 app.experimental_assets_var.set(True)
 commands={}
 for name in ids[:-1]:
  app.robot_var.set(name);app._update_from_selection();app.simulator_var.set('mujoco');app.mode_var.set('nav');app.map_var.set('nav_empty');app._update_from_selection()
  commands[name]=app.command_var.get()
  assert 'allow_experimental_assets:=true' in commands[name] and 'mode:=nav' in commands[name]
  assert 'global_planning:=a_star_planner' in commands[name] and 'local_planning:=pure_pursuit' in commands[name]
 results['candidate_commands']=commands
 m=app.marine_controls.missions
 m.runtime=dict(robot_id='tethys_auv',ready=True,position=[-3.,0.,-2.],estimated_yaw=-math.pi)
 m.mission_runtime_at=time.monotonic();m.ahead_distance.set(3.);m.ahead()
 results['auv_ahead_xyz']={k:v.get() for k,v in m.goal.items()}
 assert abs(m.goal['x'].get())<1e-8 and abs(m.goal['y'].get())<1e-8
 m.runtime=dict(robot_id='lab_rov',ready=True,position=[1.,2.,-1.],estimated_yaw=math.pi/2);m.mission_runtime_at=time.monotonic();m.ahead_distance.set(.5);m.ahead()
 results['rov_ahead_xyz']={k:v.get() for k,v in m.goal.items()}
 assert abs(m.goal['x'].get()-1.)<1e-8 and abs(m.goal['y'].get()-2.5)<1e-8
 fake=SimpleNamespace(goals=SimpleNamespace(get=lambda:'alpha 0.3 -0.5 0; beta 0.3 -1.5 0'))
 results['fleet_workload']=ExperimentQueue.fleet_workload(fake)
 assert len(results['fleet_workload']['fleet_goals'])==2
 results['aerial_command']=app.aerial_swarm_tab.command
 assert 'aerial_swarm.launch.py' in results['aerial_command']
 results['native_control_pages']=[app.control_notebook.tab(t,'text') for t in app.control_notebook.tabs()]
 results['native_main_pages']=[app.notebook.tab(t,'text') for t in app.notebook.tabs()]
finally:
 app._on_close()
 Path(os.environ['LAB_EXTENSION_RUN'],'native-gui-final.json').write_text(json.dumps(results,indent=2));print(json.dumps(results),flush=True)
