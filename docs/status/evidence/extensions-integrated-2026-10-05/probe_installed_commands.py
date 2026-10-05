"""Check every installed selection in the actual Tk Launch controls."""
import json
from pathlib import Path
from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.lab_tabs import AssetsTab

app = SimulationLauncherGui(); app.update()
report = {'scope':'Actual Tk selections/commands only; not simulator or mission qualification', 'robots':{},'maps':{}}
try:
    for robot,profile in app.robot_profiles.items():
        if not profile.get('source_id'):continue
        backend = 'mujoco' if profile.get('native_mjcf') else 'pybullet'
        app.robot_var.set(robot); app.mode_var.set('display'); app.map_var.set('nav_empty')
        app.simulator_var.set(backend); app._update_from_selection(); app.update()
        command=app.command_var.get()
        assert 'robot_model:='+robot in command and 'simulator:='+backend in command, (robot,command)
        assert app.start_button.instate(['!disabled']), app.validation_var.get()
        report['robots'][robot]=command
    app.robot_var.set('bumperbot'); app.simulator_var.set('mujoco')
    for world,profile in app.map_profiles.items():
        if not profile.get('source_id'):continue
        app.map_var.set(world); app._update_from_selection(); app.update()
        command=app.command_var.get()
        assert 'map_name:='+world in command and profile['gazebo']['world_path'] in command, (world,command)
        report['maps'][world]=command
    assets=next(app.nametowidget(tab) for tab in app.notebook.tabs() if isinstance(app.nametowidget(tab),AssetsTab))
    linked={p['id'] for p in assets.rows.values() if p.get('support')=='Installed equivalent model'}
    assert len(linked)==8,linked
    report.update(urdfhub_equivalent_profiles=sorted(linked),passed=True)
finally:
    (Path(__file__).parent/'installed-command-report.json').write_text(json.dumps(report,indent=2)+'\n')
    for job in app.tk.call('after','info'):app.tk.call('after','cancel',job)
    app.destroy()
