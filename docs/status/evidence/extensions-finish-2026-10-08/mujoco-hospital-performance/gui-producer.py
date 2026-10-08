#!/usr/bin/env python3
"""Check preserved measured robot selectors and updated Health without Run."""
import argparse
import hashlib
import json
from pathlib import Path

from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.lab_tabs import HealthTab, STATUS_YAML

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
app = SimulationLauncherGui()
report = dict(scope='Actual normal Tk mode/command selections and Health only; no Run or robot mission.',
              completed=False, selections=[])
try:
    app.update()
    for robot in ('asset_husky','asset_turtlebot3_burger','asset_turtlebot4_standard','asset_turtlebot4_lite'):
        modes=('display','loc','slam','nav') if robot=='asset_turtlebot3_burger' else ('display','loc','slam','3d_slam','nav')
        for backend in ('gazebo','mujoco','pybullet','isaac'):
            for mode in modes:
                app.robot_var.set(robot)
                app.simulator_var.set(backend)
                app.map_var.set('nav_empty')
                app.mode_var.set(mode)
                app._update_from_selection()
                app.update()
                assert app.mode_var.get()==mode, app.validation_var.get()
                assert app.start_button.instate(['!disabled']), app.validation_var.get()
                command=app.command_var.get()
                assert f'robot_model:={robot}' in command and f'simulator:={backend}' in command
                assert f'mode:={mode}' in command
                report['selections'].append(dict(robot=robot,backend=backend,mode=mode,command=command))
    app.show_tab('Health')
    health=next(c for c in app.notebook.winfo_children() if isinstance(c,HealthTab))
    health._show_platform_status()
    text=health.summary.get('1.0','end')
    for required in ('P4: partial','0.130892','151 full sourced integration','typed-array'):
        assert required in text, required
    (args.output/'health.txt').write_text(text)
    report['status_sha256']=hashlib.sha256(STATUS_YAML.read_bytes()).hexdigest()
    assert app.process is None and not app._launch_running
    report['completed']=True
finally:
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    app._on_close()
print('Actual GUI selections:',len(report['selections']),'Health:',report['completed'])
