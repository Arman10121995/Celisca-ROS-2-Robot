#!/usr/bin/env python3
"""Check actual installed map selection/autofill and Health without Run.

Use sourced ROS/SSD environments and Xvfb. No robot mission is qualified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import time

import yaml
from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.lab_tabs import HealthTab, STATUS_YAML

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
report = dict(scope='Actual Tk selections/autofill and current Health only; no Run or robot mission.',
              completed=False, records=[])
app = SimulationLauncherGui()
try:
    app.update()
    profiles = yaml.safe_load(Path('/workspace/molar/robot_lab_runtime/external_assets/installed/maps.yaml').read_text())['maps']
    for name, profile in profiles.items():
        for robot in ('bumperbot', 'labbot'):
            app.robot_var.set(robot)
            app.simulator_var.set('mujoco')
            app.map_var.set(name)
            app.mode_var.set('nav')
            app._update_from_selection()
            app.update()
            command = app.command_var.get()
            args_list = shlex.split(command)
            assert app.mode_var.get() == 'nav', app.validation_var.get()
            assert app.start_button.instate(['!disabled']), app.validation_var.get()
            assert f'robot_model:={robot}' in args_list and f'map_name:={name}' in args_list
            assert 'map_yaml:='+profile['map']['path'] in args_list
            assert any(a.startswith('global_planning:=') for a in args_list)
            assert any(a.startswith('local_planning:=') for a in args_list)
            report['records'].append(dict(map=name, robot=robot, backend='mujoco',
                command=command, map_yaml=profile['map']['path']))
    app.show_tab('Health')
    health = next(child for child in app.notebook.winfo_children() if isinstance(child, HealthTab))
    health._show_platform_status()
    app.update()
    text = health.summary.get('1.0', 'end')
    for required in ('all 17', '34 actual', '0.1305', 'eight', 'ca65f35'):
        assert required in text, required
    (args.output/'health-visible.txt').write_text(text)
    report['health'] = dict(status_path=str(STATUS_YAML),
        status_sha256=hashlib.sha256(STATUS_YAML.read_bytes()).hexdigest(),
        visible_text_sha256=hashlib.sha256(text.encode()).hexdigest())
    assert app.process is None and not app._launch_running
    report['completed'] = True
finally:
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    app._on_close()
print('Actual Tk map/autofill records:', len(report['records']), 'Health:', report['completed'])
