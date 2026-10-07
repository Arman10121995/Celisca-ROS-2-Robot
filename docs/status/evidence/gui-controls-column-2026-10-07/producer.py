"""Actual installed Tk workspace selections and screenshots; no mission claims."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

from PIL import ImageGrab
from robot_lab_gui.launcher import SimulationLauncherGui

root = Path.cwd()
out = Path(os.environ['GUI_DESIGN_OUT'])
out.mkdir(parents=True, exist_ok=False)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
(out/'producer.py').write_bytes(Path(__file__).read_bytes())
files = [p for package in ('robot_lab_gui', 'robot_lab_utils', 'robot_lab_bringup')
         for p in (root/'src'/package).rglob('*') if p.is_file() and p.suffix in ('.py', '.yaml')]
(out/'source-manifest.json').write_text(json.dumps(dict(
    git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
    source_sha256={str(p.relative_to(root)): sha(p) for p in files}), indent=2)+'\n')
app = SimulationLauncherGui()
report = dict(scope='Actual Tk workspace/selection/command/neutral screenshots; no movement or robot mission', cases=[])

def settle():
    end = time.monotonic()+.35
    while time.monotonic() < end:
        app.update(); time.sleep(.01)

def capture(name):
    settle()
    assert app.process is None and app.ros_node is None
    assert app.command_var.get() == app.command_preview.get('1.0', 'end-1c')
    x, y = app.winfo_rootx(), app.winfo_rooty()
    ImageGrab.grab(bbox=(x, y, x+app.winfo_width(), y+app.winfo_height())).save(out/(name+'.png'))
    report['cases'].append(dict(name=name, dimensions=[app.winfo_width(), app.winfo_height()],
        robot=app.robot_var.get(), map=app.map_var.get(), mode=app.mode_var.get(),
        command=app.command_var.get(), structural_tags=app.robot_tags_var.get(),
        workspace=app.notebook.tab(app.notebook.select(), 'text'),
        control=app.control_notebook.tab(app.control_notebook.select(), 'text'),
        keyboard_enabled=app.drive_input_enabled.get(), ros_node_created=app.ros_node is not None))

def choose(category, subtype, robot):
    app.robot_category_combo.set(category)
    app.robot_category_combo.event_generate('<<ComboboxSelected>>')
    app.robot_subtype_combo.set(subtype)
    app.robot_subtype_combo.event_generate('<<ComboboxSelected>>')
    app.robot_var.set(robot); app._update_from_selection(); settle()

try:
    capture('launch-drive')
    choose('Mobile robots', 'Four-wheel drive', 'four_wheel_steer_car')
    capture('four-wheel-tags')
    choose('Legged robots', 'Four legs', 'unitree_go2')
    capture('legged-tags')
    choose('Manipulators', 'Single arm', 'menagerie_franka_emika_panda')
    app.simulator_var.set('mujoco'); app.map_var.set('nav_empty'); app.mode_var.set('display')
    app._update_from_selection(); app.show_tab('Arm'); capture('arm-and-limits')
    app.show_tab('Hand'); capture('hand-and-limits')
    choose('Drones', 'Quadrotor', 'px4_x500')
    app.mode_var.set('flight'); app.simulator_var.set('gazebo'); app._update_from_selection()
    app.show_tab('Drone'); capture('drone-and-limits')
    app.robot_category_var.set('All categories'); app.robot_subtype_var.set('All types')
    app.robot_var.set('bumperbot'); app.mode_var.set('display'); app._update_from_selection()
    app.show_tab('Drive'); app.geometry('1024x768'); capture('launch-1024')
    app.drive_page.canvas.yview_moveto(1.); capture('drive-limits-1024')
    app.geometry('1600x980'); app.show_tab('Registry'); capture('registry-classifications')
    report['control_pages'] = [app.control_notebook.tab(t, 'text') for t in app.control_notebook.tabs()]
    report['visible_families'] = len(app.robot_combo['values'])
finally:
    app._on_close()
    report['closed'] = True
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
