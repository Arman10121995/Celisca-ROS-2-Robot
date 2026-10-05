"""Measure installed GUI selection, native state, and owned Run/Stop."""
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.lab_tabs import AssetsTab

out = Path(__file__).parent/os.environ.get('PROBE_OUTPUT','native')
out.mkdir(exist_ok=True)
robot = os.environ.get('PROBE_ROBOT','menagerie_franka_emika_panda')
backend = os.environ.get('PROBE_BACKEND','mujoco')
app = SimulationLauncherGui(); app.update()
result = {'scope':'Installed GUI selection and passive native display; no movement or mission claim',
          'domain':os.environ.get('ROS_DOMAIN_ID'), 'commands':[], 'checks':{}}
def select(robot, backend, world):
    app.robot_var.set(robot); app.simulator_var.set(backend)
    app.map_var.set(world); app.mode_var.set('display')
    app._update_from_selection(); app.update()
    command = app.command_var.get()
    assert 'robot_model:='+robot in command, command
    assert 'simulator:='+backend in command, command
    assert 'map_name:='+world in command, command
    assert app.start_button.instate(['!disabled']), app.validation_var.get()
    result['commands'].append(command)

node = None
try:
    for selected_robot,selected_backend,world in [('menagerie_franka_emika_panda','mujoco','nav_empty'),
            ('asset_fetch','pybullet','dataset_room2'),
            ('asset_franka_panda_panda','gazebo','dataset_bookstore'),
            ('bumperbot','mujoco','celisca_floor_1')]:
        select(selected_robot,selected_backend,world)
    result['checks']['selection_commands']=True
    assets = next(app.nametowidget(tab) for tab in app.notebook.tabs()
                  if isinstance(app.nametowidget(tab), AssetsTab))
    row = next(row for row,p in assets.rows.items() if p.get('id')=='menagerie_franka_emika_panda')
    assets.tree.selection_set(row); assets.open_selected(); app.update()
    assert app.robot_var.get()=='menagerie_franka_emika_panda'
    assert app.notebook.tab(app.notebook.select(),'text')=='Launch'
    result['checks']['installed_shortcut']=True
    select(robot,backend,'dataset_room2')
    app.gui_var.set('true'); app._update_from_selection(); app.update()
    rclpy.init(); node=rclpy.create_node('installed_display_observer')
    clocks, joints, frames, descriptions = [], [], [], []
    node.create_subscription(Clock,'/clock',lambda m:clocks.append(m.clock.sec+m.clock.nanosec/1e9),10)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.append(m),10)
    node.create_subscription(TFMessage,'/tf',lambda m:frames.extend(t.child_frame_id for t in m.transforms),10)
    node.create_subscription(String,'/robot_description',lambda m:descriptions.append(m.data),
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    result['runtime_command']=app.command_var.get()
    app._start_launch(); assert app.process is not None
    result['owned_launch_pid']=app.process.pid
    deadline=time.monotonic()+float(os.environ.get('PROBE_TIMEOUT','35'))
    while time.monotonic()<deadline:
        app.update(); rclpy.spin_once(node,timeout_sec=.01)
        if len(clocks)>100 and len(joints)>100 and descriptions and clocks[-1]-clocks[0]>=2.:break
        if app.process.poll() is not None:break
    assert len(clocks)>100, (len(clocks),app.process.poll())
    assert all(b>=a for a,b in zip(clocks,clocks[1:])) and clocks[-1]-clocks[0]>=2., clocks
    result.update(observed_clock_samples=len(clocks),observed_joint_samples=len(joints),observed_joint_names=joints[-1].name if joints else [])
    assert len(joints)>100 and len(joints[-1].name)==(10 if backend=='gazebo' else 9), (len(joints),joints[-1].name if joints else [])
    assert all(math.isfinite(v) for m in joints for v in m.position+m.velocity+m.effort)
    assert len(set(frames))>=9, set(frames)
    assert descriptions and '<robot' in descriptions[-1]
    result.update(clock_samples=len(clocks),distinct_clock_samples=len(set(clocks)),sim_time_span=clocks[-1]-clocks[0],
        joint_samples=len(joints),joint_names=joints[-1].name,positions=list(joints[-1].position),
        measured_body_frames=sorted(set(frames)),description_bytes=len(descriptions[-1]))
    result['checks'].update(clock=True,finite_native_joint_states=True,measured_native_body_tf=True,description=True)
    from PIL import ImageGrab
    ImageGrab.grab().save(out/'gui-native-display.png')
    app._stop_launch()
    deadline=time.monotonic()+20
    while time.monotonic()<deadline and app._launch_running:
        app.update(); time.sleep(.05)
    assert app.process.poll() is not None
    result['launch_returncode']=app.process.returncode
    assert not app._launch_running
    result['checks']['owned_stop']=True
    result['passed']=True
finally:
    if app.process and app.process.poll() is None:
        app._stop_launch()
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and app.process.poll() is None:
            app.update(); time.sleep(.05)
    (out/'gui-native-output.log').write_text(app.output.get('1.0','end'))
    (out/'gui-native-report.json').write_text(json.dumps(result,indent=2)+'\n')
    if node:node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    for job in app.tk.call('after','info'):app.tk.call('after','cancel',job)
    app.destroy()
