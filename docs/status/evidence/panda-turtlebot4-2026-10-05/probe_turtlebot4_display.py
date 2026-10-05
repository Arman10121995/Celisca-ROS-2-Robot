"""Actual GUI Run/Stop and original TurtleBot 4 model state, per backend."""
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from robot_lab_gui.launcher import SimulationLauncherGui

robot = os.environ['PROBE_ROBOT']; backend = os.environ['PROBE_BACKEND']
out = Path(__file__).parent/(robot+'-'+backend); out.mkdir(exist_ok=True)
report = {'robot':robot,'backend':backend,'world':'dataset_room2',
          'scope':'GUI model/map Display with actual simulator states; no Drive, mapping or navigation claim',
          'checks':{},'passed':False}
app = SimulationLauncherGui(); app.update(); node = None
try:
    app.robot_var.set(robot); app.simulator_var.set(backend)
    app.map_var.set('dataset_room2'); app.mode_var.set('display'); app.gui_var.set('true')
    app._update_from_selection(); app.update()
    report['command'] = app.command_var.get()
    assert 'robot_model:='+robot in report['command'] and 'simulator:='+backend in report['command']
    assert 'map_name:=dataset_room2' in report['command']
    assert app.start_button.instate(['!disabled']), app.validation_var.get()
    profile = app.robot_profiles[robot]
    model = ET.parse(profile['xacro']).getroot()
    required = {joint.attrib['name'] for joint in model.findall('joint') if joint.attrib['type']!='fixed'}
    assert required == {'left_wheel_joint','right_wheel_joint','wheel_drop_left_joint','wheel_drop_right_joint'}
    rclpy.init(); node = rclpy.create_node('turtlebot4_display_observer')
    clocks, joints, transforms, descriptions = [], [], [], []
    node.create_subscription(Clock,'/clock',lambda m:clocks.append(m.clock.sec+m.clock.nanosec/1e9),10)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.append(m),10)
    node.create_subscription(TFMessage,'/tf',lambda m:transforms.extend(t.child_frame_id for t in m.transforms),10)
    node.create_subscription(TFMessage,'/tf_static',lambda m:transforms.extend(t.child_frame_id for t in m.transforms),
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/robot_description',lambda m:descriptions.append(m.data),
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    app._start_launch(); assert app.process
    report['owned_pid'] = app.process.pid
    end = time.monotonic()+float(os.environ.get('PROBE_BUDGET','55'))
    while time.monotonic()<end:
        app.update(); rclpy.spin_once(node,timeout_sec=.01)
        if len(clocks)>100 and len(joints)>100 and descriptions and clocks[-1]-clocks[0]>=2:break
        if app.process.poll() is not None:break
    assert len(clocks)>100 and len(joints)>100 and descriptions, (len(clocks),len(joints),len(descriptions))
    assert all(b>=a for a,b in zip(clocks,clocks[1:])) and clocks[-1]-clocks[0]>=2
    assert required <= set(joints[-1].name), joints[-1].name
    assert all(math.isfinite(v) for message in joints for v in message.position+message.velocity+message.effort)
    assert len(set(transforms))>=20, sorted(set(transforms))
    assert 'turtlebot4' in descriptions[-1] and 'left_wheel_joint' in descriptions[-1]
    report.update(clock_samples=len(clocks),sim_time_span=clocks[-1]-clocks[0],
        joint_samples=len(joints),joint_names=joints[-1].name,positions=list(joints[-1].position),
        body_frames=sorted(set(transforms)),description_bytes=len(descriptions[-1]))
    report['checks'].update(command=True,finite_model_joint_feedback=True,clock=True,description=True,tf=True)
    from PIL import ImageGrab
    ImageGrab.grab().save(out/'display.png')
    app._stop_launch(); end=time.monotonic()+20
    while app._launch_running and time.monotonic()<end:app.update();time.sleep(.03)
    assert app.process.poll() is not None and not app._launch_running
    report['launch_returncode']=app.process.returncode
    report['checks']['owned_cleanup']=True
    report['passed']=True
finally:
    if app.process and app.process.poll() is None:
        app._stop_launch();end=time.monotonic()+20
        while app.process.poll() is None and time.monotonic()<end:app.update();time.sleep(.03)
    (out/'gui-output.log').write_text(app.output.get('1.0','end'))
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if node:node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    for job in app.tk.call('after','info'):app.tk.call('after','cancel',job)
    app.destroy()
