"""Real GUI/ROS/MuJoCo arm screen; predefined limits, no pose writes."""
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_lab_gui.launcher import SimulationLauncherGui

out = Path(__file__).parent
report = {'scope': 'Native Panda / MuJoCo / dataset_room2, actual GUI controls and ROS action',
          'limits': {'goal_error_rad': .03, 'jog_min_motion_rad': .07, 'stopped_velocity_rad_s': .03,
                     'stopped_drift_rad': .02, 'watchdog_s': .8}, 'checks': {}}
rclpy.init()
node = rclpy.create_node('panda_measured_observer')
states, joints = [], []
node.create_subscription(String, '/arm/status', lambda m: states.append(json.loads(m.data)), 10)
node.create_subscription(JointState, '/joint_states', lambda m: joints.append(m), 10)
client = ActionClient(node, FollowJointTrajectory, '/arm/follow_joint_trajectory')
app = SimulationLauncherGui(); app.update()
arm = app.arm_tab

def pump():
    app.update()
    rclpy.spin_once(node, timeout_sec=.01)

def wait(condition, budget=20):
    end = time.monotonic()+budget
    while not condition() and time.monotonic() < end:
        pump()
        if app.process and app.process.poll() is not None:
            raise AssertionError('Owned launch exited: '+str(app.process.returncode))
    assert condition(), 'Timed out: '+(states[-1]['status'] if states else 'no state')

def measured():
    message = joints[-1]
    return np.array([message.position[message.name.index('joint'+str(i))] for i in range(1, 8)])

def goal(delta, duration=3.):
    request = FollowJointTrajectory.Goal()
    request.trajectory.joint_names = ['joint'+str(i) for i in range(1, 8)]
    target = measured().copy(); target[0] += delta
    point = JointTrajectoryPoint(positions=target.tolist())
    point.time_from_start.sec = int(duration)
    point.time_from_start.nanosec = int((duration-int(duration))*1e9)
    request.trajectory.points = [point]
    return request, target

def send(request):
    future = client.send_goal_async(request)
    wait(future.done)
    return future.result()

def settle():
    end = time.monotonic()+1.5
    while time.monotonic() < end: pump()
    position = measured().copy()
    end = time.monotonic()+.8
    while time.monotonic() < end: pump()
    velocity = np.array([joints[-1].velocity[joints[-1].name.index('joint'+str(i))] for i in range(1, 8)])
    drift = float(np.max(np.abs(measured()-position)))
    assert np.max(np.abs(velocity)) < .03 and drift < .02, (velocity, drift)
    return {'max_velocity_rad_s': float(np.max(np.abs(velocity))), 'drift_rad': drift}

try:
    app.robot_var.set('menagerie_franka_emika_panda'); app.simulator_var.set('mujoco')
    app.map_var.set('dataset_room2'); app.mode_var.set('display'); app.gui_var.set('true')
    app._update_from_selection(); app.update()
    report['command'] = app.command_var.get()
    assert 'arm_control:=panda' in report['command']
    app._start_launch(); report['owned_pid'] = app.process.pid
    wait(lambda: arm.ready() and joints and arm.jog_buttons[1].instate(['!disabled']), 35)
    app.show_tab('Arm'); app.update()
    report['checks']['gui_command_and_measured_controls'] = True
    report['jog'] = []
    for index in range(7):
        wait(lambda: arm.jog_buttons[index*2+1].instate(['!disabled']))
        before = measured().copy()
        arm.jog_buttons[index*2+1].invoke()
        wait(lambda: states and states[-1]['busy'])
        wait(lambda: not states[-1]['busy'] and arm.handle is None)
        after = measured().copy()
        difference = after[index]-before[index]
        error = abs(after[index]-(before[index]+.1))
        assert difference > .07 and error < .03, (index,difference,error,states[-1])
        report['jog'].append({'joint': index+1, 'motion_rad': float(difference), 'goal_error_rad': float(error)})
    wait(lambda: arm.home_button.instate(['!disabled']))
    arm.home_button.invoke(); wait(lambda: states[-1]['busy'])
    wait(lambda: not states[-1]['busy'] and arm.handle is None)
    home_error = float(np.max(np.abs(measured()-np.array(states[-1]['home']))))
    assert home_error < .03, home_error
    report['home_max_error_rad'] = home_error
    report['checks']['seven_joint_jog_and_home'] = True
    request, _ = goal(0.)
    request.trajectory.points[0].positions[0] = 4.
    assert not send(request).accepted
    request, _ = goal(.4, .2)
    assert not send(request).accepted
    request, _ = goal(0.)
    request.trajectory.joint_names[0] = 'finger_joint1'
    assert not send(request).accepted
    report['checks']['limit_velocity_and_joint_rejection'] = True
    request, _ = goal(.5)
    handle = send(request); assert handle.accepted
    result = handle.get_result_async()
    wait(lambda: states[-1]['busy'])
    start = measured().copy()
    wait(lambda: measured()[0]-start[0] > .05)
    future = handle.cancel_goal_async(); wait(future.done); wait(result.done)
    assert result.result().status == 5, result.result()
    report['cancel'] = settle()
    report['checks']['cancel'] = True
    request, _ = goal(.5)
    handle = send(request); assert handle.accepted
    result = handle.get_result_async(); start = measured().copy()
    wait(lambda: measured()[0]-start[0] > .05)
    arm.stop_button.invoke(); wait(result.done)
    assert result.result().status == 6 and result.result().result.error_code == -4
    report['stop'] = settle()
    report['checks']['gui_stop'] = True
    request, _ = goal(.5)
    handle = send(request); assert handle.accepted
    result = handle.get_result_async(); wait(lambda: states[-1]['busy'])
    arm.closed = True; arm.after_cancel(arm.job)
    drop = time.monotonic(); wait(result.done, 4)
    assert result.result().status == 6 and 'heartbeat lost' in result.result().result.error_string
    report['watchdog'] = {'observed_abort_wall_s': time.monotonic()-drop, **settle()}
    report['checks']['heartbeat_loss'] = True
    arm.closed = False; arm.poll()
    assert all(np.all(np.isfinite(message.position+message.velocity+message.effort)) for message in joints)
    assert len(joints) > 300
    report['joint_samples'] = len(joints)
    from PIL import ImageGrab
    ImageGrab.grab().save(out/'panda-arm-gui.png')
    app._stop_launch()
    end = time.monotonic()+20
    while app._launch_running and time.monotonic()<end: app.update(); time.sleep(.03)
    assert app.process.poll() is not None and not app._launch_running
    report['launch_returncode'] = app.process.returncode
    report['checks']['owned_cleanup'] = True
    report['passed'] = True
finally:
    if app.process and app.process.poll() is None:
        app._stop_launch()
        end = time.monotonic()+20
        while app.process.poll() is None and time.monotonic()<end: app.update(); time.sleep(.03)
    (out/'trace.json').write_text(json.dumps(states)+'\n')
    (out/'gui-output.log').write_text(app.output.get('1.0','end'))
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    for job in app.tk.call('after','info'): app.tk.call('after','cancel',job)
    app.destroy()
