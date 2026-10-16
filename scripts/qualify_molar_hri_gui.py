#!/usr/bin/env python3
"""Bounded normal-GUI MOLAR simulation probe; no physical robot interfaces."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from robot_lab_gui.launcher import SimulationLauncherGui


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['gazebo', 'mujoco', 'pybullet', 'isaac'], required=True)
    parser.add_argument('--robot', choices=['molar_hri', 'probot_l'], default='molar_hri')
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--startup-budget', type=float, default=240.)
    args = parser.parse_args()
    out = args.output.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.parent.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Use the workspace SSD')
    out.mkdir(exist_ok=False)
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())
    app = SimulationLauncherGui()
    app.update()
    rclpy.init()
    node = rclpy.create_node('molar_simulation_acceptance_observer')
    body, joints = [], []
    report = dict(robot=args.robot, backend=args.backend, map=args.map, mode='display', passed=False,
                  phases={}, scope='Bounded actual GUI differential-drive/lift/two-stage telescope/roller/Stop/watchdog; not navigation or physical robot acceptance',
                  domain=os.environ.get('ROS_DOMAIN_ID'))

    def body_state(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        body.append(dict(wall=time.monotonic(), sim=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                         x=p.x, y=p.y, z=p.z,
                         yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                         tilt=math.acos(max(-1., min(1., 1-2*(q.x*q.x+q.y*q.y))))))

    def joint_state(msg):
        joints.append(dict(wall=time.monotonic(), position=dict(zip(msg.name, msg.position)),
                           velocity=dict(zip(msg.name, msg.velocity))))

    node.create_subscription(Odometry, '/odom/ground_truth', body_state, qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', joint_state, 30)

    def pump():
        app.update()
        rclpy.spin_once(node, timeout_sec=.003)
        if app.process and app.process.poll() is not None:
            raise RuntimeError('Owned launch exited: ' + str(app.process.poll()))

    def wait_for(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            pump()
            if predicate():
                return
        raise TimeoutError('Condition timeout; inspect GUI/launch log')

    def interval(seconds):
        assert body
        start = body[-1]['sim']
        wait_for(lambda: body[-1]['sim']-start >= seconds, max(60., seconds*40))

    def phase(label, seconds):
        first = len(body)-1
        interval(seconds)
        samples = body[first:]
        assert len(samples) > 5
        a, b = samples[0], samples[-1]
        result = dict(displacement=math.hypot(b['x']-a['x'], b['y']-a['y']),
                      yaw_change=sum(math.atan2(math.sin(y['yaw']-x['yaw']), math.cos(y['yaw']-x['yaw'])) for x,y in zip(samples,samples[1:])),
                      max_tilt=max(x['tilt'] for x in samples), sim_seconds=b['sim']-a['sim'])
        report['phases'][label] = result
        return result

    def key(value, down=True):
        (app._drive_key_press if down else app._drive_key_release)(SimpleNamespace(keysym=value, widget=app))

    def command(joint, value):
        app.control_notebook.select(app.molar_controls)
        wait_for(app.molar_controls.ready, 12.)
        app.molar_controls.target_vars[joint].set(value)
        assert app.molar_controls.apply_buttons[joint].instate(['!disabled'])
        app.molar_controls.apply_buttons[joint].invoke()

    def q(name):
        assert joints and time.monotonic()-joints[-1]['wall'] < 1.
        return joints[-1]['position'][name]

    lift, stage, child, roller = 'conveyor_lift_joint', 'telescope_stage1_joint', 'telescope_stage2_joint', 'transfer_roller_joint'
    try:
        app.robot_var.set(args.robot)
        app.simulator_var.set(args.backend)
        app.map_var.set(args.map)
        app.mode_var.set('display')
        app.gui_var.set('false')
        app._update_from_selection()
        app.update()
        assert app.start_button.instate(['!disabled']), app.validation_var.get()
        report['command'] = app.command_var.get()
        report['source_revision'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
        app.start_button.invoke()
        wait_for(lambda: len(body)>30 and len(joints)>20, args.startup_budget)
        app.control_notebook.select(app.molar_controls)
        wait_for(app.molar_controls.ready, 20.)
        report['initial_articulation_state'] = app.molar_controls.state
        assert phase('neutral_start', 1.5)['displacement'] < .04
        app.control_notebook.select(app.drive_page)
        app.update()
        if not app.drive_input_enabled.get():
            app.drive_input_checkbox.invoke()
        key('w')
        assert phase('forward', 3.)['displacement'] > .15
        key('a')
        turn = phase('simultaneous_forward_turn', 2.)
        assert turn['displacement'] > .06 and abs(turn['yaw_change']) > .15
        key('a', False)
        key('w', False)
        app._stop_drive()
        interval(1.5)
        assert phase('base_stop', 1.)['displacement'] < .03

        before_lift = q(lift)
        command(lift, .06)
        interval(3.)
        report['lift'] = dict(before=before_lift, after=q(lift))
        assert q(lift)-before_lift > .025, report['lift']
        command(stage, .06)
        interval(2.)
        report['telescope'] = dict(stage1=q(stage), stage2=q(child), total_tip=q(stage)+q(child))
        assert q(stage) > .035 and q(child) > .035 and abs(q(stage)-q(child)) < .012, report['telescope']
        app.control_notebook.select(app.drive_page)
        app.update()
        if not app.drive_input_enabled.get():
            app.drive_input_checkbox.invoke()
        key('w')
        assert phase('extended_base_inhibit', 1.5)['displacement'] < .03
        key('w', False)
        app._stop_drive()

        command(roller, 2.)
        interval(1.5)
        velocity = joints[-1]['velocity'].get(roller, 0.)
        report['roller_velocity'] = velocity
        assert velocity > .5, velocity
        held = {lift:q(lift), stage:q(stage), child:q(child)}
        app.molar_controls.stop_button.invoke()
        interval(1.5)
        report['stop_hold'] = {name: q(name)-value for name,value in held.items()}
        assert max(abs(x) for x in report['stop_hold'].values()) < .015
        assert abs(joints[-1]['velocity'].get(roller, 0.)) < .1
        command(roller, 2.)
        interval(.8)
        # Leaving the owned control tab removes the heartbeat; no new target
        # may survive publisher loss, and lift/slide must not snap to zero.
        app.control_notebook.select(app.drive_page)
        interval(2.)
        report['watchdog_roller_velocity'] = joints[-1]['velocity'].get(roller, 0.)
        assert abs(report['watchdog_roller_velocity']) < .1
        assert q(lift) > .02 and q(stage) > .03
        command(stage, 0.)
        interval(2.)
        command(lift, 0.)
        interval(3.)
        assert abs(q(stage)) < .01 and abs(q(child)) < .01 and q(lift) < .015
        assert all(math.isfinite(v) for item in body for v in (item['x'],item['y'],item['z'],item['tilt']))
        assert max(item['tilt'] for item in body) < .3
        report['passed'] = True
    except BaseException as exc:
        report['exception'] = repr(exc)
        raise
    finally:
        app._stop_drive()
        if app.process and app.process.poll() is None:
            app._stop_launch()
            deadline = time.monotonic()+60.
            while app._launch_running and time.monotonic()<deadline:
                app.update()
                rclpy.spin_once(node, timeout_sec=.01)
        report['launch_returncode'] = app.process.poll() if app.process else None
        if report['launch_returncode'] != 0:
            report['passed'] = False
        report['body_samples'], report['joint_samples'] = len(body), len(joints)
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'trace.json').write_text(json.dumps(dict(body=body,joints=joints)))
        (out/'gui-output.log').write_text(app.output.get('1.0','end'))
        node.destroy_node()
        app._on_close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
