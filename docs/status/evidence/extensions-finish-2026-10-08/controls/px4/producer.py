#!/usr/bin/env python3
"""Bounded, owned GUI flight with seven directions and independent body truth.

Run serially on the SSD with sourced ROS/PX4 environments and a display.
This screen measures manual control on one map; it is not aerial navigation.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_lab_gui.launcher import SimulationLauncherGui


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map', default='nav_empty')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.parent.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Flight logs must be on the mounted workspace SSD')
    args.output.mkdir(exist_ok=False)
    out, root = args.output, Path(__file__).resolve().parents[1]
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())
    app = SimulationLauncherGui(); app.update()
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('px4_seven_direction_gui_observer')
    executor = SingleThreadedExecutor(); executor.add_node(node)
    body, commands, states, rotors = [], [], [], []
    report = dict(robot='px4_x500', backend='gazebo', map=args.map, mode='flight', passed=False,
        scope='Normal owned Tk Run/Takeoff/seven direction buttons/altitude/release/Stop/publisher loss/Land. Native X500/PX4 and independent Gazebo body; one open static map, no obstacle planner.')
    last_log = 0.

    def receive_body(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        body.append(dict(wall=time.monotonic(), x=p.x, y=p.y, z=p.z,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))

    node.create_subscription(Odometry, '/px4/odometry_truth', receive_body, 20)
    node.create_subscription(Twist, '/key_vel', lambda m: commands.append(dict(
        wall=time.monotonic(), vx=m.linear.x, vy=m.linear.y, vz=m.linear.z, wz=m.angular.z)), 20)
    node.create_subscription(String, '/px4/status', lambda m: states.append(json.loads(m.data)), 20)
    node.create_subscription(JointState, '/joint_states', lambda m: rotors.append(list(m.velocity)), 20)

    def pump(plant=True):
        nonlocal last_log
        app.update()
        for _ in range(20):
            executor.spin_once(timeout_sec=0.)
        if time.monotonic()-last_log > 3:
            (out/'live-gui-output.log').write_text(app.output.get('1.0', 'end'))
            last_log = time.monotonic()
        if plant:
            assert app.process and app.process.poll() is None, 'Owned Flight launch exited'
        time.sleep(.005)

    def until(predicate, budget=20, plant=True):
        deadline = time.monotonic()+budget
        while not predicate() and time.monotonic() < deadline:
            pump(plant)
        assert predicate(), ('Flight timeout', states[-1] if states else {})

    def interval(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            pump()
        assert body and time.monotonic()-body[-1]['wall'] < 1., 'Stale independent body'

    def yaw_change(points):
        return sum(math.atan2(math.sin(b['yaw']-a['yaw']), math.cos(b['yaw']-a['yaw']))
                   for a, b in zip(points, points[1:]))

    def xyz(point):
        return [point[key] for key in 'xyz']

    def stop():
        app.drone_drive_widgets['Stop'].invoke()
        interval(2.5)
        first = dict(body[-1]); interval(1.)
        drift = math.dist(xyz(first), xyz(body[-1]))
        angle = abs(math.atan2(math.sin(body[-1]['yaw']-first['yaw']), math.cos(body[-1]['yaw']-first['yaw'])))
        assert drift < .2 and angle < .06, ('Stopped drift', drift, angle)
        return dict(drift_m=drift, yaw_drift_rad=angle)

    try:
        app.robot_var.set('px4_x500'); app.simulator_var.set('gazebo')
        app.mode_var.set('flight'); app.map_var.set(args.map); app.gui_var.set('false')
        app._update_from_selection(); app.show_tab('Drone'); app.update()
        assert app.start_button.instate(['!disabled']), app.validation_var.get()
        report['command'] = app.command_var.get()
        paths = [p for package in ('robot_lab_gui', 'robot_lab_adapter', 'robot_lab_bringup')
                 for p in (root/'src'/package).rglob('*.py')]
        paths.append(root/'src/robot_lab_robots/config/robots.yaml')
        from ament_index_python.packages import get_package_share_directory
        world_profile = app.map_profiles[args.map]
        world = Path(world_profile['gazebo']['world_path'])
        if not world.is_absolute():
            world = Path(get_package_share_directory(world_profile['gazebo']['world_package']))/world
        px4_root = Path(os.environ.get('PX4_ROOT', '/workspace/molar/px4/PX4-Autopilot'))
        px4_files = [px4_root/'build/px4_sitl_default/bin/px4',
                     px4_root/'Tools/simulation/gz/models/x500/model.sdf',
                     px4_root/'Tools/simulation/gz/models/x500_base/model.sdf']
        manifest = dict(revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
            domain=os.environ.get('ROS_DOMAIN_ID'), profile=app.robot_profiles['px4_x500'],
            selected_world=dict(path=str(world), sha256=hashlib.sha256(world.read_bytes()).hexdigest(), profile=world_profile),
            px4_revision=subprocess.check_output(['git', '-C', str(px4_root), 'rev-parse', 'HEAD'], text=True).strip(),
            px4_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in px4_files},
            source_sha256={str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (out/'source-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        app.start_button.invoke(); report['owned_pid'] = app.process.pid
        until(lambda: body and states and states[-1].get('position_ready') and states[-1].get('phase') == 'idle', 100)
        initial = dict(body[-1]); count = len(commands)
        app.drive_input_checkbox.invoke(); interval(2.)
        report['neutral'] = dict(commands=len(commands)-count, displacement_m=math.dist(xyz(initial), xyz(body[-1])),
                                 armed=states[-1].get('armed'))
        assert report['neutral']['commands'] == 0 and report['neutral']['displacement_m'] < .08 and not report['neutral']['armed']
        app.flight_buttons[0].invoke()
        until(lambda: states and states[-1].get('phase') == 'flying', 55)
        interval(3.)
        report['takeoff_height_m'] = body[-1]['z']-initial['z']
        assert abs(report['takeoff_height_m']-3.) < .35
        report['directions'] = {}
        for label, sign in (('Turn Left', 1), ('Turn Right', -1)):
            first, command_start = len(body), len(commands)
            app.drone_drive_widgets[label].invoke(); interval(2.)
            app.drone_drive_widgets[label].invoke(); interval(2.)
            points = body[first:]; turn = yaw_change(points)
            xy = math.dist([points[0]['x'], points[0]['y']], [points[-1]['x'], points[-1]['y']])
            maximum = max(abs(c['wz']) for c in commands[command_start:])
            report['directions'][label] = dict(yaw_rad=turn, net_xy_m=xy, command_max_yaw_rad_s=maximum, stop=stop())
            assert .25 < turn*sign < .85 and xy < .25 and maximum <= .300001, report['directions'][label]
        for label, axis, sign in (('Forward', 0, 1), ('Reverse', 0, -1), ('Strafe L', 1, 1), ('Strafe R', 1, -1)):
            first = dict(body[-1])
            app.drone_drive_widgets[label].invoke(); interval(2.)
            app.drone_drive_widgets[label].invoke(); interval(2.)
            last = body[-1]; dx, dy = last['x']-first['x'], last['y']-first['y']
            relative = [dx*math.cos(first['yaw'])+dy*math.sin(first['yaw']),
                        -dx*math.sin(first['yaw'])+dy*math.cos(first['yaw'])]
            report['directions'][label] = dict(body_frame_travel_m=relative, stop=stop())
            assert relative[axis]*sign > .3 and abs(relative[1-axis]) < .25, report['directions'][label]
        report['altitude'] = {}
        for sign, label in ((1., 'up'), (-1., 'down')):
            first = dict(body[-1])
            app.drive_altitude_widgets[sign].invoke(); interval(2.)
            app.drive_altitude_widgets[sign].invoke(); interval(2.)
            height = body[-1]['z']-first['z']
            report['altitude'][label] = dict(height_change_m=height, stop=stop())
            assert .3 < height*sign < 1.6, report['altitude'][label]
        app.drone_drive_widgets['Forward'].invoke(); interval(1.5)
        # Disable only the owned input tick: actual ROS publisher loss must
        # be handled by the controller watchdog while the plant keeps running.
        app.after_cancel(app.drive_repeat_job); app.drive_repeat_job = None
        interval(3.); first = dict(body[-1]); interval(1.)
        report['publisher_loss_drift_m'] = math.dist(xyz(first), xyz(body[-1]))
        assert report['publisher_loss_drift_m'] < .2
        stop()
        app.flight_buttons[2].invoke()
        until(lambda: states and states[-1].get('phase') == 'idle' and not states[-1].get('armed'), 45)
        report['landing_height_error_m'] = abs(body[-1]['z']-initial['z'])
        assert report['landing_height_error_m'] < .1
        report['rotor_max_rad_s'] = max(abs(v) for velocities in rotors for v in velocities)
        assert report['rotor_max_rad_s'] > 10 and len(body) > 100
        app._stop_launch(); until(lambda: not app._launch_running, 60, False)
        report['launch_returncode'] = app.process.poll()
        assert report['launch_returncode'] == 0
        report['passed'] = True
    except BaseException as exc:
        report['error'] = repr(exc)
        raise
    finally:
        if app.process and app.process.poll() is None:
            if states and states[-1].get('armed'):
                app.flight_buttons[2].invoke()
                try:
                    until(lambda: not states[-1].get('armed'), 45)
                except Exception:
                    pass
            app._stop_launch()
            try:
                until(lambda: not app._launch_running, 60, False)
            except Exception:
                pass
        report['launch_returncode'] = app.process.poll() if app.process else None
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'trace.json').write_text(json.dumps(dict(body=body, commands=commands, states=states, rotors=rotors))+'\n')
        (out/'gui-output.log').write_text(app.output.get('1.0', 'end'))
        executor.shutdown(); node.destroy_node(); app._on_close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
