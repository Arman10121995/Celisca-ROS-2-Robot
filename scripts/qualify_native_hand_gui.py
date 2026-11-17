#!/usr/bin/env python3
"""Normal-GUI grasp, lift, hold, return and release for a native hand (R5.8).

Drives the real Robot Lab window (Tk) for one native-hand robot on MuJoCo:

1. Select the robot, Display and the map; tick **Add free grasp object on
   next Run**; Run.
2. On the Hand page, press **Grasp object**.
3. Press **Lift and hold object**: UMI uses its source vertical motor, the
   other hands the fixture's lab support stage.
4. Hold for ``--hold`` seconds; press **Lower to start**, then **Release
   object**.

Each stage is judged by the hand controller's measured states:

- the grasp must reach ``holding``;
- the lift must reach ``holding`` (unsupported, at height, still for one
  simulated second) and stay so through the hold;
- the return must reach ``supported``;
- the release must reach ``supported_release``.

A 4 Hz trace of the measured grasp and transport state goes into the
report. Requires DISPLAY or xvfb-run, a sourced workspace and one simulation
at a time. ``--output`` must not exist yet.

  xvfb-run -a -s "-screen 0 1600x1000x24" python3 scripts/qualify_native_hand_gui.py \\
      --robot menagerie_robotiq_2f85 --output /workspace/molar/robot_lab_runtime/hands/robotiq
"""
import argparse
import json
import time
from pathlib import Path

from robot_lab_gui.launcher import SimulationLauncherGui


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--robot', required=True)
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--height', type=float, default=.05)
    parser.add_argument('--hold', type=float, default=3.)
    parser.add_argument('--contact', type=float, default=2., help='contact stop, N per group')
    parser.add_argument('--effort', type=float, default=.8, help='effort stop fraction')
    parser.add_argument('--pose', default='', help='hand pose for the grasp (default: the first)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(robot=args.robot, simulator='mujoco', map=args.map, height_m=args.height, steps=[],
                  trace=[], passed=False)
    app = SimulationLauncherGui()
    owner = app.hand_tab.generic_controls
    page = owner.hand_controls
    last_sample = [0.]

    def hand():
        return (owner.state or {}).get('hand', {})

    def sample():
        if time.monotonic() - last_sample[0] < .25 or not owner.state:
            return
        last_sample[0] = time.monotonic()
        grasp, transport = hand().get('object_grasp', {}), hand().get('object_transport', {})
        report['trace'].append(dict(
            wall_s=round(time.monotonic() - started, 2),
            grasp=grasp.get('state'), left_n=round(grasp.get('left_force_n', 0.), 3),
            right_n=round(grasp.get('right_force_n', 0.), 3), supported=grasp.get('supported'),
            slip_m=round(grasp.get('slip_m', 0.), 4), angular_slip_rad=round(grasp.get('angular_slip_rad', 0.), 4),
            transport=transport.get('state'), height_m=transport.get('measured_height_m')))

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.update()
            sample()
            time.sleep(.02)

    def until(predicate, seconds, label):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.update()
            sample()
            if predicate():
                report['steps'].append(dict(reached=label, wall_s=round(time.monotonic() - started, 2)))
                return
            time.sleep(.05)
        raise TimeoutError(label)

    def press(button, label):
        until(lambda: owner.ready() and button.instate(['!disabled']), 30, label + ' enabled')
        button.invoke()
        report['steps'].append(dict(pressed=button.cget('text'), feedback=page.feedback.get(),
                                    wall_s=round(time.monotonic() - started, 2)))

    def grasp_state():
        return hand().get('object_grasp', {}).get('state')

    def transport_state():
        return hand().get('object_transport', {}).get('state')

    started = time.monotonic()
    try:
        app.robot_var.set(args.robot)
        app.simulator_var.set('mujoco')
        app.mode_var.set('display')
        app.map_var.set(args.map)
        app.gui_var.set('false')
        app._update_from_selection()
        app.show_tab('Hand')
        app.update()
        page.fixture.set(True)
        page.fixture_changed()
        app.update()
        report['command'] = app.command_var.get()
        if 'grasp_fixture:=true' not in report['command']:
            raise RuntimeError('the GUI command has no grasp fixture: ' + report['command'])
        app._start_launch()
        until(lambda: owner.ready() and hand().get('object_present'), 300, 'hand ready with object')
        pump(3.)
        report['hand'] = dict(adapter=hand().get('adapter'), lift_mode=hand().get('object_lift_mode'),
                              qualification=hand().get('qualification'))
        page.contact.set(args.contact)
        page.effort.set(args.effort)
        if args.pose:
            page.pose.set(args.pose)
        press(page.grasp_object_button, 'Grasp object')
        until(lambda: grasp_state() == 'holding', 60, 'grasp holding')
        pump(1.)
        report['grasp'] = dict(hand().get('object_grasp', {}))
        page.lift_height.set(args.height)
        press(page.lift_button, 'Lift and hold object')
        # The grasp request left the transport 'interrupted'; see it start first.
        until(lambda: transport_state() in ('lifting', 'holding'), 20, 'lift started')
        until(lambda: transport_state() in ('holding', 'interrupted'), 60, 'lift settled')
        if transport_state() != 'holding':
            raise RuntimeError('lift interrupted: %s' % hand().get('object_transport'))
        pump(args.hold)
        report['lift'] = dict(hand().get('object_transport', {}), grasp=dict(hand().get('object_grasp', {})))
        if transport_state() != 'holding' or grasp_state() != 'holding':
            raise RuntimeError('hold not sustained: %s' % report['lift'])
        press(page.lower_button, 'Lower to start')
        until(lambda: transport_state() in ('supported', 'interrupted'), 60, 'return settled')
        report['return'] = dict(hand().get('object_transport', {}))
        if transport_state() != 'supported':
            raise RuntimeError('return interrupted: %s' % report['return'])
        press(page.release_object_button, 'Release object')
        until(lambda: grasp_state() == 'supported_release', 60, 'supported release')
        report['release'] = dict(hand().get('object_grasp', {}))
        report['passed'] = True
    except BaseException as exc:
        report['error'] = repr(exc)
        report['feedback'] = page.feedback.get()
        report['last_hand'] = dict(grasp=hand().get('object_grasp'), transport=hand().get('object_transport'))
    finally:
        if app.process and app.process.poll() is None:
            app._stop_launch()
            try:
                until(lambda: not app._launch_running, 90, 'shutdown')
            except TimeoutError:
                pass
        report['launch_returncode'] = app.process.poll() if app.process else None
        (args.output / 'report.json').write_text(json.dumps(report, indent=1, default=str) + '\n')
        (args.output / 'gui-output.log').write_text(app.output.get('1.0', 'end'))
        app._on_close()
    print(json.dumps({k: report.get(k) for k in ('passed', 'hand', 'error')}, indent=1, default=str))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
