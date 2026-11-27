#!/usr/bin/env python3
"""Mobile-arm page driven through the real Tk launcher on a live simulation.

Selects a profile whose articulation declares ``panel: arm`` (default: the
myAGV Pro + myCobot Pro630), launches it with the launcher's own Run path
(headless simulator), then through the page's widgets: Apply on joint 1,
a Jog on joint 2, Home, and Stop part-way through a long move.  Each step is
judged on the measured joint state the plant publishes. Requires DISPLAY or
xvfb-run, a sourced workspace, and one simulation at a time.
"""
import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import struct
import subprocess
import time

from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.urdf_arm_controls import POSE_SOURCE_LABELS


def screenshot(path):
    from PIL import Image
    data = subprocess.run(['xwd', '-root', '-silent'], check=True, capture_output=True).stdout
    fields = struct.unpack('>25I', data[:100])
    header, width, height = fields[0], fields[4], fields[5]
    byte_order, bytes_per_line, ncolors = fields[7], fields[12], fields[19]
    raw = data[header + ncolors * 12:header + ncolors * 12 + bytes_per_line * height]
    Image.frombuffer('RGB', (width, height), raw, 'raw', 'BGRX' if byte_order == 0 else 'XRGB',
                     bytes_per_line, 1).save(path)


def pick_place(app, page, args, report, press, settle, until, pump, screenshot):
    """Home, then one pick and place through the page's own widgets."""
    home = app._robot_config()['articulation']['home']
    press(page.home_button)
    settle(home, 'home', 60.)
    until(lambda: page.pick_fresh() and page.pick_state.get('truth_age_s') is not None, 60,
          'pick-and-place workflow and measured free-object feedback')
    for value, key in zip(args.pick_place, ('objects', 'supports'), strict=True):
        available = page.pick_state.get(key) or []
        if value not in available:
            raise ValueError(f'{value!r} is unavailable in {key}: {", ".join(available)}')
    page.pick_object.set(args.pick_place[0])
    page.pick_support.set(args.pick_place[1])
    if args.pose_source != 'truth':
        if args.pick_place[0] not in page.pick_state.get('rgbd_objects', []):
            raise ValueError('RGB-D target is unavailable for this robot and world')
        page.pick_source.set(POSE_SOURCE_LABELS[args.pose_source])
    previous = json.dumps(page.pick_state.get('last_result'))
    until(lambda: page.pick_run_button.instate(['!disabled']), 60 if args.mode == 'nav' else 20,
          'Run pick / place enabled')
    press(page.pick_run_button)
    until(lambda: page.pick_state.get('phase') == 'executing'
          or json.dumps(page.pick_state.get('last_result')) != previous, 30, 'task started')
    if page.pick_state.get('phase') != 'executing':
        report['pick_place'] = dict(result=page.pick_state.get('last_result'), page_status=page.pick_status.get())
        return
    deadline = time.monotonic() + args.task_timeout

    def finished():
        return (page.pick_state.get('phase') == 'idle'
                and json.dumps(page.pick_state.get('last_result')) != previous)

    if page.pick_state.get('mobile'):
        # A fetch drives first: photograph it carrying, mid-route.
        until(lambda: page.pick_state.get('stage') == 'carry' or finished(),
              max(0., deadline - time.monotonic()), 'carrying or task finished')
        if not finished():
            pump(12.)
            if not finished():
                screenshot(args.output / 'carrying.png')
                pump(1.)
    else:
        pump(8.)
        screenshot(args.output / 'carrying.png')
        pump(1.)
    if not finished():
        until(finished, max(0., deadline - time.monotonic()), 'task finished')
    result = page.pick_state.get('last_result') or {}
    report['pick_place'] = dict(result=result, page_status=page.pick_status.get())
    screenshot(args.output / 'placed.png')
    report['passed'] = bool(report['page_selected'] and result.get('passed')
                            and result.get('pose_source', 'truth') == args.pose_source)
    if args.mode == 'nav':
        stages = {stage.get('phase') for stage in result.get('stages', [])}
        report['passed'] = report['passed'] and {'carry_footprint_ready', 'carry_nav2'} <= stages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='asset_elephant_myagv_pro_pro630')
    parser.add_argument('--simulator', default='pybullet')
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--mode', default='display', choices=('display', 'nav'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--experimental', action='store_true', help='allow the selected profile’s experimental workflow')
    parser.add_argument('--pose-source', choices=('truth', 'rgbd'), default='truth',
                        help='object pose source used by --pick-place')
    task = parser.add_mutually_exclusive_group()
    task.add_argument('--pick-place', nargs=2, metavar=('OBJECT', 'SUPPORT'),
                        help='instead of the joint steps: Home, then Run pick / place of OBJECT onto SUPPORT '
                             'through the page (R5.7), judged by the node\'s truth checks')
    task.add_argument('--gripper-presets', action='store_true',
                      help='use Open and Close and measure the leader and its linked followers')
    task.add_argument('--flange-offset', type=float, nargs=3, metavar=('DX', 'DY', 'DZ'),
                      help='plan and execute this offset in metres from the measured source flange')
    parser.add_argument('--task-timeout', type=float, default=600.,
                        help='wall seconds for one pick/place or fetch task (a slow simulator needs more)')
    args = parser.parse_args()
    if args.pose_source != 'truth' and not args.pick_place:
        parser.error('--pose-source requires --pick-place')
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(robot=args.robot, simulator=args.simulator, map=args.map, mode=args.mode,
                  experimental=args.experimental, steps=[], passed=False)
    if args.pick_place:
        report['request'] = dict(object=args.pick_place[0], support=args.pick_place[1],
                                 pose_source=args.pose_source, wall_timeout_s=args.task_timeout)
        report['task_stages'] = []
    app = SimulationLauncherGui()
    page = app.urdf_arm_controls
    started_at = time.monotonic()

    def update():
        app.update()
        if not args.pick_place or not page.pick_state:
            return
        stage = {key: page.pick_state.get(key) for key in ('phase', 'stage', 'drive_phase')}
        if report['task_stages'] and all(
                report['task_stages'][-1][key] == value for key, value in stage.items()):
            return
        stage.update(wall_seconds=round(time.monotonic() - started_at, 2),
                     message=page.pick_state.get('message'),
                     last_completed=page.pick_state.get('last_completed'))
        report['task_stages'].append(stage)
        with (args.output / 'task-stages.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(stage) + '\n')

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            update()
            time.sleep(.02)

    def until(predicate, seconds, label, *, require_launch=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            update()
            if predicate():
                return
            if require_launch and app.process is not None and app.process.poll() is not None:
                raise RuntimeError(f'Run exited while waiting for {label} (code {app.process.returncode})')
            time.sleep(.05)
        raise TimeoutError(label)

    def measured():
        return {name: channel['measured'] for name, channel in page.channels().items()}

    def press(button):
        # A screenshot blocks the Tk loop for ~1 s: the page treats its feedback
        # as stale and the plant's watchdog holds (heartbeat absent), so each
        # screenshot is followed by a pump and each press by a readiness wait.
        if button is not page.stop_button:
            until(page.ready, 10, 'fresh feedback before ' + button.cget('text'))
        state = str(button.state())
        button.invoke()
        report['steps'].append(dict(pressed=button.cget('text'), button_state=state, page_status=page.status.get()))

    def settle(targets, label, seconds=30.):
        start = time.monotonic()
        sim_start = (page.state or {}).get('grip_contacts', {}).get('sim_time_s')
        try:
            until(lambda: page.ready() and all(abs(measured()[n] - v) < .03 for n, v in targets.items()), seconds, label)
        except TimeoutError:
            state = page.state or {}
            sim_end = state.get('grip_contacts', {}).get('sim_time_s')
            report['settle_timeout'] = dict(step=label, targets=targets, wall_seconds=round(time.monotonic() - start, 2),
                sim_seconds=round(sim_end - sim_start, 4) if sim_start is not None and sim_end is not None else None,
                actuator_status=state.get('status'), fault=state.get('fault'),
                channels={name: {key: channel.get(key) for key in ('target', 'reference', 'measured')}
                          for name, channel in page.channels().items() if name in targets})
            raise
        report['steps'].append(dict(step=label, targets=targets, settled_s=round(time.monotonic() - start, 2),
                                    measured={n: round(v, 4) for n, v in measured().items()}))

    try:
        app.robot_var.set(args.robot)
        app.simulator_var.set(args.simulator)
        app.mode_var.set(args.mode)
        app.map_var.set(args.map)
        app.gui_var.set('false')
        app.experimental_assets_var.set(args.experimental)
        app._update_from_selection()
        app.update()
        report['page_selected'] = app.control_notebook.select() == str(page)
        report['rows'] = [child.cget('text') for child in page.joint_frame.winfo_children()]
        app.show_tab('Mobile arm')             # the Launch workspace, this page raised
        report['command'] = app.command_var.get()
        requested = dict(robot=args.robot, simulator=args.simulator, map=args.map, mode=args.mode)
        resolved = dict(robot=app.robot_var.get(), simulator=app.simulator_var.get(),
                        map=app.map_var.get(), mode=app.mode_var.get())
        if resolved != requested:
            raise RuntimeError(f'GUI selected {resolved}, requested {requested}')
        profiles = dict(command=report['command'], robot=app._robot_config(), world=app._map_config())
        profile_bytes = (json.dumps(profiles, sort_keys=True, indent=2) + '\n').encode('utf-8')
        (args.output / 'launch-profile.json').write_bytes(profile_bytes)
        report['launch_profile_sha256'] = sha256(profile_bytes).hexdigest()
        app._start_launch()
        until(page.ready, 300, 'arm page ready')
        report['model_sha256'] = page.state.get('model_sha256')
        pump(3.)
        screenshot(args.output / 'ready.png')
        pump(1.)
        names = page.joint_names
        if args.pick_place:
            pick_place(app, page, args, report, press, settle, until, pump, screenshot)
        elif args.flange_offset:
            until(lambda: page.measured_tool() is not None, 30, 'measured source flange')
            page.use_tool_pose()
            start = page.measured_tool()['position']
            target = [round(value + delta, 6) for value, delta in zip(start, args.flange_offset)]
            for variable, value in zip(page.tool_xyz, target):
                variable.set(value)
            page.tool_orientation.set('Position only')
            outcome = report['flange'] = dict(start_m=start, target_m=target, before_joints=measured())
            press(page.plan_pose_button)
            until(lambda: page.plan_matches() or page.planning().get('phase') == 'failed', 20, 'flange plan')
            outcome['plan'] = page.planning()
            if not page.plan_matches():
                raise RuntimeError(outcome['plan'].get('message'))
            press(page.execute_button)
            until(lambda: page.planning().get('phase') in {'succeeded', 'failed'}, 95, 'flange execution')
            outcome['execution'] = page.planning()
            outcome['measured_m'] = page.measured_tool()['position']
            outcome['after_joints'] = measured()
            outcome['motion_m'] = math.dist(start, outcome['measured_m'])
            outcome['error_m'] = math.dist(target, outcome['measured_m'])
            report['passed'] = bool(report['page_selected'] and outcome['execution']['phase'] == 'succeeded'
                                    and outcome['error_m'] <= .006
                                    and outcome['motion_m'] >= .5 * math.dist(start, target) > 0.)
            screenshot(args.output / 'flange.png')
        elif args.gripper_presets:
            config = page.arm()
            leader, = config['grip_presets']
            followers = [name for name, spec in config.get('couplings', {}).items() if spec['joint'] == leader]
            results = {}
            for label, button in zip(('open', 'close'), page.grip_buttons, strict=True):
                target = float(config['grip_presets'][leader][label])
                press(button)
                settle({leader: target, **{name: target for name in followers}}, 'gripper '+label)
                pump(1.)
                feedback = measured()
                results[label] = dict(target=target, leader=feedback[leader],
                    followers={name: feedback[name] for name in followers},
                    max_skew_rad=max((abs(feedback[name]-feedback[leader]) for name in followers), default=0.))
                screenshot(args.output/(label+'.png'))
                pump(.6)
            if set(results) != {'open', 'close'}:
                raise RuntimeError('Expected the two declared gripper presets')
            report['gripper'] = results
            report['passed'] = bool(report['page_selected'] and all(
                abs(row['leader']-row['target']) <= .03 and row['max_skew_rad'] <= .01 for row in results.values()))
            if leader in page.grip_openings:
                from robot_lab_utils.urdf_articulation import grip_aperture_interpolate
                opening, feedback, aperture, button = page.grip_openings[leader]
                requested_mm = round(aperture[-1][1]*500.)
                opening.set(requested_mm)
                target = grip_aperture_interpolate(requested_mm/1000., aperture, by_gap=True)
                press(button)
                settle({leader: target}, 'calibrated opening')
                pump(1.)
                inferred = grip_aperture_interpolate(measured()[leader], aperture)*1000.
                report['calibrated_opening'] = dict(command_mm=requested_mm, estimated_mm=inferred,
                    feedback=feedback.get(), shaft_target_rad=target, measured_shaft_rad=measured()[leader])
                report['passed'] = report['passed'] and abs(inferred-requested_mm) <= 1.
                screenshot(args.output/'opening.png')
        else:
            home = {n: round(v, 4) for n, v in measured().items()}
            report['home_measured'] = home
            page.target_vars[names[0]].set(.5)
            press(page.apply_buttons[names[0]])
            settle({names[0]: .5}, 'apply J1 0.5 rad')
            page.step.set(.2)
            page.target_vars[names[1]].set(round(measured()[names[1]], 4))
            press(page.jog_buttons[names[1], 1])
            settle({names[1]: home[names[1]] + .2}, 'jog J2 +0.2 rad')
            screenshot(args.output / 'moved.png')
            pump(1.)                               # heartbeat resumes before the next command
            press(page.home_button)
            settle({n: 0. for n in names}, 'home')
            # Stop part-way through a long J1 move: the measured joint must hold.
            page.target_vars[names[0]].set(2.)
            press(page.apply_buttons[names[0]])
            pump(.8)
            press(page.stop_button)
            pump(.5)
            until(page.ready, 10, 'feedback after stop')
            held = measured()[names[0]]
            pump(2.)
            drift = abs(measured()[names[0]] - held)
            stop_ok = drift < .02 and abs(held - 2.) > .3
            report['stop'] = dict(held_at=round(held, 4), target_was=2., drift_after_rad=round(drift, 4), passed=stop_ok)
            press(page.home_button)
            settle({n: 0. for n in names}, 'home after stop', 40.)
            report['passed'] = bool(report['page_selected'] and len(report['rows']) == len(names) and stop_ok)
    except BaseException as exc:
        report['passed'] = False
        report['error'] = repr(exc)
        report['page_state'] = dict(status=page.status.get(), feedback=page.feedback.get(), owned=page.owned(),
                                    visible=page.visible(), selected=page.selected())
        if args.pick_place:
            report['pick_state'] = page.pick_state
        try:
            screenshot(args.output / 'failure.png')
        except Exception:
            pass
    finally:
        if app.process and app.process.poll() is None:
            app._stop_launch()
            try:
                until(lambda: not app._launch_running, 90, 'shutdown', require_launch=False)
            except TimeoutError:
                pass
        report['launch_returncode'] = app.process.poll() if app.process else None
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        (args.output / 'gui-output.log').write_text(app.output.get('1.0', 'end'))
        app._on_close()
    print(json.dumps({k: report.get(k) for k in ('passed', 'page_selected', 'rows', 'stop', 'error')}, indent=1))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
