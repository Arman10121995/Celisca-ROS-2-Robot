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
import json
from pathlib import Path
import struct
import subprocess
import time

from robot_lab_gui.launcher import SimulationLauncherGui


def screenshot(path):
    from PIL import Image
    data = subprocess.run(['xwd', '-root', '-silent'], check=True, capture_output=True).stdout
    fields = struct.unpack('>25I', data[:100])
    header, width, height = fields[0], fields[4], fields[5]
    byte_order, bytes_per_line, ncolors = fields[7], fields[12], fields[19]
    raw = data[header + ncolors * 12:header + ncolors * 12 + bytes_per_line * height]
    Image.frombuffer('RGB', (width, height), raw, 'raw', 'BGRX' if byte_order == 0 else 'XRGB',
                     bytes_per_line, 1).save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='asset_elephant_myagv_pro_pro630')
    parser.add_argument('--simulator', default='pybullet')
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(robot=args.robot, simulator=args.simulator, map=args.map, steps=[], passed=False)
    app = SimulationLauncherGui()
    page = app.urdf_arm_controls

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.update()
            time.sleep(.02)

    def until(predicate, seconds, label):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.update()
            if predicate():
                return
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
        until(lambda: page.ready() and all(abs(measured()[n] - v) < .03 for n, v in targets.items()), seconds, label)
        report['steps'].append(dict(step=label, targets=targets, settled_s=round(time.monotonic() - start, 2),
                                    measured={n: round(v, 4) for n, v in measured().items()}))

    try:
        app.robot_var.set(args.robot)
        app.simulator_var.set(args.simulator)
        app.mode_var.set('display')
        app.map_var.set(args.map)
        app.gui_var.set('false')
        app._update_from_selection()
        app.update()
        report['page_selected'] = app.control_notebook.select() == str(page)
        report['rows'] = [child.cget('text') for child in page.joint_frame.winfo_children()]
        app.show_tab('Mobile arm')             # the Launch workspace, this page raised
        report['command'] = app.command_var.get()
        app._start_launch()
        until(page.ready, 300, 'arm page ready')
        pump(3.)
        screenshot(args.output / 'ready.png')
        pump(1.)
        names = page.joint_names
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
        report['error'] = repr(exc)
        report['page_state'] = dict(status=page.status.get(), feedback=page.feedback.get(), owned=page.owned(),
                                    visible=page.visible(), selected=page.selected())
        try:
            screenshot(args.output / 'failure.png')
        except Exception:
            pass
    finally:
        if app.process and app.process.poll() is None:
            app._stop_launch()
            try:
                until(lambda: not app._launch_running, 90, 'shutdown')
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
