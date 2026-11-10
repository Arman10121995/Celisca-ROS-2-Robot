#!/usr/bin/env python3
"""Live robot-vacuum session driven through the real Tk Vacuum tab.

Run serially on the SSD with a sourced ROS environment and a display.  The
Setup page launches the robot and the cleaning system; two jobs then run
through the operator controls:
  1. a room picked by clicking the live room map (Rooms tool), paused and
     resumed mid-job; the pause must hold the robot still by simulator truth;
  2. a zone drawn with the Zone tool, with a no-go box and a virtual wall
     drawn with the No-go and Wall tools.
Each job is judged like a recorded job (cleaning_screen): the History record
the GUI shows, plus an independent truth trail from /odom/ground_truth.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from robot_lab_gui.launcher import SimulationLauncherGui

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from r9_3_support_matrix import cleaning_screen  # noqa: E402

OFFICE = (7.5, 2.0)                                   # a point inside the office
ZONE = ((4.0, -3.5), (7.5, -0.5))
NO_GO = ((5.25, -2.5), (6.25, -1.5))
WALL = ((7.0, -3.6), (7.0, -2.0))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def screenshot(path):
    """Grab the X display (xwd) and save it as PNG."""
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
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map', default='small_house')
    parser.add_argument('--simulator', default='gazebo')
    parser.add_argument('--job-budget', type=float, default=1500.0, help='seconds per job')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.parent.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Run logs must be on the mounted workspace SSD')
    args.output.mkdir(exist_ok=False)
    out = args.output
    (out / 'producer.py').write_bytes(Path(__file__).read_bytes())
    os.environ['ROBOT_LAB_RUNTIME_ROOT'] = str(out / 'store')     # a fresh cleaning store
    app = SimulationLauncherGui(); app.update()
    tab = app.vacuum_tab
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('vacuum_gui_observer')
    executor = SingleThreadedExecutor(); executor.add_node(node)
    truth = []
    node.create_subscription(Odometry, '/odom/ground_truth', lambda m: truth.append(
        (time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))),
        qos_profile_sensor_data)
    report = dict(robot='oomwoo', backend=args.simulator, map=args.map, mode='nav', passed=False, jobs=[],
                  scope='Normal Tk Vacuum tab: Setup launch, room picked on the live room map, Pause/Resume, '
                        'Zone/No-go/Wall map tools, Start, History; simulator truth is independent of the node.')
    trail, last_log, last_trail = [], 0., 0.

    def state():
        return (tab.status or {}).get('state', '')

    def pump(plant=True):
        nonlocal last_log, last_trail
        app.update()
        for _ in range(20):
            executor.spin_once(timeout_sec=0.)
        now = time.monotonic()
        if truth and now - last_trail >= 1.0:
            last_trail = now
            pose = (tab.status or {}).get('pose')
            trail.append(dict(t=round(now, 2), state=state(), truth=[round(v, 3) for v in truth[-1][1:]],
                              pose=pose))
        if now - last_log > 5:
            (out / 'live-gui-output.log').write_text(app.output.get('1.0', 'end'))
            last_log = now
        if plant:
            assert app.process and app.process.poll() is None, 'Owned vacuum launch exited'
        time.sleep(.01)

    def until(predicate, budget, what, plant=True):
        deadline = time.monotonic() + budget
        while not predicate() and time.monotonic() < deadline:
            pump(plant)
        assert predicate(), ('Timeout: ' + what, tab.status)

    def interval(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            pump()
        assert truth and time.monotonic() - truth[-1][0] < 1., 'Stale simulator truth'

    def run_job(name, prepare, pause=False):
        """Prepare the selection, press Start, optionally Pause/Resume, wait for the dock."""
        before = len(tab.history)
        prepare()
        app.update()
        request = tab.start_request()
        first = len(trail)
        tab.start_button.invoke()
        until(lambda: state() in ('cleaning',), 60, name + ' start')
        job = dict(name=name, request=request)
        if pause:
            interval(40.)
            assert state() == 'cleaning', ('Not cleaning before Pause', tab.status)
            tab.pause_button.invoke()
            until(lambda: state() == 'paused', 10, 'pause')
            interval(1.5)
            held = truth[-1]
            interval(5.)
            drift = math.hypot(truth[-1][1] - held[1], truth[-1][2] - held[2])
            turn = abs(math.atan2(math.sin(truth[-1][3] - held[3]), math.cos(truth[-1][3] - held[3])))
            screenshot(out / ('%s-paused.png' % name))
            tab.resume_button.invoke()
            until(lambda: state() == 'cleaning', 10, 'resume')
            moved_from = truth[-1]
            interval(8.)
            job['pause'] = dict(drift_m=drift, yaw_drift_rad=turn,
                                moved_after_resume_m=math.hypot(truth[-1][1] - moved_from[1], truth[-1][2] - moved_from[2]))
        interval(20.)
        screenshot(out / ('%s-cleaning.png' % name))
        until(lambda: len(tab.history) > before and state() in ('docked', 'charging'), args.job_budget, name)
        interval(3.)
        screenshot(out / ('%s-docked.png' % name))
        record = dict(tab.history[-1])
        history_rows = [tab.history_tree.item(i, 'values') for i in tab.history_tree.get_children()]
        record.update(final_state=state(), final_truth=list(truth[-1][1:]), final_pose=(tab.status or {}).get('pose'),
                      trail=trail[first:], dock=(tab.layout or {}).get('dock'), robot_model='oomwoo',
                      layout={k: (tab.layout or {}).get(k) for k in ('no_go', 'walls', 'zones')},
                      rooms={str(k): v for k, v in ((tab.layout or {}).get('rooms') or {}).items()},
                      request=request, sensors=(tab.status or {}).get('sensors'))
        job.update(record=record, history_row=list(history_rows[0]) if history_rows else None,
                   screen=cleaning_screen(record))
        if pause:
            job['screen'] = job['screen'] and job['pause']['drift_m'] < .03 and job['pause']['yaw_drift_rad'] < .05 \
                and job['pause']['moved_after_resume_m'] > .05
        (out / ('%s.json' % name)).write_text(json.dumps(record, indent=1, sort_keys=True) + '\n')
        report['jobs'].append({k: v for k, v in job.items() if k != 'record'})
        return job

    def pick_office():
        tab.tool_var.set('select'); tab._tool_changed()
        tab.selected_rooms = []
        tab.apply_tool('select', OFFICE, OFFICE, moved=False)
        tab.edge_var.set(True); tab.passes_var.set(1)
        assert tab.selected_rooms, 'No room under the office point'

    def draw_zone():
        tab.apply_tool('nogo', *NO_GO)
        tab.apply_tool('wall', *WALL)
        until(lambda: (tab.layout.get('no_go') and tab.layout.get('walls')), 20, 'layout echo')
        tab.tool_var.set('zone'); tab._tool_changed()
        tab.apply_tool('zone', *ZONE)
        until(lambda: tab.layout.get('zones'), 20, 'zone echo')
        tab.edge_var.set(False)

    try:
        app.show_tab('Vacuum'); app.update()
        tab.vacuum_robot_var.set('oomwoo'); tab._on_robot_selected()
        tab.vacuum_map_var.set(args.map); tab.vacuum_sim_var.set(args.simulator); tab.vacuum_mode_var.set('nav')
        # The Setup button's command, headless (no Gazebo client or RViz windows).
        command = tab.launch_command() + ['gui:=false', 'start_rviz:=false']
        report['command'] = command
        paths = [p for package in ('robot_lab_gui', 'robot_lab_vacuum_cleaning', 'robot_lab_bringup')
                 for p in (ROOT / 'src' / package).rglob('*.py')]
        paths += [ROOT / 'src/robot_lab_robots/config/robots.yaml', ROOT / 'src/robot_lab_description/launch/gazebo.launch.py']
        manifest = dict(revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True, cwd=ROOT).strip(),
                        domain=os.environ.get('ROS_DOMAIN_ID'),
                        source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
                        producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (out / 'source-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        app.start_launch_command(command); tab._ensure_ros()
        report['owned_pid'] = app.process.pid
        until(lambda: tab.status.get('map_ready') and tab.status.get('localised') and truth, 240, 'ready')
        interval(5.)
        tab._resegment()                       # the Rooms page "Divide map again" button
        until(lambda: tab.rooms_grid is not None and tab.rooms_grid.max() > 0 and tab.layout.get('rooms'), 60, 'rooms')
        report['rooms_divided'] = len(tab.layout.get('rooms') or {})
        interval(10.)
        screenshot(out / 'ready.png')
        report['ready_state'] = state()
        first = run_job('room-office', pick_office, pause=True)
        second = run_job('zone-keepout', draw_zone)
        report['passed'] = bool(first['screen'] and second['screen'])
        app._stop_launch(); until(lambda: not app._launch_running, 90, 'shutdown', False)
        report['launch_returncode'] = app.process.poll()
    except BaseException as exc:
        report['error'] = repr(exc)
        try:
            screenshot(out / 'failure.png')
        except Exception:
            pass
        raise
    finally:
        if app.process and app.process.poll() is None:
            app._stop_launch()
            try:
                until(lambda: not app._launch_running, 90, 'shutdown', False)
            except Exception:
                pass
        report['launch_returncode'] = app.process.poll() if app.process else None
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        (out / 'gui-output.log').write_text(app.output.get('1.0', 'end'))
        executor.shutdown(); node.destroy_node(); app._on_close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
