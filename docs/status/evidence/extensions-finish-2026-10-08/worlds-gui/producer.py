#!/usr/bin/env python3
"""Bounded actual GUI Generate/Stop actions; artifacts remain on the SSD."""
import hashlib
import json
from pathlib import Path
import time
import subprocess

from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_gui.lab_tabs import WorldsTab, AssetsTab

RUN = Path(__file__).resolve().parent


def descendants(pid):
    parents = {}
    output = subprocess.check_output(['ps', '-eo', 'pid=,ppid=,args='], text=True)
    for line in output.splitlines():
        fields = line.split(None, 2)
        if len(fields) >= 2:
            parents[int(fields[0])] = int(fields[1])
    result = set()
    pending = [pid]
    while pending:
        parent = pending.pop()
        for child, ppid in parents.items():
            if ppid == parent and child not in result:
                result.add(child)
                pending.append(child)
    return result


def active(pid):
    try:
        status = (Path('/proc') / str(pid) / 'stat').read_text().split(') ', 1)[1].split()[0]
        return status != 'Z'
    except OSError:
        return False


def widgets(parent):
    yield parent
    for child in parent.winfo_children():
        yield from widgets(child)


def wait(app, condition, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        value = condition()
        if value:
            return value
        app.update()
        time.sleep(.05)
    raise RuntimeError('Bounded GUI condition did not complete')


app = SimulationLauncherGui()
app.withdraw()
app.update()
report = {'checks': {}, 'source_sha256': {}}
owned = set()
try:
    tree = list(widgets(app))
    world = next(w for w in tree if isinstance(w, WorldsTab))
    assets = next(w for w in tree if isinstance(w, AssetsTab))
    report['catalog_rows'] = sum(len(assets.tree.get_children(p)) for p in assets.tree.get_children())
    report['checks']['actual_world_and_asset_tabs'] = report['catalog_rows'] > 0
    launch_before = (app.robot_var.get(), app.map_var.get(), app.mode_var.get(), app.command_var.get())
    world.world_var.set('nav_obstacle')
    default_seed = app.map_profiles['nav_obstacle']['spawn']
    assert world.seed_x_var.get() == float(default_seed['x'])
    assert world.seed_y_var.get() == float(default_seed['y'])
    world.seed_x_var.set(-6.75)
    world.seed_y_var.set(-6.75)
    report['custom_seed'] = [-6.75, -6.75]
    world.resolution_var.set(.015)
    world.output_var.set(str(RUN/'gui-world-cancel-ps-child'))
    report['cancel_command'] = world.generation_command()
    world.generate_button.invoke()
    process = app.bg_processes['world_maps']
    owned.add(process.pid)

    def gazebo_started():
        owned.update(descendants(process.pid))
        commands = {}
        for pid in owned:
            try:
                commands[pid] = (Path('/proc')/str(pid)/'cmdline').read_bytes().replace(b'\0', b' ').decode()
            except OSError as exc:
                report.setdefault('observer_errors', []).append(str(exc))
        report.setdefault('observed_processes', {}).update(commands)
        return commands if any(pid != process.pid and 'gazebo' in cmd and ('--child' in cmd or 'generate_occupancy' not in cmd) for pid, cmd in commands.items()) else None

    report['cancel_processes'] = wait(app, gazebo_started, 40)
    stop = next(w for w in widgets(world) if w.winfo_class() == 'TButton' and w.cget('text') == 'Stop Generation')
    stop.invoke()
    wait(app, lambda: not any(active(pid) for pid in owned), 12)
    report['checks']['stop_ends_owned_processes'] = not any(active(pid) for pid in owned)
    report['cancel_cli_exit_code'] = process.poll()
    report['cancel_completed_reports'] = len(list((RUN/'gui-world-cancel-ps-child').rglob('*.generation.json')))
    report['checks']['generation_was_still_running'] = report['cancel_completed_reports'] == 0

    world.resolution_var.set(.1)
    world.output_var.set(str(RUN/'gui-world-complete-ps-child'))
    report['generate_command'] = world.generation_command()
    assert report['generate_command'][report['generate_command'].index('--seed-x')+1] == '-6.75'
    assert report['generate_command'][report['generate_command'].index('--seed-y')+1] == '-6.75'
    report['checks']['custom_seed_forwarded'] = True
    world.generate_button.invoke()
    process = app.bg_processes['world_maps']
    wait(app, lambda: process.poll() is not None, 90)
    reports = list((RUN/'gui-world-complete-ps-child').rglob('*.generation.json'))
    report['checks']['completed_actual_export'] = process.returncode == 0 and len(reports) == 1
    if len(reports) != 1:
        raise RuntimeError('GUI generation did not produce exactly one measured report')
    artifact = json.loads(reports[0].read_text())
    report['generation_report'] = str(reports[0])
    report['generated_cells'] = {k: artifact[k] for k in ('free_cells', 'occupied_cells', 'unknown_cells')}
    report['checks']['actual_obstacles_and_free_region'] = artifact['free_cells'] > 10 and artifact['occupied_cells'] > 0
    report['generation_report_sha256'] = hashlib.sha256(reports[0].read_bytes()).hexdigest()
    assert launch_before == (app.robot_var.get(), app.map_var.get(), app.mode_var.get(), app.command_var.get())
    report['checks']['launch_selection_unchanged'] = True
    for p in ['src/robot_lab_gui/robot_lab_gui/launcher.py', 'src/robot_lab_gui/robot_lab_gui/lab_tabs.py',
              'src/robot_lab_maps/tools/generate_occupancy_map.py']:
        report['source_sha256'][p] = hashlib.sha256(Path(p).read_bytes()).hexdigest()
except Exception as exc:
    report['error'] = str(exc)
finally:
    report['console'] = app.output.get('1.0', 'end-1c')
    report['passed'] = not report.get('error') and bool(report['checks']) and all(report['checks'].values())
    (RUN/'gui-worlds-check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    app._on_close()
raise SystemExit(0 if report['passed'] else 1)
