#!/usr/bin/env python3
"""Record one real GUI localization/mapping/navigation workflow.

Use a sourced ROS environment, Xvfb, isolated domain and SSD output. Run
serially. An experimental provider must be declared explicitly; it proves
the candidate path without granting modes in the production asset store.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', required=True)
    parser.add_argument('--backend', choices=('gazebo', 'mujoco', 'pybullet', 'isaac'), required=True)
    parser.add_argument('--mode', choices=('loc', 'slam', '3d_slam', 'nav'), required=True)
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--radius', type=float, default=.62)
    parser.add_argument('--experimental-provider', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--budget', type=float, default=900.)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.parent.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Physical trial output must stay on the workspace SSD')
    provider = os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '')
    if args.experimental_provider and Path(provider).resolve() != args.experimental_provider.resolve():
        parser.error('ROBOT_LAB_RUNTIME_ROOT must match the declared experimental provider')
    args.output.mkdir(exist_ok=False)
    out, root = args.output, Path(__file__).resolve().parents[1]
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())
    from robot_lab_gui.launcher import SimulationLauncherGui
    import robot_lab_gui.launcher as gui_module
    app = SimulationLauncherGui()
    app.update()
    check, last_log = None, 0.
    report = dict(robot=args.robot, backend=args.backend, mode=args.mode, map=args.map, passed=False,
        provider=provider, experimental_provider=bool(args.experimental_provider),
        scope='Actual Tk selection/autofill/Run/Save/Stop and independent body truth. '
              'Named static route/map only; no universal-map, hardware or vendor firmware grant.')

    def pump():
        nonlocal last_log
        app.update()
        time.sleep(.02)
        if time.monotonic()-last_log > 5:
            (out/'live-gui-output.log').write_text(app.output.get('1.0', 'end'))
            last_log = time.monotonic()

    try:
        app.robot_var.set(args.robot)
        app.simulator_var.set(args.backend)
        app.map_var.set(args.map)
        app.mode_var.set(args.mode)
        app.gui_var.set('false')
        app._update_from_selection()
        app.update()
        assert app.mode_var.get() == args.mode, app.validation_var.get()
        assert app.start_button.instate(['!disabled']), app.validation_var.get()
        report['command'] = app.command_var.get()
        report['algorithm_selection'] = {k:v.get() for k,v in app.slot_vars.items()}
        profile = app.robot_profiles[args.robot]
        model = Path(profile['xacro'])
        (out/'executed.urdf').write_bytes(model.read_bytes())
        packages = ('robot_lab_utils', 'robot_lab_gui', 'robot_lab_bringup', 'robot_lab_description',
                    'robot_lab_mujoco', 'robot_lab_pybullet', 'robot_lab_isaac', 'robot_lab_navigation',
                    'robot_lab_mapping', 'robot_lab_localization', 'robot_lab_controller')
        hashes = {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for name in packages for p in (root/'src'/name).rglob('*')
                  if p.is_file() and p.suffix in ('.py', '.yaml', '.xml', '.xacro', '.cpp')}
        extra = root/'scripts/extension_husky_control.py'
        if extra.is_file(): hashes[str(extra.relative_to(root))] = hashlib.sha256(extra.read_bytes()).hexdigest()
        # Bind the world and occupancy bytes used by this trial, not just the
        # provider's paths. External assets may live outside the repository.
        from ament_index_python.packages import get_package_share_directory
        import yaml
        map_profile = app.map_profiles[args.map]
        world = Path(map_profile['gazebo']['world_path'])
        if not world.is_absolute():
            world = Path(get_package_share_directory(map_profile['gazebo']['world_package']))/world
        grid = Path(map_profile['map']['path'])
        if not grid.is_absolute():
            grid = Path(get_package_share_directory(map_profile['map']['package']))/grid
        image = grid.parent/yaml.safe_load(grid.read_text())['image']
        for asset in (world, grid, image):
            key = str(asset.relative_to(root)) if asset.is_relative_to(root) else str(asset)
            hashes[key] = hashlib.sha256(asset.read_bytes()).hexdigest()
        (out/'source-manifest.json').write_text(json.dumps(dict(
            revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
            source_sha256=hashes, profile=profile, map_profile=app.map_profiles[args.map],
            producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            executed_urdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest(), provider=provider,
            experimental_provider=bool(args.experimental_provider)), indent=2)+'\n')
        app.start_button.invoke()
        report['owned_pid'] = app.process.pid
        if args.mode == 'nav':
            command = ['python3', str(root/'scripts/sim_nav_check.py'), '--base', args.base_frame,
                '--timeout', '600', '--via-topic', '--max-position-error', '.15', '--max-yaw-error-deg', '5',
                '--trace-out', str(out/'navigation.trace.json'), '--robot-radius', str(args.radius),
                '--min-route-clearance', '.02', '--world-file',
                str(root/'src/robot_lab_maps/maps'/args.map/'worlds'/(args.map+'.world'))]
            if args.map == 'nav_obstacle':
                command += ['--offset-x', '3.5', '--offset-y', '3.5', '--goal-yaw-deg', '0', '--require-detour']
            else: command += ['--offset-x', '2', '--goal-yaw-deg', '0']
        else:
            command = ['python3', str(root/'scripts/sim_workflow_check.py'), '--mode', args.mode,
                '--base-frame', args.base_frame, '--timeout', '300', '--out', str(out/'workflow.json')]
        report['check_command'] = command
        with (out/'check.log').open('w') as log:
            check = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, cwd=root)
            deadline = time.monotonic()+args.budget
            while check.poll() is None and time.monotonic() < deadline:
                pump()
                assert app.process.poll() is None, 'Owned launch exited during workflow'
            assert check.poll() is not None, 'Physical workflow exceeded the declared budget'
        report['check_returncode'] = check.returncode
        if args.mode == 'nav':
            lines = (out/'check.log').read_text().splitlines()
            report['workflow'] = json.loads(next(v for v in reversed(lines) if v.startswith('{')))
        else: report['workflow'] = json.loads((out/'workflow.json').read_text())
        assert check.returncode == 0, report['workflow']
        if args.mode in ('slam', '3d_slam'):
            target = out/('saved-map.db' if args.mode == '3d_slam' else 'saved-map')
            gui_module.filedialog.asksaveasfilename = lambda **kwargs: str(target)
            assert app.save_map_button.instate(['!disabled'])
            app._save_map()
            outputs = [target, target.with_suffix('.pcd')] if args.mode == '3d_slam' else [target.with_suffix('.yaml'), target.with_suffix('.pgm')]
            deadline = time.monotonic()+90
            while not all(p.is_file() and p.stat().st_size > 0 for p in outputs) and time.monotonic() < deadline: pump()
            assert all(p.is_file() and p.stat().st_size > 0 for p in outputs), 'GUI map export did not complete'
            if args.mode == '3d_slam':
                import io
                import sqlite3
                import numpy as np
                with sqlite3.connect(target) as db:
                    assert db.execute('pragma integrity_check').fetchone()[0] == 'ok'
                    nodes = db.execute('select count(*) from Node').fetchone()[0]
                cloud = outputs[1].read_text()
                assert 'DATA ascii\n' in cloud
                points = np.loadtxt(io.StringIO(cloud.split('DATA ascii\n', 1)[1]))
                assert nodes > 1 and len(points) > 100 and np.isfinite(points[:, :3]).all()
                report['save_map'] = dict(database_nodes=nodes, pointcloud_points=len(points))
            else: report['save_map'] = dict(yaml=outputs[0].read_text(), image_bytes=outputs[1].stat().st_size)
            deadline = time.monotonic()+10
            while app._aux_processes and time.monotonic() < deadline: pump()
            assert not app._aux_processes, 'Map exporter did not exit after saving'
            report['map_exporter_exited'] = True
        app._stop_drive()
        app._stop_launch()
        deadline = time.monotonic()+60
        while app._launch_running and time.monotonic() < deadline: pump()
        report['launch_returncode'] = app.process.poll()
        assert report['launch_returncode'] == 0
        report['passed'] = True
    except BaseException as exc:
        report['exception'] = repr(exc)
        raise
    finally:
        if check and check.poll() is None:
            check.terminate()
            try: check.wait(timeout=5)
            except subprocess.TimeoutExpired: check.kill(); check.wait()
        if app.process and app.process.poll() is None:
            app._stop_drive()
            app._stop_launch()
            deadline = time.monotonic()+60
            while app._launch_running and time.monotonic() < deadline: pump()
        report['launch_returncode'] = app.process.poll() if app.process else None
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'gui-output.log').write_text(app.output.get('1.0', 'end'))
        app._on_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
