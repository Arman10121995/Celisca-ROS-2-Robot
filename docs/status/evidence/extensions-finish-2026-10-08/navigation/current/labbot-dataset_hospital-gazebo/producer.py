#!/usr/bin/env python3
"""Run one real, owned normal-GUI Nav2 screen on an installed world.

Source ROS/SSD environments first. Run serially under Xvfb with an isolated
ROS_DOMAIN_ID. A physical body endpoint is required; catalog counts and Nav2's
success status alone do not pass. Large trace/log output stays on the SSD.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=('gazebo','mujoco','pybullet','isaac'),required=True)
    parser.add_argument('--robot',default='labbot')
    parser.add_argument('--map',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=360)
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.parent.stat().st_dev==Path('/').stat().st_dev:
        parser.error('Trial outputs must be on the workspace SSD')
    args.output.mkdir(exist_ok=False)
    out=args.output
    root=Path(__file__).resolve().parents[1]
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())
    from robot_lab_gui.launcher import SimulationLauncherGui
    app=SimulationLauncherGui();app.update()
    check=None
    report=dict(robot=args.robot,backend=args.backend,map=args.map,mode='nav',passed=False,
        scope='Normal Tk selection/autofill/Run/Stop; actual RViz-style topic goal; independent simulator body endpoint. Short static route, no complete mesh clearance or universal-map qualification.')
    last_log=0.
    def pump():
        nonlocal last_log
        app.update();time.sleep(.01)
        if time.monotonic()-last_log>5:
            (out/'live-gui-output.log').write_text(app.output.get('1.0','end'))
            last_log=time.monotonic()
    try:
        app.robot_var.set(args.robot);app.simulator_var.set(args.backend)
        app.map_var.set(args.map);app.mode_var.set('nav');app.gui_var.set('false')
        app._update_from_selection();app.update()
        assert app.mode_var.get()=='nav',app.validation_var.get()
        assert app.start_button.instate(['!disabled']),app.validation_var.get()
        assert f'robot_model:={args.robot}' in app.command_var.get()
        assert f'map_name:={args.map}' in app.command_var.get()
        profile=app.map_profiles[args.map]
        from ament_index_python.packages import get_package_share_directory
        world=Path(profile['gazebo']['world_path'])
        if not world.is_absolute():world=Path(get_package_share_directory(profile['gazebo']['world_package']))/world
        grid=Path(profile['map']['path'])
        if not grid.is_absolute():grid=Path(get_package_share_directory(profile['map']['package']))/grid
        import yaml
        image=grid.parent/yaml.safe_load(grid.read_text())['image']
        files=[world,grid,image,root/'src/robot_lab_navigation/config/robots'/(args.robot+'.yaml'),
               root/'src/robot_lab_gui/robot_lab_gui/lab_tabs.py']
        packages=('robot_lab_utils','robot_lab_gui','robot_lab_bringup','robot_lab_description',
                  'robot_lab_mujoco','robot_lab_pybullet','robot_lab_isaac','robot_lab_navigation',
                  'robot_lab_localization','robot_lab_controller','robot_lab_robots')
        files += [p for package in packages for p in (root/'src'/package).rglob('*')
                  if p.is_file() and p.suffix in ('.py','.yaml','.xml','.xacro','.cpp')]
        robot_profile=app.robot_profiles[args.robot]
        if Path(robot_profile.get('xacro','')).is_file():files.append(Path(robot_profile['xacro']))
        report['source']=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
            hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},
            profile=profile,robot_profile=robot_profile,domain=os.environ.get('ROS_DOMAIN_ID'),
            producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        report['command']=app.command_var.get()
        (out/'pre-trial.json').write_text(json.dumps(report,indent=2)+'\n')
        app.start_button.invoke();report['owned_pid']=app.process.pid
        command=['python3',str(root/'scripts/sim_nav_check.py'),'--timeout',str(args.timeout),
            '--via-topic','--max-position-error','.15','--max-yaw-error-deg','5',
            '--goal-yaw-deg','0','--trace-out',str(out/'body.trace.json')]
        report['check_command']=command
        with (out/'check.log').open('w') as stream:
            check=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,cwd=root)
            deadline=time.monotonic()+args.timeout*3+90
            while check.poll() is None and time.monotonic()<deadline:
                pump();assert app.process.poll() is None,'Owned launch exited before navigation'
            assert check.poll() is not None,'Navigation probe exceeded its budget'
        report['check_returncode']=check.returncode
        lines=(out/'check.log').read_text().splitlines()
        report['navigation']=json.loads(next(v for v in reversed(lines) if v.startswith('{')))
        samples=json.loads((out/'body.trace.json').read_text())['samples']
        assert len(samples)>10,'Navigation has no independent physical body trace'
        assert all(math.isfinite(s[k]) for s in samples for k in ('x','y','z','yaw'))
        expected_z=float(profile.get('spawn',{}).get('z',0))
        report['body_height']=dict(expected_floor_m=expected_z,
            minimum_m=min(s['z'] for s in samples),maximum_m=max(s['z'] for s in samples),samples=len(samples))
        assert (report['body_height']['minimum_m']>=expected_z-.25
                and report['body_height']['maximum_m']<=expected_z+.75),report['body_height']
        assert check.returncode==0,report['navigation']
        app._stop_launch();deadline=time.monotonic()+60
        while app._launch_running and time.monotonic()<deadline:pump()
        report['launch_returncode']=app.process.poll()
        assert report['launch_returncode']==0,report['launch_returncode']
        report['passed']=True
    except BaseException as exc:
        report['exception']=repr(exc)
        raise
    finally:
        if check and check.poll() is None:
            check.terminate()
            try:check.wait(timeout=5)
            except subprocess.TimeoutExpired:check.kill();check.wait()
        if app.process and app.process.poll() is None:
            app._stop_drive();app._stop_launch();deadline=time.monotonic()+60
            while app._launch_running and time.monotonic()<deadline:pump()
        report['launch_returncode']=app.process.poll() if app.process else None
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        (out/'gui-output.log').write_text(app.output.get('1.0','end'))
        if getattr(app,'ros_executor',None):
            app.ros_executor.shutdown(timeout_sec=2.)
        if app.arm_tab:app.arm_tab.close()
        if app.hand_tab:app.hand_tab.close()
        app.destroy()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
