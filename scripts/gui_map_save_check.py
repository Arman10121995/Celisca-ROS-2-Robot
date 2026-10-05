#!/usr/bin/env python3
"""Exercise the installed GUI Save Map button against an active RTAB-Map node.

Set ROBOT_LAB_RTABMAP_DIR to the active database directory and use the GUI's
<map>_<robot>.db naming. Requires DISPLAY (or xvfb-run). Only the file picker
is automated; ROS backup, SQLite copy and PointCloud2 export execute normally.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import time
from unittest.mock import patch

import numpy as np
from robot_lab_gui.launcher import SimulationLauncherGui


def read_pcd(path):
    lines=path.read_text().splitlines()
    start=lines.index('DATA ascii')+1
    count=int(next(line.split()[1] for line in lines if line.startswith('POINTS ')))
    points=np.asarray([[float(v) for v in line.split()[:3]] for line in lines[start:]])
    if points.shape!=(count,3) or count<100 or not np.isfinite(points).all():
        raise ValueError('Incomplete or invalid exported point cloud')
    return {'points':count,'min_xyz':points.min(axis=0).tolist(),'max_xyz':points.max(axis=0).tolist()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',default='four_wheel_steer_car')
    parser.add_argument('--map-name',default='nav_empty')
    parser.add_argument('--simulator',default='mujoco')
    parser.add_argument('--steering-mode',default='crab')
    parser.add_argument('--target',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    target=args.target.resolve(); cloud=target.with_suffix('.pcd')
    if target.exists() or cloud.exists():parser.error('Choose fresh output paths')
    app=SimulationLauncherGui();report={'checks':{}}
    try:
        app.robot_var.set(args.robot);app.map_var.set(args.map_name)
        app.simulator_var.set(args.simulator);app.mode_var.set('3d_slam')
        app.steering_mode_var.set(args.steering_mode)
        app._update_from_selection()
        source=Path(app._rtabmap_database_path())
        report.update(robot_id=args.robot,map_name=args.map_name,simulator=args.simulator,
                      active_database=str(source),output_database=str(target),command=app.command_var.get())
        report['checks']['active_database_exists']=source.is_file()
        if not source.is_file():raise RuntimeError('Active GUI database not found: '+str(source))
        with patch('robot_lab_gui.launcher.filedialog.asksaveasfilename',return_value=str(target)):
            app.save_map_button.invoke()
        deadline=time.monotonic()+70
        geometry=None
        while time.monotonic()<deadline:
            app.update()
            if cloud.is_file():
                try:geometry=read_pcd(cloud)
                except (ValueError,StopIteration):pass
                if geometry:break
            time.sleep(.05)
        if geometry is None:raise RuntimeError('GUI did not export a complete PCD within 70 seconds')
        with sqlite3.connect(target.as_uri()+'?mode=ro',uri=True) as db:
            integrity=db.execute('PRAGMA integrity_check').fetchone()[0]
            nodes=db.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
        report.update(point_cloud=geometry,sqlite_nodes=nodes,sqlite_integrity=integrity,
            artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (target,cloud)})
        report['checks'].update(sqlite_integrity=integrity=='ok',mapped_keyframes=nodes>0,
            real_3d_geometry=geometry['max_xyz'][2]-geometry['min_xyz'][2]>.3,
            source_preserved=source.is_file() and source.resolve()!=target)
    except Exception as exc:
        report['error']=str(exc)
    finally:
        report['gui_console']=app.output.get('1.0','end-1c')
        report['passed']=not report.get('error') and bool(report['checks']) and all(report['checks'].values())
        args.out.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        app._on_close()
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
