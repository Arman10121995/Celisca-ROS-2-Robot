#!/usr/bin/env python3
"""Archive actual extension trials and derive trace envelopes without inventing outcomes.

Raw physics logs/traces remain on the SSD. Reports and pre-trial source hashes
are copied unchanged; derived envelopes explicitly identify their raw inputs.
This collector grants no robot mode and does not rewrite historical evidence.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=Path('/workspace/molar/robot_lab_runtime/extensions-finish-2026-10-07'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    dest = root/'docs/status/evidence/extensions-finish-2026-10-08'
    dest.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location('support_matrix', root/'scripts/r9_3_support_matrix.py')
    support = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = support
    spec.loader.exec_module(support)
    index_path = root/'docs/status/runtime-evidence-index.yaml'
    index = yaml.safe_load(index_path.read_text())
    prefix = str(dest.relative_to(root))
    old_records = [r for r in index['records'] if not r['report'].startswith(prefix+'/')]
    records, manifest = [], []

    def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()

    def artifact(path): return dict(path=str(path), sha256=sha(path), bytes=path.stat().st_size)

    def copy_trial(raw, folder):
        folder.mkdir(parents=True, exist_ok=True)
        files = [artifact(p) for p in sorted(raw.iterdir()) if p.is_file()]
        for name in ('report.json', 'source-manifest.json', 'producer.py', 'workflow.json',
                     'navigation.json', 'pre-trial.json'):
            if (raw/name).is_file(): shutil.copy2(raw/name, folder/name)
        manifest.append(dict(raw_directory=str(raw), archive=str(folder.relative_to(root)), artifacts=files))

    def record(report, raw, folder, kind, task, limitations):
        source_name = 'source-manifest.json' if (folder/'source-manifest.json').is_file() else 'pre-trial.json'
        source = json.loads((folder/source_name).read_text())
        report_path = folder/'report-measured.json'
        report_path.write_text(json.dumps(report, indent=2)+'\n')
        source_path = folder/source_name
        records.append(dict(robot_id=report['robot'], environment_id=report['map'], simulator=report['backend'],
            task_type=task, date='2026-10-08',
            revision=source.get('git_head', source.get('revision', source.get('source', {}).get('revision', '')))+
                     ' plus exact pre-trial working-tree source manifest',
            report=str(report_path.relative_to(root)), sha256=sha(report_path),
            source_hashes=str(source_path.relative_to(root)), source_hashes_sha256=sha(source_path),
            report_kind=kind, scope=report.get('scope', '')+' Raw trace/log paths and hashes retained in collection manifest.',
            limitations=limitations))

    # Earlier stages remain visible. In particular, the old gyro-only Isaac
    # report's original passed marker is preserved, but the stronger body-drift
    # gate rejects that run. It precedes the corrected current-stage repeat.
    finals = {'final-gazebo', 'final-mujoco', 'final-pybullet', 'isaac-body-feedback'}
    stages = sorted(p for p in (args.runtime/'husky').iterdir() if p.is_dir() and (p/'report.json').is_file())
    stages.sort(key=lambda p: (p.name in finals, p.name))
    for raw in stages:
        folder = dest/'husky'/raw.name
        copy_trial(raw, folder)
        if raw.name not in finals|{'isaac-yaw-feedback'}: continue
        report = json.loads((raw/'report.json').read_text())
        trace = json.loads((raw/'trace.json').read_text())
        assert trace['body'] and trace['camera_info']
        assert all(math.isfinite(v) for s in trace['body'] for v in s.values())
        report['max_body_tilt_rad'] = max(s['tilt'] for s in trace['body'])
        report['sensors']['camera_info'] = trace['camera_info'][-1]
        report['derived_from'] = dict(producer_report=artifact(raw/'report.json'), physical_trace=artifact(raw/'trace.json'))
        accepted = support.measured_result(report, 'skid_drive_screen')
        if raw.name in finals: assert accepted, raw.name
        else: assert not accepted, 'The excessive-drift Isaac trial must stay negative under the current gate'
        record(report, raw, folder, 'skid_drive_screen', 'teleoperation', [
            'Named nav_empty GUI Drive/WASD screen; physical joystick and other maps untested',
            'Declared lab lidar/RGB-D/IMU kit and motor/solver/controller envelope; not vendor firmware/hardware',
            'Stop/watchdog measured after a bounded settling window, not instantaneous braking',
            'Reset here is a full owned Stop/Run relaunch; robot-only reset and monotonic clock require separate mode proof',
            'RTF is measured on this host/run only'])

    batch = args.runtime/'turtlebot3-current-drive/batch-report.json'
    if batch.is_file():
        rows = json.loads(batch.read_text())
        assert len(rows) == 12 and all(r['passed'] and r['producer_returncode'] == r['launch_returncode'] == 0 for r in rows)
        for row in rows:
            raw = Path(row['report']).parent
            folder = dest/'regressions/turtlebot3'/raw.name
            copy_trial(raw, folder)
            report = json.loads((raw/'report.json').read_text())
            trace = json.loads((raw/'trace.json').read_text())
            report['max_body_tilt_rad'] = max(s['tilt'] for s in trace['body'])
            report['joint_position_ranges'] = {}
            for wheel in ('wheel_left_joint', 'wheel_right_joint'):
                values = [j['positions'][j['names'].index(wheel)] for j in trace['joints'] if wheel in j['names']]
                report['joint_position_ranges'][wheel] = max(values)-min(values)
            report['derived_from'] = dict(producer_report=artifact(raw/'report.json'), physical_trace=artifact(raw/'trace.json'))
            assert support.measured_result(report, 'drive_lidar_screen'), raw.name
            record(report, raw, folder, 'drive_lidar_screen', 'teleoperation', [
                'Current shared wheel/spawner regression; does not rerun all historical localization/mapping/navigation cells',
                'Named nav_empty actual GUI Drive/Stop/watchdog and source LDS/IMU mounts; physical joystick/other maps untested',
                'Source RGB-only cameras remain disabled; no borrowed RGB-D or 3D SLAM claim'])
        shutil.copy2(batch, dest/'regressions/turtlebot3/batch-report.json')
        shutil.copy2(args.runtime/'turtlebot3-regression-batch.py', dest/'regressions/turtlebot3/batch-runner.py')

    shared = args.runtime/'shared-regressions/batch-report.json'
    if shared.is_file():
        rows = json.loads(shared.read_text())
        assert len(rows) == 4 and all(r['passed'] and r['producer_returncode'] == r['launch_returncode'] == 0 for r in rows)
        for row in rows:
            raw = Path(row['report']).parent
            folder = dest/'regressions'/row['job']
            copy_trial(raw, folder)
            if row['job'].startswith('asset_turtlebot4'):
                report = json.loads((raw/'report.json').read_text())
                assert support.measured_result(report, 'drive_sensor_screen'), row['job']
                record(report, raw, folder, 'drive_sensor_screen', 'teleoperation', [
                    'Current Isaac physical Drive/lidar/RGB-D regression; historical higher-mode screens retain their own revisions',
                    'Other maps, physical joystick and vendor firmware/hazard/docking behavior unqualified'])
            elif row['job'] == 'panda-planning':
                report = json.loads((raw/'report.json').read_text())
                assert support.measured_result(report, 'arm_cartesian_screen')
                record(report, raw, folder, 'arm_cartesian_screen', 'arm_cartesian', [
                    'Current native Panda MuJoCo/nav_empty physical MoveIt regression',
                    'Servo, attached/dynamic objects, other arms/hands/backends and mobile manipulation unqualified'])
        shutil.copy2(shared, dest/'regressions/batch-report.json')
        shutil.copy2(args.runtime/'shared-regression-batch.py', dest/'regressions/batch-runner.py')

    for stage in ('modes', 'modes-current'):
        for raw in sorted((args.runtime/'husky'/stage).glob('*')):
            if not raw.is_dir() or not (raw/'report.json').is_file(): continue
            folder = dest/'husky'/stage/raw.name
            copy_trial(raw, folder)
            wrapper = json.loads((raw/'report.json').read_text())
            measured = dict(wrapper.get('workflow', {}), robot=wrapper['robot'], backend=wrapper['backend'], map=wrapper['map'],
                            scope=wrapper['scope']+' Experimental provider is declared in the original report.')
            if wrapper['mode'] == 'nav':
                kind, task = 'navigation_screen', 'navigation'
                measured['gui_recording_completed'] = (wrapper.get('passed') is True
                    and wrapper.get('launch_returncode') == 0)
                trace_path = raw/'navigation.trace.json'
                if trace_path.is_file():
                    samples = json.loads(trace_path.read_text())['samples']
                    source = json.loads((raw/'source-manifest.json').read_text())
                    measured['body_height'] = dict(
                        expected_floor_m=float(source['map_profile'].get('spawn', {}).get('z', 0)),
                        minimum_m=min(s['z'] for s in samples),
                        maximum_m=max(s['z'] for s in samples), samples=len(samples))
                    measured['derived_from'] = dict(physical_trace=artifact(trace_path),
                        original_report=artifact(raw/'report.json'))
            else:
                kind, task = 'workflow_screen', wrapper['mode'] if wrapper['mode'] in ('slam', '3d_slam') else 'localization'
                measured['checks'] = dict(measured.get('checks', {}),
                    gui_recording_completed=wrapper.get('passed') is True and wrapper.get('launch_returncode') == 0)
            record(measured, raw, folder, kind, task, [
                'Exact declared provider/source/map/backend trial; production modes are granted separately by hashed certificates',
                'Short static workflow; no universal-map, long mission, contact telemetry or vendor hardware claim'])
            for name in ('saved-map.yaml', 'saved-map.pgm'):
                if (raw/name).is_file(): shutil.copy2(raw/name, folder/name)

    # Keep earlier imported-world failures before the corrected current
    # repeats so the latest exact cell remains visible in the support matrix.
    navigation = sorted((args.runtime/'navigation').rglob('report.json'))
    navigation.sort(key=lambda p: p.stat().st_mtime)
    normal = args.runtime/'husky/normal-production-navigation/report.json'
    if normal.is_file(): navigation.append(normal)
    for path in navigation:
        raw = path.parent
        folder = dest/raw.relative_to(args.runtime)
        copy_trial(raw, folder)
        wrapper = json.loads(path.read_text())
        measured = dict(wrapper.get('navigation', wrapper.get('workflow', {})),
            robot=wrapper['robot'], backend=wrapper['backend'], map=wrapper['map'],
            scope=wrapper['scope'], gui_recording_completed=wrapper.get('passed') is True
                  and wrapper.get('launch_returncode') == 0)
        if 'body_height' in wrapper: measured['body_height'] = wrapper['body_height']
        trace = raw/'navigation.trace.json'
        if trace.is_file():
            samples = json.loads(trace.read_text())['samples']
            source = json.loads((raw/'source-manifest.json').read_text())
            measured['body_height'] = dict(
                expected_floor_m=float(source['map_profile'].get('spawn', {}).get('z', 0)),
                minimum_m=min(s['z'] for s in samples), maximum_m=max(s['z'] for s in samples),
                samples=len(samples))
            measured['derived_from'] = dict(physical_trace=artifact(trace), original_report=artifact(path))
        record(measured, raw, folder, 'navigation_screen', 'navigation', [
            'Short named static ground-floor route; no upper-floor, actor or universal-map qualification',
            'Independent body position/heading/floor checks; complete mesh/contact clearance remains separate',
            'Owned launch root cleanup is recorded; individual child shutdown errors remain in raw logs'])

    for raw_name, label, kind, task in (
            ('panda-intuitive-controls-normal', 'panda-normal', 'arm_cartesian_screen', 'arm_cartesian'),
            ('px4-seven-directions', 'px4', 'flight_manual_screen', 'flight')):
        raw = args.runtime/raw_name
        if not (raw/'report.json').is_file():
            continue
        folder = dest/'controls'/label
        copy_trial(raw, folder)
        report = json.loads((raw/'report.json').read_text())
        if kind == 'flight_manual_screen':
            trace = json.loads((raw/'trace.json').read_text())['body']
            assert trace and all(math.isfinite(p[k]) for p in trace for k in ('x', 'y', 'z', 'yaw'))
            report['body_trace'] = dict(samples=len(trace), initial_z=trace[0]['z'],
                min_z=min(p['z'] for p in trace), max_z=max(p['z'] for p in trace))
            report['derived_from'] = dict(original_report=artifact(raw/'report.json'), physical_trace=artifact(raw/'trace.json'))
        assert support.measured_result(report, kind), raw_name
        record(report, raw, folder, kind, task, [
            'Exact current normal owned GUI control workflow and original trace/source hashes',
            'One static nav_empty screen; other robots/maps/backends, payload/Servo and aerial obstacle planning remain unqualified'])

    for raw in sorted((args.runtime/'drive-current-controls').glob('*')):
        if not raw.is_dir() or not (raw/'report.json').is_file():
            continue
        folder = dest/'controls/drive'/raw.name
        copy_trial(raw, folder)
        report = json.loads((raw/'report.json').read_text())
        if report['robot'] != 'asset_husky':
            continue  # Core Drive receipts remain distinct from the four-wheel source-sensor gate.
        trace = json.loads((raw/'trace.json').read_text())
        report['max_body_tilt_rad'] = max(p['tilt'] for p in trace['body'])
        report['sensors']['camera_info'] = trace['camera_info'][-1]
        report['derived_from'] = dict(original_report=artifact(raw/'report.json'), physical_trace=artifact(raw/'trace.json'))
        record(report, raw, folder, 'skid_drive_screen', 'teleoperation', [
            'Current shared Drone/Drive GUI regression with actual four-wheel/source-sensor/body feedback',
            'Historical twenty higher-mode trials retain their source stages; this Drive repeat does not rerun them'])

    # Preserve diagnostic controls and core-base Drive receipts without
    # attributing the Husky four-wheel/sensor gate to a different topology.
    for raw_name, archive_name in (
            ('panda-intuitive-controls-experimental', 'panda-producer-error'),
            ('panda-intuitive-controls-experimental-fixed', 'panda-sag-negative'),
            ('panda-intuitive-controls-bias', 'panda-bias-experimental'),
            ('panda-hand-current-controls', 'panda-contact-point-negative'),
            ('panda-hand-current-controls-contact-sum', 'panda-hand')):
        raw = args.runtime/raw_name
        if (raw/'report.json').is_file():
            copy_trial(raw, dest/'controls'/archive_name)
    for stage in ('core-drive-display-routing', 'core-drive-rotor-matched'):
        for raw in sorted((args.runtime/stage).glob('*')):
            if raw.is_dir() and (raw/'report.json').is_file():
                copy_trial(raw, dest/'controls/core'/stage/raw.name)

    index['records'] = old_records+records
    index['date'] = '2026-10-08'
    index['scope'] = 'Exact hashed named simulation screens through October 8; unlisted cells remain untested.'
    index_path.write_text(yaml.safe_dump(index, sort_keys=False))
    (dest/'checkpoint-collection.json').write_text(json.dumps(dict(raw_root=str(args.runtime),
        collection_source=artifact(Path(__file__)), trials=manifest, indexed_records=len(records)), indent=2)+'\n')
    print('Archived', len(manifest), 'actual trials and', len(records), 'measured scoped records; no mode grants.')


if __name__ == '__main__':
    main()
