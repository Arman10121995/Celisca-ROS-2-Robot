"""Archive real TurtleBot3 GUI trials and derive envelopes from physical traces.

The producer's original report is preserved byte-for-byte. report-measured.json
adds extrema calculated from its raw trace; neither this script nor unit tests
produce a simulator success. Only the final serial batch is indexed.
"""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys

import yaml

repo = Path.cwd()
runtime = Path(__file__).parent
destination = repo/'docs/status/evidence/turtlebot3-sensors-2026-10-07'
destination.mkdir(parents=True, exist_ok=True)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def artifact(path):
    return dict(path=str(path), sha256=sha(path), bytes=path.stat().st_size)


spec = importlib.util.spec_from_file_location('support_matrix', repo/'scripts/r9_3_support_matrix.py')
support = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = support
spec.loader.exec_module(support)
batch = json.loads((runtime/'sensors-final-batch.json').read_text())
expected = {'asset_turtlebot3_'+variant+'-'+backend
            for variant in ('burger', 'waffle', 'waffle_pi')
            for backend in ('gazebo', 'mujoco', 'pybullet', 'isaac')}
assert {row['case'] for row in batch} == expected and len(batch) == 12
index_path = repo/'docs/status/runtime-evidence-index.yaml'
index = yaml.safe_load(index_path.read_text())
prefix = str(destination.relative_to(repo))
index['records'] = [record for record in index['records']
                    if not str(record['report']).startswith(prefix)]
manifest = dict(raw_root=str(runtime/'sensors-final'),
                baseline='18ecdd24172fd6adfa0d5e2c55809dd2d882d4ae plus exact pre-trial source/installed manifests',
                records=[], retained_stages=[])

for result in batch:
    assert result['returncode'] == 0 and result['passed'] is True
    raw = runtime/'sensors-final'/result['case']
    report = json.loads((raw/'report.json').read_text())
    source = json.loads((raw/'source-manifest.json').read_text())
    trace = json.loads((raw/'trace.json').read_text())
    assert report['passed'] is True and report['launch_returncode'] == 0
    assert source['producer_sha256'] == sha(raw/'producer.py')
    assert source['executed_urdf_sha256'] == sha(raw/'executed.urdf')
    # The producer records these counts before Stop/Close, then continues
    # observing while waiting for the owned launch to exit. The raw trace
    # includes that additional tail; extrema below include it too.
    assert len(trace['body']) >= report['body_samples']
    assert len(trace['joints']) >= report['joint_samples']
    assert all(math.isfinite(value) for body in trace['body'] for value in body.values())
    assert all(math.isfinite(value) for joint in trace['joints']
               for value in joint['positions']+joint['velocities'])
    measured = dict(report)
    measured['max_body_tilt_rad'] = max(body['tilt'] for body in trace['body'])
    measured['trace_samples_including_shutdown'] = dict(body=len(trace['body']), joints=len(trace['joints']))
    measured['joint_position_ranges'] = {}
    for wheel in ('wheel_left_joint', 'wheel_right_joint'):
        positions = [joint['positions'][joint['names'].index(wheel)]
                     for joint in trace['joints'] if wheel in joint['names']]
        measured['joint_position_ranges'][wheel] = max(positions)-min(positions)
    measured['derived_from'] = dict(producer_report=artifact(raw/'report.json'),
                                    physical_trace=artifact(raw/'trace.json'))
    assert support.measured_result(measured, 'drive_lidar_screen'), result['case']
    folder = destination/result['case']
    folder.mkdir(exist_ok=True)
    for name in ('report.json', 'source-manifest.json', 'producer.py'):
        shutil.copy2(raw/name, folder/name)
    (folder/'report-measured.json').write_text(json.dumps(measured, indent=2)+'\n')
    manifest['records'].append(dict(case=result['case'], producer_returncode=result['returncode'],
        raw_directory=str(raw), artifacts=[artifact(p) for p in sorted(raw.iterdir()) if p.is_file()]))
    index['records'].append(dict(robot_id=report['robot'], environment_id=report['map'],
        simulator=report['backend'], task_type='teleoperation', date='2026-10-07',
        revision=source['git_head']+' plus exact source/installed stage in pre-trial manifest',
        report=str((folder/'report-measured.json').relative_to(repo)),
        sha256=sha(folder/'report-measured.json'),
        source_hashes=str((folder/'source-manifest.json').relative_to(repo)),
        source_hashes_sha256=sha(folder/'source-manifest.json'), report_kind='drive_lidar_screen',
        scope=report['scope']+' Joint ranges/tilt derived from the retained physical trace. '
              'Serial producer exit 0 and owned plant cleanup checked by the archived batch runner.',
        limitations=['Named nav_empty GUI keyboard/Drive screen; physical joystick and other maps untested',
            'Original 360-ray LDS with 0.12–3.5 m bounds at 5 Hz; source cameras are RGB-only and rendering is disabled',
            'Lab wheel control and virtual-frame inertia regularization, not OpenCR/vendor firmware qualification',
            'Stop/watchdog limits use a final window after 2.5 simulation seconds, not instantaneous physical braking',
            'RTF is measured on this host/run, not a general performance guarantee']))

for stage in sorted(runtime.glob('sensors-stage*')):
    if not stage.is_dir():
        continue
    record = dict(stage=stage.name, indexed=False, reports=[])
    for path in sorted(stage.glob('*/report.json')):
        report = json.loads(path.read_text())
        record['reports'].append(dict(artifact(path), passed=report.get('passed'),
                                      launch_returncode=report.get('launch_returncode')))
        if report.get('passed') is not True:
            folder = destination/'negative'/stage.name/path.parent.name
            folder.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, folder/'report.json')
    record['artifacts'] = [artifact(path) for path in sorted(stage.glob('*/*')) if path.is_file()]
    manifest['retained_stages'].append(record)
for name in ('sensors-final-batch.json', 'run_sensors-final.py', 'sensors-final-batch.log'):
    shutil.copy2(runtime/name, destination/name)
shutil.copy2(Path(__file__), destination/'collect_sensors.py')
(destination/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
index['date'] = '2026-10-07'
index['scope'] = 'Exact hashed October 5–7 named simulation screens; unlisted cells remain untested.'
index_path.write_text(yaml.safe_dump(index, sort_keys=False))
print('Archived twelve actual TurtleBot3 LDS/GUI Drive successes; earlier stages retained separately.')
