#!/usr/bin/env python3
"""R9.3: append sim_nav_mission reports to the runtime evidence index.

Each group in the batch YAML names report globs from one evidence round,
the revision text recorded in that round's README, the commit whose files
are hashed (`hash_revision`), the measured scope and its limitations.  For
every group the script writes a source-hash manifest beside the reports
(`source-hashes-<hash_revision>.json`: sha256 of each listed file as git
stores it at that commit) and appends one record per report, in the
batch's order, so the latest round controls a cell in the matrix.

Only sim_nav_mission reports (a 'steps' object) are indexed, unless the
group says ``report_kind: drive_screen`` (sim_drive_check.py reports, a
'phases' list) with an explicit ``identity: {robot_id, environment_id,
simulator, task_type}``.  Report
identity comes from the file name: <simulator>_<robot>[_seedN]_<map>.json,
or, for the 4WS rounds, <simulator>_<pattern>[_seedN]_<map>.json (pattern mecanum
means the mecanum car).  Records already indexed (same report path) are
skipped.

  python3 scripts/r9_3_index_missions.py batch.yaml
"""
import glob
import hashlib
import json
import os
import subprocess
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDEX = os.path.join(ROOT, 'docs', 'status', 'runtime-evidence-index.yaml')
SIMULATORS = ('gazebo', 'mujoco', 'pybullet', 'isaac')
PATTERNS = ('ackermann', 'in_phase', 'crab', 'pivot', 'mecanum')


def robots():
    data = yaml.safe_load(open(os.path.join(ROOT, 'src', 'robot_lab_robots', 'config', 'robots.yaml')))
    return sorted(data['robots'], key=len, reverse=True)


def identity(name, robot_ids, four_wheel):
    stem = os.path.basename(name)[:-len('.json')]
    simulator, rest = stem.split('_', 1)
    if simulator not in SIMULATORS:
        raise ValueError('no simulator in %s' % name)
    if four_wheel:
        pattern = next(p for p in PATTERNS if rest.startswith(p + '_'))
        environment = rest[len(pattern) + 1:]
        if environment.startswith('seed'):
            environment = environment.split('_', 1)[1]
        if pattern == 'mecanum':
            return 'mecanum_car', environment, simulator, ''
        return 'four_wheel_steer_car', environment, simulator, pattern
    robot = next(r for r in robot_ids if rest.startswith(r + '_'))
    rest = rest[len(robot) + 1:]
    if rest.startswith('seed'):
        rest = rest.split('_', 1)[1]
    return robot, rest, simulator, ''


def git_sha256(revision, path):
    blob = subprocess.run(['git', '-C', ROOT, 'show', '%s:%s' % (revision, path)], capture_output=True)
    if blob.returncode:
        return None   # the file did not exist at that revision
    return hashlib.sha256(blob.stdout).hexdigest()


def main():
    batch = yaml.safe_load(open(sys.argv[1]))
    text_before = open(INDEX).read()
    index = yaml.safe_load(text_before)
    if list(index)[-1] != 'records':
        raise SystemExit('records must be the last key of the index to append to it')
    known = {record['report'] for record in index['records']}
    new_records = []
    robot_ids = robots()
    added = 0
    for group in batch['groups']:
        reports = sorted(p for pattern in group['reports'] for p in glob.glob(os.path.join(ROOT, pattern)))
        if not reports:
            raise SystemExit('no reports for %s' % group['reports'])
        folder = os.path.dirname(reports[0])
        manifest = os.path.join(folder, 'source-hashes-%s.json' % group['hash_revision'])
        files = sorted(set(batch['source_files'] + group.get('source_files', [])))
        content = {'revision': group['hash_revision'],
                   'derived': 'sha256 of each file as git stores it at this revision, computed %s after the '
                              % batch.get('date', 'the batch date') +
                              'run; uncommitted working-tree changes at run time are not captured',
                   'files': {f: git_sha256(group['hash_revision'], f) for f in files}}
        text = json.dumps(content, indent=1) + '\n'
        if os.path.exists(manifest) and open(manifest).read() != text:
            # Another group of the same folder and revision hashed other files.
            manifest = manifest[:-len('.json')] + '-%s.json' % hashlib.sha256(text.encode()).hexdigest()[:8]
        with open(manifest, 'w') as handle:
            handle.write(text)
        for report in reports:
            relative = os.path.relpath(report, ROOT)
            if relative in known:
                continue
            content = json.load(open(report))
            kind = group.get('report_kind', 'mission_screen')
            if kind == 'drive_screen':
                if not (isinstance(content, dict) and isinstance(content.get('phases'), list)):
                    continue
                fixed = group['identity']
                robot, environment, simulator, steering = (fixed['robot_id'], fixed['environment_id'],
                                                           fixed['simulator'], fixed.get('steering_mode', ''))
            elif not (isinstance(content, dict) and isinstance(content.get('steps'), dict)):
                continue   # leg-odometry traces and other side files are not missions
            else:
                robot, environment, simulator, steering = identity(group.get('name_override', report), robot_ids,
                                                                   group.get('four_wheel', False))
            new_records.append({
                'robot_id': robot, 'environment_id': environment, 'simulator': simulator,
                'task_type': group.get('identity', {}).get('task_type', 'navigation_mission'),
                'steering_mode': steering, 'date': group['date'],
                'revision': group['revision'], 'report': relative,
                'sha256': hashlib.sha256(open(report, 'rb').read()).hexdigest(),
                'source_hashes': os.path.relpath(manifest, ROOT),
                'source_hashes_sha256': hashlib.sha256(text.encode()).hexdigest(),
                'report_kind': kind, 'scope': group['scope'],
                'limitations': list(group['limitations'])})
            known.add(relative)
            added += 1
    if not new_records:
        print('nothing new to index; index unchanged (%d records)' % len(index['records']))
        return
    # Append as text: the existing records keep their bytes.
    class NoAliases(yaml.SafeDumper):
        def ignore_aliases(self, data):
            return True
    appended = yaml.dump(new_records, Dumper=NoAliases, sort_keys=False, width=110, allow_unicode=True)
    lines = text_before.splitlines(keepends=True)
    head = {'date': "date: '%s'\n" % batch['date']}
    for i, line in enumerate(lines):
        if line.startswith('date:'):
            lines[i] = head['date']
            break
    for i, line in enumerate(lines):
        if line.startswith('scope:'):
            end = i + 1
            while end < len(lines) and lines[end].startswith(' '):
                end += 1
            lines[i:end] = ['scope: %s\n' % json.dumps(batch['scope'])]
            break
    text = ''.join(lines)
    if not text.endswith('\n'):
        text += '\n'
    text += appended
    check = yaml.safe_load(text)
    assert len(check['records']) == len(index['records']) + added
    with open(INDEX, 'w') as handle:
        handle.write(text)
    print('added %d records; index now %d' % (added, len(check['records'])))


if __name__ == '__main__':
    main()
