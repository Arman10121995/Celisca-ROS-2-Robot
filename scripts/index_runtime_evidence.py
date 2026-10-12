#!/usr/bin/env python3
"""Add measured reports to docs/status/runtime-evidence-index.yaml (R9.3).

The support matrix (scripts/r9_3_support_matrix.py) only counts a cell
when its report and a source-hash manifest are hashed in the index.  This
appends records from a JSON spec:

    {"revision": "<git revision the run used>", "revision_note": "...",
     "source_files": ["src/...", ...],          # hashed at that revision
     "manifest": "docs/status/evidence/<set>/source-hashes.json",
     "report_kind": "mission_screen" | "drive_screen" | ...,
     "task_type": "...", "date": "YYYY-MM-DD", "scope": "...",
     "limitations": ["..."],
     "records": [{"robot_id": ..., "environment_id": ..., "simulator": ...,
                  "steering_mode": "", "report": "docs/status/evidence/..."}]}

The manifest holds the sha256 of each source file *as committed at the
revision* (``git show <rev>:<file>``), so it records what ran, not the
working tree.  Existing records are kept; a record whose report is
already indexed is skipped.

  python3 scripts/index_runtime_evidence.py spec.json
"""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def source_manifest(revision, files):
    manifest = {'revision': revision, 'files': {}}
    for name in files:
        blob = subprocess.run(['git', '-C', str(ROOT), 'show', '%s:%s' % (revision, name)],
                              capture_output=True, check=True).stdout
        manifest['files'][name] = sha256(blob)
    return manifest


def main(spec_path):
    spec = json.loads(Path(spec_path).read_text())
    manifest_path = ROOT / spec['manifest']
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(source_manifest(spec['revision'], spec['source_files']),
                                        indent=1, sort_keys=True) + '\n')
    manifest_hash = sha256(manifest_path.read_bytes())
    index_path = ROOT / 'docs/status/runtime-evidence-index.yaml'
    text = index_path.read_text()
    index = yaml.safe_load(text)
    known = {record.get('report') for record in index.get('records', [])}
    new_records = []
    for item in spec['records']:
        report = ROOT / item['report']
        if item['report'] in known:
            continue
        new_records.append(dict(
            robot_id=item['robot_id'], environment_id=item['environment_id'],
            simulator=item['simulator'], task_type=spec['task_type'],
            steering_mode=item.get('steering_mode', ''), date=spec['date'],
            revision='%s (%s)' % (spec['revision'], spec['revision_note']),
            report=item['report'], sha256=sha256(report.read_bytes()),
            source_hashes=spec['manifest'], source_hashes_sha256=manifest_hash,
            report_kind=spec['report_kind'], scope=spec['scope'],
            limitations=list(spec.get('limitations', []))))
    # Append as text: the existing records keep their exact formatting.
    if new_records:
        import re
        text = re.sub(r"^date: .*$", "date: '%s'" % spec['date'], text, count=1, flags=re.M)
        if not text.endswith('\n'):
            text += '\n'
        text += yaml.safe_dump(new_records, sort_keys=False, width=100)
        index_path.write_text(text)
        assert len(yaml.safe_load(text)['records']) == len(index['records']) + len(new_records)
    print(json.dumps({'added': len(new_records), 'records': len(index['records']) + len(new_records)}))


if __name__ == '__main__':
    main(sys.argv[1])
