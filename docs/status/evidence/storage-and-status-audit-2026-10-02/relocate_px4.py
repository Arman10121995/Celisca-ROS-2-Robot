#!/usr/bin/env python3
"""One-off verified relocation, preserving the original /tmp paths."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import stat
import subprocess
import time

BASE = Path('/workspace/molar/robot_lab_runtime')
DEST = BASE / 'px4' / 'legacy-tmp'
REPORT = BASE / 'storage-migration-2026-10-02' / 'manifest.json'
DEST.mkdir(parents=True, exist_ok=True)
if os.stat('/workspace').st_dev == os.stat('/tmp').st_dev:
    raise RuntimeError('workspace and /tmp are not separate filesystems')
items = sorted(p for p in Path('/tmp').iterdir()
               if p.name.startswith(('px4', 'v162'))
               and not p.is_symlink() and (p.is_file() or p.is_dir()))

def active_references():
    refs = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            links = [proc/'cwd'] + list((proc/'fd').iterdir())
        except (FileNotFoundError, PermissionError):
            continue
        for link in links:
            try:
                target = os.readlink(link).removesuffix(' (deleted)')
            except OSError:
                continue
            for p in items:
                if target == str(p) or target.startswith(str(p) + '/'):
                    refs.append({'pid': proc.name, 'reference': str(link), 'target': target})
    return refs

def snapshot(path):
    result = []
    paths = [path] + sorted(path.rglob('*')) if path.is_dir() else [path]
    for p in paths:
        st = p.lstat()
        entry = {'path': '.' if p == path else str(p.relative_to(path)),
                 'mode': stat.S_IMODE(st.st_mode), 'uid': st.st_uid, 'gid': st.st_gid,
                 'mtime_ns': st.st_mtime_ns}
        if p.is_symlink():
            entry.update(type='symlink', target=os.readlink(p))
        elif p.is_file():
            h = hashlib.sha256()
            with p.open('rb') as f:
                for block in iter(lambda: f.read(8*1024*1024), b''):
                    h.update(block)
            entry.update(type='file', size=st.st_size, sha256=h.hexdigest())
        elif p.is_dir():
            entry.update(type='directory')
        else:
            raise RuntimeError('unsupported special file: %s' % p)
        if (p.lstat().st_size, p.lstat().st_mtime_ns) != (st.st_size, st.st_mtime_ns):
            raise RuntimeError('file changed while reading: %s' % p)
        result.append(entry)
    return result

report = {'date': '2026-10-02', 'source_device': os.stat('/tmp').st_dev,
          'destination_device': os.stat(DEST).st_dev, 'state': 'active',
          'method': 'rsync -aHAX; SHA-256/metadata equality; rename; compatibility symlink; remove verified old copy',
          'before_disk': dict(zip(('total','used','free'), shutil.disk_usage('/'))), 'items': []}
def save():
    staging = REPORT.with_suffix('.new')
    staging.write_text(json.dumps(report, indent=2) + '\n')
    staging.replace(REPORT)

refs = active_references()
if refs:
    raise RuntimeError('active source references: %s' % refs)
save()
for src in items:
    dst = DEST / src.name
    backup = src.with_name(src.name + '.ssd-migration-20261002')
    if dst.exists() or dst.is_symlink() or backup.exists():
        raise RuntimeError('refusing to replace existing destination/backup: %s' % src)
    before = snapshot(src)
    subprocess.run(['rsync', '-aHAX', '--', str(src), str(DEST) + '/'], check=True)
    after = snapshot(dst)
    if before != after:
        raise RuntimeError('content or metadata mismatch: %s' % src)
    if active_references():
        raise RuntimeError('an active reference appeared before replacing %s' % src)
    src.rename(backup)
    try:
        src.symlink_to(dst, target_is_directory=dst.is_dir())
    except BaseException:
        backup.rename(src)
        raise
    if src.resolve() != dst or os.stat(src).st_dev != os.stat(DEST).st_dev:
        raise RuntimeError('compatibility link failed: %s' % src)
    report['items'].append({'original': str(src), 'destination': str(dst),
                            'verified': True, 'snapshot': before,
                            'bytes': sum(e.get('size', 0) for e in before)})
    save()
    if backup.is_dir():
        shutil.rmtree(backup)
    else:
        backup.unlink()
    print('VERIFIED AND MOVED', src, report['items'][-1]['bytes'], flush=True)
report.update(state='complete', after_disk=dict(zip(('total','used','free'), shutil.disk_usage('/'))))
report['total_bytes'] = sum(e['bytes'] for e in report['items'])
save()
print('COMPLETE', len(report['items']), report['total_bytes'], report['after_disk'], flush=True)
