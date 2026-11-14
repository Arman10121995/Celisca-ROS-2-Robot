#!/usr/bin/env python3
"""Licenses of the external source checkouts the lab runs (ROADMAP R9.1).

The dependency inventory (``license_inventory.py``) covers ROS/Python/Debian
packages. This covers the pinned source trees used at runtime: the PX4
checkout (and each of its git submodules) and every vendor source under
``$ROBOT_LAB_RUNTIME_ROOT/external_assets/sources/<source>/<revision>/``.
Each tree's license files (LICENSE, LICENCE, COPYING, NOTICE) are read and
identified by their text; ``package.xml`` ``<license>`` tags are listed
next to them. Nothing is inferred from a project's name: a tree without a
license file stays ``unresolved``, and one whose files disagree is ``mixed``.

  python3 scripts/external_source_licenses.py --json out.json --markdown out.md
"""
import argparse
import json
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

LICENSE_FILE = re.compile(r'^(licen[cs]e|copying|notice)(\.(md|txt|rst))?$|^licen[cs]e[-_.]', re.I)
SIGNATURES = [
    ('Apache-2.0', [r'apache license', r'version 2\.0']),
    ('MPL-2.0', [r'mozilla public license', r'2\.0']),
    ('EPL-2.0', [r'eclipse public license', r'2\.0']),
    ('LGPL-3.0', [r'gnu lesser general public license', r'version 3']),
    ('LGPL-2.1', [r'gnu lesser general public license', r'version 2\.1']),
    ('GPL-3.0', [r'gnu general public license', r'version 3']),
    ('GPL-2.0', [r'gnu general public license', r'version 2']),
    ('CC-BY-4.0', [r'creative commons attribution 4\.0']),
    ('CC-BY-SA-4.0', [r'creative commons attribution-sharealike 4\.0']),
    ('BSL-1.0', [r'boost software license']),
    ('Unlicense', [r'this is free and unencumbered software released into the public domain']),
    ('Zlib', [r'this software is provided .as-is.', r'altered source versions must be plainly marked']),
    ('MIT', [r'permission is hereby granted, free of charge']),
    ('ISC', [r'permission to use, copy, modify, and(/or)? distribute this software for any purpose']),
]


def identify(text):
    lower = re.sub(r'\s+', ' ', text.lower())
    for spdx, patterns in SIGNATURES:
        if all(re.search(p, lower) for p in patterns):
            return spdx
    if 'redistribution and use in source and binary forms' in lower:
        clauses = len(re.findall(r'neither the name|endorse or promote', lower))
        return 'BSD-3-Clause' if clauses else 'BSD-2-Clause'
    if 'public domain' in lower:
        return 'public-domain'
    return None


def scan_tree(root, depth=1):
    """License files at the tree root (and one level down for multi-package repos)."""
    root = Path(root)
    found = []
    candidates = [p for p in root.iterdir() if p.is_file()] if root.is_dir() else []
    if depth:
        for child in (root.iterdir() if root.is_dir() else []):
            if child.is_dir() and not child.name.startswith('.'):
                candidates += [p for p in child.iterdir() if p.is_file() and LICENSE_FILE.match(p.name)]
    for path in sorted(candidates):
        if LICENSE_FILE.match(path.name):
            try:
                text = path.read_text(errors='replace')[:20000]
            except OSError:
                continue
            found.append(dict(file=str(path.relative_to(root)), license=identify(text)))
    tags = set()
    for package in list(root.glob('package.xml')) + list(root.glob('*/package.xml')):
        try:
            tags.update(t.text.strip() for t in ET.parse(package).getroot().findall('license') if t.text)
        except ET.ParseError:
            pass
    licenses = sorted({f['license'] for f in found if f['license']})
    status = ('unresolved' if not licenses else 'mixed' if len(licenses) > 1 else 'resolved')
    if found and any(f['license'] is None for f in found) and status == 'resolved':
        status = 'resolved (some files unidentified)'
    return dict(path=str(root), license_files=found, licenses=licenses, package_xml=sorted(tags), status=status)


def git_revision(path):
    try:
        return subprocess.run(['git', '-C', str(path), 'rev-parse', 'HEAD'], capture_output=True, text=True,
                              timeout=10).stdout.strip() or None
    except (OSError, subprocess.TimeoutExpired):
        return None


def px4(root):
    tree = scan_tree(root, depth=0)
    tree.update(name='PX4-Autopilot', revision=git_revision(root))
    try:
        tag = subprocess.run(['git', '-C', str(root), 'describe', '--tags'], capture_output=True, text=True,
                             timeout=10).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        tag = ''
    tree['tag'] = tag or None
    modules = []
    gitmodules = Path(root) / '.gitmodules'
    paths = re.findall(r'path = (.+)', gitmodules.read_text()) if gitmodules.is_file() else []
    for relative in paths:
        sub = Path(root) / relative.strip()
        if not sub.is_dir() or not any(sub.iterdir()):
            modules.append(dict(name=relative.strip(), status='not checked out'))
            continue
        row = scan_tree(sub)
        row.update(name=relative.strip(), revision=git_revision(sub))
        modules.append(row)
    tree['submodules'] = modules
    return tree


def vendor_sources(store):
    rows = []
    for source in sorted(Path(store).iterdir()) if Path(store).is_dir() else []:
        for revision in sorted(p for p in source.iterdir() if p.is_dir()):
            row = scan_tree(revision)
            row.update(name=source.name, revision=revision.name)
            rows.append(row)
    return rows


def markdown(result):
    lines = ['# External source licenses', '', 'Generated by `scripts/external_source_licenses.py` from the license '
             'files on disk; nothing is inferred from project names.', '']
    p = result['px4']
    lines += ['## PX4-Autopilot', '', '%s at `%s` (%s): **%s** (%s)' % (p['path'], p.get('revision'), p.get('tag'),
              ', '.join(p['licenses']) or 'unresolved', p['status']), '',
              '| Submodule | Revision | License | Status |', '|---|---|---|---|']
    for m in p['submodules']:
        lines.append('| %s | %s | %s | %s |' % (m['name'], (m.get('revision') or '')[:10],
                     ', '.join(m.get('licenses', [])) or '-', m['status']))
    lines += ['', '## Vendor sources', '', '| Source | Revision | License files | package.xml | Status |', '|---|---|---|---|---|']
    for r in result['vendor_sources']:
        lines.append('| %s | %s | %s | %s | %s |' % (r['name'], r['revision'][:10], ', '.join(r['licenses']) or '-',
                     ', '.join(r['package_xml']) or '-', r['status']))
    counts = result['summary']
    lines += ['', 'Summary: %s.' % ', '.join('%s %d' % (k, v) for k, v in counts.items())]
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--px4', default=os.environ.get('PX4_ROOT', '/workspace/molar/px4/PX4-Autopilot'))
    parser.add_argument('--store', default=str(Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
                                               / 'external_assets' / 'sources'))
    parser.add_argument('--json', default='')
    parser.add_argument('--markdown', default='')
    options = parser.parse_args()
    result = dict(px4=px4(options.px4), vendor_sources=vendor_sources(options.store))
    statuses = [result['px4']['status']] + [m['status'] for m in result['px4']['submodules']] + \
               [r['status'] for r in result['vendor_sources']]
    result['summary'] = {s: statuses.count(s) for s in sorted(set(statuses))}
    if options.json:
        Path(options.json).write_text(json.dumps(result, indent=2) + '\n')
    if options.markdown:
        Path(options.markdown).write_text(markdown(result))
    print(json.dumps(result['summary']))


if __name__ == '__main__':
    main()
