#!/usr/bin/env python3
"""R9.1 asset attribution: declared asset licenses against the files in the tree.

For every robot and environment in the registry (robot_lab_registry
config robots.yaml / environments.yaml) and every provisioned policy, this
finds the asset files the entry names, walks up from each to the nearest
LICENSE / COPYING file inside the workspace, and identifies that file's
license from its text.  It reports, per entry:

* the declared license (registry ``source.license``) and repository;
* the license file found next to the assets, and the license it contains;
* whether they agree, and entries whose assets carry no license file.

A missing license file does not mean the asset is unlicensed - the
workspace's own Apache-2.0 assets (Bumperbot, lab worlds) inherit the
package license - so each row says which file the conclusion rests on.

  python3 scripts/asset_attribution.py --out <dir>
"""
import argparse
import json
import os
import re

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, 'src')
REGISTRY = os.path.join(SRC, 'robot_lab', 'robot_lab_registry', 'config')
LICENSE_NAMES = re.compile(r'^(LICEN[CS]E|COPYING)([-_.].*)?$', re.I)

PATTERNS = [
    ('GPL-3.0', r'GNU GENERAL PUBLIC LICENSE\s+Version 3'),
    ('LGPL-2.1', r'GNU LESSER GENERAL PUBLIC LICENSE\s+Version 2\.1'),
    ('GPL-2.0', r'GNU GENERAL PUBLIC LICENSE\s+Version 2'),
    ('Apache-2.0', r'Apache License,?\s+Version 2\.0'),
    ('MPL-2.0', r'Mozilla Public License,?\s+(v\.\s*|version\s+)2\.0'),
    ('MIT', r'Permission is hereby granted, free of charge'),
    ('BSD-3-Clause', r'Neither the name of'),
    ('BSD-2-Clause', r'Redistribution and use in source and binary forms'),
    ('CC-BY-SA-4.0', r'Attribution-ShareAlike 4\.0 International'),
    ('CC-BY-4.0', r'Creative Commons Attribution 4\.0'),
]


def identify(path):
    try:
        text = open(path, errors='replace').read()
    except OSError:
        return None
    for name, pattern in PATTERNS:
        if re.search(pattern, text, re.I):
            return name
    return 'unrecognised'


def package_root(path):
    """The directory holding the nearest package.xml at or above path."""
    current = os.path.dirname(path)
    while current.startswith(SRC) and current != SRC:
        if os.path.exists(os.path.join(current, 'package.xml')):
            return current
        current = os.path.dirname(current)
    return None


def nearest_license(path):
    """(license file, its package) walking up from an asset file inside src/."""
    current = path if os.path.isdir(path) else os.path.dirname(path)
    while current.startswith(SRC) and current != SRC:
        for name in sorted(os.listdir(current)) if os.path.isdir(current) else []:
            if LICENSE_NAMES.match(name):
                return os.path.join(current, name)
        # Keep walking past package roots: vendored descriptions often carry
        # the license of the repository they were copied from higher up.
        if os.path.basename(current) in ('src',) or current == os.path.join(SRC, 'robot_lab_robots'):
            return None
        current = os.path.dirname(current)
    return None


def manifest_license(package_dir):
    try:
        text = open(os.path.join(package_dir, 'package.xml')).read()
    except OSError:
        return None
    match = re.search(r'<license>([^<]+)</license>', text)
    return match.group(1).strip() if match else None


def asset_paths(entry):
    paths = []
    assets = entry.get('assets') or {}
    for value in assets.values() if isinstance(assets, dict) else []:
        for item in value if isinstance(value, list) else [value]:
            if isinstance(item, str):
                paths.append(item)
    for key in ('world_file', 'map_file', 'world', 'map'):
        if isinstance(entry.get(key), str):
            paths.append(entry[key])
    out = []
    for p in paths:
        candidates = [os.path.join(SRC, p), os.path.join(SRC, 'robot_lab', p)]
        hit = next((c for c in candidates if os.path.exists(c)), None)
        if hit is None:
            found = [os.path.join(dp, f) for dp, _, fs in os.walk(SRC) for f in fs
                     if os.path.join(dp, f).endswith(p)] if '/' in p else []
            hit = found[0] if found else None
        out.append((p, hit))
    return out


def same(declared, detected):
    if not declared or not detected:
        return None
    norm = lambda s: re.sub(r'[^a-z0-9]', '', s.lower()).replace('license', '')  # noqa: E731
    d, f = norm(declared), norm(detected)
    return d == f or d.startswith(f) or f.startswith(d)


def entries():
    for kind, file in (('robot', 'robots.yaml'), ('environment', 'environments.yaml')):
        data = yaml.safe_load(open(os.path.join(REGISTRY, file)))
        items = data if isinstance(data, list) else list(data.values())
        for item in items:
            if isinstance(item, dict) and item.get('id'):
                yield kind, item
    policies = os.path.join(SRC, 'robot_lab_adapter', 'policies')
    for name in sorted(os.listdir(policies)):
        directory = os.path.join(policies, name)
        if os.path.isdir(directory):
            yield 'policy', {'id': name, 'assets': {'dir': os.path.relpath(directory, SRC)},
                             'source': {'license': None, 'repository': None}}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rows = []
    for kind, entry in entries():
        source = entry.get('source') or {}
        declared = source.get('license') if isinstance(source, dict) else None
        files = asset_paths(entry)
        resolved = [hit for _, hit in files if hit]
        license_files = sorted({nearest_license(hit) for hit in resolved} - {None})
        detected = sorted({identify(f) for f in license_files} - {None})
        packages = sorted({package_root(hit) for hit in resolved} - {None})
        manifests = sorted({manifest_license(p) for p in packages} - {None})
        if detected and detected != ['unrecognised']:
            basis = 'license file'
            agree = all(same(declared, d) for d in detected) if declared else None
        elif manifests:
            basis = 'package manifest'
            agree = all(same(declared, m) for m in manifests) if declared else None
        else:
            basis = 'none found'
            agree = None
        rows.append({
            'kind': kind, 'id': entry['id'], 'declared': declared,
            'repository': source.get('repository') if isinstance(source, dict) else None,
            'assets': len(files), 'assets_found': len(resolved),
            'license_files': [os.path.relpath(f, ROOT) for f in license_files], 'detected': detected,
            'package_manifest_licenses': manifests, 'basis': basis, 'agrees': agree,
        })
    os.makedirs(args.out, exist_ok=True)
    json.dump(rows, open(os.path.join(args.out, 'attribution.json'), 'w'), indent=1)
    counts = {}
    for r in rows:
        key = ('agrees' if r['agrees'] else 'differs' if r['agrees'] is False else 'unverified')
        counts.setdefault(r['kind'], {}).setdefault(key, 0)
        counts[r['kind']][key] += 1
    lines = ['# Asset attribution (generated by scripts/asset_attribution.py)', '',
             '| Kind | Agrees | Differs | Unverified |', '|---|---:|---:|---:|']
    for kind, c in counts.items():
        lines.append('| %s | %d | %d | %d |' % (kind, c.get('agrees', 0), c.get('differs', 0), c.get('unverified', 0)))
    lines += ['', '| Kind | Id | Declared | Found (basis) | Agrees | Assets found | License files |',
              '|---|---|---|---|---|---|---|']
    for r in rows:
        found = ', '.join(r['detected'] or r['package_manifest_licenses']) or '-'
        lines.append('| %s | %s | %s | %s (%s) | %s | %d/%d | %s |' % (
            r['kind'], r['id'], r['declared'] or '-', found, r['basis'],
            {True: 'yes', False: '**no**', None: '-'}[r['agrees']], r['assets_found'], r['assets'],
            '<br>'.join(r['license_files']) or '-'))
    open(os.path.join(args.out, 'attribution.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines[:8]))


if __name__ == '__main__':
    main()
