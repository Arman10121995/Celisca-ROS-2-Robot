#!/usr/bin/env python3
"""Resolve the licenses of the workspace's declared dependencies (R9.1).

Every dependency named in a workspace ``package.xml`` (depend, build,
exec and test depends) or a ``setup.py`` ``install_requires`` is resolved
from its own installed metadata, never from a name list:

* workspace packages: their ``package.xml`` ``<license>``;
* ROS packages: ``$AMENT_PREFIX_PATH/share/<name>/package.xml``;
* rosdep system keys (``python3-numpy``, ``libfoo-dev``): the Debian
  package's ``/usr/share/doc/<package>/copyright`` (``License:`` fields);
* Python distributions: ``importlib.metadata`` (License-Expression,
  License, or ``License ::`` classifiers).

Anything not found stays ``unresolved`` with the places looked at.

  python3 scripts/license_inventory.py --json out.json --markdown out.md
"""
import argparse
import collections
import glob
import json
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEPEND_TAGS = ('depend', 'build_depend', 'build_export_depend', 'exec_depend',
               'test_depend', 'buildtool_depend')


def workspace_packages():
    packages = {}
    for path in sorted(ROOT.glob('src/**/package.xml')):
        if any(part in ('build', 'install', 'log') for part in path.parts):
            continue
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            continue
        name = (root.findtext('name') or '').strip()
        if name:
            packages[name] = (path, root)
    return packages


def declared_dependencies(packages):
    """{dependency: {'kinds': set, 'used_by': set}} from package.xml and setup.py."""
    found = collections.defaultdict(lambda: {'kinds': set(), 'used_by': set()})
    for name, (path, root) in packages.items():
        for tag in DEPEND_TAGS:
            for element in root.findall(tag):
                dep = (element.text or '').strip()
                if dep:
                    found[dep]['kinds'].add(tag)
                    found[dep]['used_by'].add(name)
        setup = path.parent / 'setup.py'
        if setup.is_file():
            text = setup.read_text(errors='replace')
            match = re.search(r'install_requires\s*=\s*\[(.*?)\]', text, re.S)
            for item in re.findall(r"['\"]([^'\"]+)['\"]", match.group(1) if match else ''):
                dep = re.split(r'[<>=!~ ;\[]', item, maxsplit=1)[0].strip()
                if dep and dep != 'setuptools':
                    found['pypi:' + dep]['kinds'].add('install_requires')
                    found['pypi:' + dep]['used_by'].add(name)
    return found


def ros_package_license(name):
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '/opt/ros/humble').split(os.pathsep):
        manifest = Path(prefix) / 'share' / name / 'package.xml'
        if manifest.is_file():
            try:
                licenses = [(e.text or '').strip() for e in ET.parse(manifest).getroot().findall('license')]
            except ET.ParseError:
                continue
            licenses = [l for l in licenses if l]
            if licenses:
                return ' AND '.join(licenses), str(manifest)
    return None, None


DEBIAN_ALIASES = {
    # rosdep keys whose Debian package is named differently.
    'python3-pytest': 'python3-pytest', 'eigen': 'libeigen3-dev', 'boost': 'libboost-dev',
    'opencv': 'libopencv-dev', 'pcl': 'libpcl-dev', 'yaml-cpp': 'libyaml-cpp-dev',
    'ament_python': None,
}


def debian_license(key):
    candidates = [DEBIAN_ALIASES.get(key, key), key]
    if key.startswith('python3-'):
        candidates.append(key)
    for package in [c for c in candidates if c]:
        copyright_file = Path('/usr/share/doc') / package / 'copyright'
        if copyright_file.is_file():
            text = copyright_file.read_text(errors='replace')
            licenses = re.findall(r'^License:\s*(\S[^\n]*)', text, re.M)
            if licenses:
                counts = collections.Counter(l.strip() for l in licenses)
                return ' AND '.join(l for l, _ in counts.most_common(3)), str(copyright_file)
    return None, None


def python_license(name):
    try:
        from importlib import metadata
        meta = metadata.metadata(name)
    except Exception:
        return None, None
    expression = meta.get('License-Expression')
    if expression:
        return expression, 'python metadata ' + name
    classifiers = [c.split('::')[-1].strip() for c in meta.get_all('Classifier') or []
                   if c.startswith('License ::')]
    if classifiers:
        return ' AND '.join(sorted(set(classifiers))), 'python metadata ' + name
    text = (meta.get('License') or '').strip()
    if text and text.upper() != 'UNKNOWN' and len(text) < 120:
        return text, 'python metadata ' + name
    return None, None


# rosdep python keys whose distribution name differs.
PYTHON_DISTRIBUTIONS = {'pil': 'Pillow', 'serial': 'pyserial', 'trimesh-pip': 'trimesh',
                        'rtree': 'Rtree', 'yaml': 'PyYAML'}
# ROS 1 build/run keys declared only by vendored upstream description
# manifests (Unitree); nothing in this ROS 2 workspace builds or runs them.
ROS1_ONLY = {'catkin', 'roscpp', 'roslaunch', 'rviz', 'gazebo', 'gazebo_ros', 'controller_manager_msgs'}


def resolve(dep, packages):
    if dep in packages:
        path, root = packages[dep]
        licenses = [(e.text or '').strip() for e in root.findall('license') if (e.text or '').strip()]
        return dict(source='workspace', license=' AND '.join(licenses) or None, evidence=str(path.relative_to(ROOT)))
    if dep.startswith('pypi:'):
        license_, evidence = python_license(dep[5:])
        return dict(source='python', license=license_, evidence=evidence)
    license_, evidence = ros_package_license(dep)
    if license_:
        return dict(source='ros', license=license_, evidence=evidence)
    license_, evidence = debian_license(dep)
    if license_:
        return dict(source='debian', license=license_, evidence=evidence)
    if dep.startswith('python3-'):
        key = dep[len('python3-'):]
        license_, evidence = python_license(PYTHON_DISTRIBUTIONS.get(key, key))
        if license_:
            return dict(source='python', license=license_, evidence=evidence)
    if dep == 'python3':
        for path in sorted(glob.glob('/usr/lib/python3*/LICENSE.txt')):
            if 'PYTHON SOFTWARE FOUNDATION LICENSE VERSION 2' in Path(path).read_text(errors='replace'):
                return dict(source='debian', license='PSF-2.0', evidence=path)
    if dep in ROS1_ONLY:
        return dict(source='ros1-vendored', license=None,
                    evidence='ROS 1 key in a vendored description manifest; not built or run here')
    return dict(source='unresolved', license=None,
                evidence='not installed: no package.xml under AMENT_PREFIX_PATH, no /usr/share/doc '
                         'copyright, no python metadata')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json')
    parser.add_argument('--markdown')
    args = parser.parse_args()
    packages = workspace_packages()
    declared = declared_dependencies(packages)
    rows = []
    for dep in sorted(declared):
        info = resolve(dep, packages)
        rows.append(dict(name=dep, kinds=sorted(declared[dep]['kinds']),
                         used_by=sorted(declared[dep]['used_by']), **info))
    resolved = [r for r in rows if r['license']]
    unused = [r for r in rows if r['source'] == 'ros1-vendored']
    summary = dict(dependencies=len(rows), resolved=len(resolved),
                   unresolved=len(rows) - len(resolved) - len(unused),
                   ros1_vendored_unused=len(unused),
                   by_source=dict(collections.Counter(r['source'] for r in rows)),
                   copyleft=[r['name'] for r in resolved
                             if re.search(r'\bA?GPL|LGPL|MPL', r['license'] or '')],
                   workspace_packages=len(packages))
    report = dict(summary=summary, dependencies=rows)
    print(json.dumps(summary, indent=1))
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=1) + '\n')
    if args.markdown:
        lines = ['| Dependency | License | Source | Used by |', '|---|---|---|---|']
        for r in rows:
            lines.append('| %s | %s | %s | %s |' % (r['name'], r['license'] or '**unresolved**', r['source'],
                                                   ', '.join(r['used_by'][:3]) + (' ...' if len(r['used_by']) > 3 else '')))
        Path(args.markdown).write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
