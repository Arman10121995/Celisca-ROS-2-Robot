#!/usr/bin/env python3
"""One record per algorithm method, from the catalog, the dispatch and the code (ROADMAP R7).

Each method id in ``algorithms.yaml`` or ``algorithm_dispatch.yaml`` gets a
record of:

- what a selection actually starts (dispatch kind and target);
- whether that package/executable is installed on this host;
- its mathematics (the R7 method subrecord, when there is one);
- the test files that name it;
- whether its name matches its mathematics.

Name/math checks come from two sources. Automatic: the catalog says one
implementation and the dispatch runs another, or two ids run the same
target. Curated: ``NAME_MATH`` below, written after reading each node's
code. Nothing here runs a method; installed and tested are facts about
files, not about a mission.

  python3 scripts/algorithm_method_records.py --yaml out.yaml --markdown out.md --ledger docs/status/platform-status.yaml
"""
import argparse
import os
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / 'src/robot_lab/robot_lab_registry/config/algorithms.yaml'
DISPATCH = ROOT / 'src/robot_lab_bringup/config/algorithm_dispatch.yaml'
SUBRECORDS = ROOT / 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'

# Read from each node's code (2026-11-14). Ids whose name, family or category
# does not describe what runs. "corrected": the catalog/dispatch now says what
# runs; "mismatch": the id stays (evidence and GUI selections use it) and this
# record says what it is; "not_a_method": a relay listed in an algorithm category.
NAME_MATH = {
    'imu_complementary_filter': ('mismatch', 'Id says complementary filter; it runs imu_tools\' Madgwick gradient-descent '
                                 'filter (imu_filter_madgwick_node); the imu_tools complementary filter package is not '
                                 'installed. Kept as the upstream Madgwick reference beside madgwick_filter; family '
                                 'corrected to gradient_descent.'),
    'dijkstra_planner': ('mismatch', 'Dispatches the same NavFn plugin as navfn_planner (use_astar false, i.e. '
                         'Dijkstra), so the two ids are one method and not separate comparison arms; '
                         'robot_lab_planning::DijkstraPlanner is built but not dispatched.'),
    'pd_motion_planner': ('mismatch', 'Catalogued as a local planner; it is a PD law on pose error toward a lookahead '
                          'point, with no obstacle term or trajectory search.'),
    'ekf_3d_estimator': ('mismatch', 'Id says 3D; the node runs a planar constant turn-rate and velocity (CTRV) EKF '
                         '(tracking_filters.CtrvEkf). state_estimation.EKF3DEstimator (per-axis scalar gains, not an '
                         'EKF) is unused legacy code.'),
    'motion_model_estimator': ('mismatch', 'Generic id; the node runs an interacting multiple model filter (constant '
                               'velocity + coordinated turn). The fixed-gain MotionModelEstimator class is unused '
                               'legacy code.'),
    'pose_graph_estimator': ('mismatch', 'Id says pose graph; the node runs a 2 s sliding-window least-squares smoother '
                             'over GNSS fixes with motion constraints (a small pose graph without loop closures). The '
                             'exponential-decay PoseGraphEstimator class is unused legacy code.'),
    'robot_localization_ekf': ('mismatch', 'Same robot_localization ekf_node as ekf_localization_node (one method, two '
                               'ids); the catalog executable was nav2_ekf_node, which is not a binary (corrected to '
                               'ekf_node).'),
    'ekf_localization_node': ('mismatch', 'Same robot_localization ekf_node as robot_localization_ekf (one method, two '
                              'ids).'),
    'navfn_planner': ('corrected', 'Was catalogued as a potential field; NavFn is a Dijkstra wavefront navigation '
                      'function (A* with use_astar) descended by gradient, i.e. grid search.'),
    'a_star_planner': ('corrected', 'Runs Nav2 Smac 2D (A* with a cost-aware heuristic); the catalog named the '
                       'undispatched robot_lab_planning::AStarPlanner.'),
    'pure_pursuit': ('corrected', 'Runs Nav2 Regulated Pure Pursuit; the catalog named it plain pure pursuit in '
                     'robot_lab_motion. The id regulated_pure_pursuit runs robot_lab\'s own copy of the same law.'),
    'simple_controller': ('corrected', 'Was catalogued as PID; it is differential-drive inverse kinematics plus wheel '
                          'odometry, with no feedback law.'),
    'noisy_controller': ('corrected', 'Was catalogued as PID; it is wheel odometry with Gaussian encoder noise '
                         '(sigma 0.005 rad), with no feedback law.'),
    'laser_scan_to_pointcloud': ('corrected', 'Its dispatch and launch fragment started pointcloud_to_laserscan_node, '
                                 'the reverse conversion; now laserscan_to_pointcloud_node. Runtime availability '
                                 'is determined from the current package index below.'),
    'costmap_2d_observation': ('corrected', 'The catalog named nav2_costmap_2d::ObservationLayer, which does not exist; '
                               'the Nav2 ObstacleLayer (observation buffers, ray-traced clearing) is what runs.'),
    'teb_local_planner': ('corrected', 'The catalog named nav2_teb_planner, which does not exist, and said integrated; '
                          'upstream is teb_local_planner (TebLocalPlannerROS), not installed. Now cataloged; '
                          'follow_the_gap is the R7.7 replacement.'),
    'mavros_offboard_controller': ('corrected', 'Retired zero-thrust AttitudeTarget stub whose node raises on start, '
                                   'yet its dispatch still started it. Now unavailable, status blocked.'),
    'imu_republisher': ('not_a_method', 'Frame/topic relay, not an estimator.'),
    'twist_relay': ('not_a_method', 'Message relay, not a control law.'),
}

# The mathematics of methods without an R7 subrecord, read from their code
# or upstream documentation (2026-11-14).
MATHEMATICS = {
    'bicycle_pure_pursuit': 'Kinematic bicycle at the fixed axle; curvature 2 y / L_d^2 to a lookahead point, '
                            'kappa = tan(delta) / L, steering and slew limits from the source',
    'bicycle_stanley': 'Stanley law delta = heading error + atan(k e / v) at the fixed axle, with a rear-steer '
                       'feedback adaptation and source steering limits',
    'bicycle_mpc': 'Linearised kinematic bicycle, steering-rate MPC over a 3-20 step horizon, warm-started, with a '
                   'solve deadline',
    'cleaning_controller': 'Boustrophedon-style coverage lanes over the cleanable grid cells, tracking cleaned cells',
    'map_coverage_controller': 'Frontier exploration of the occupancy grid, then wall following to confirm cells',
    'humanoid_standing_controller': 'Joint PD to a standing pose (legs Kp 120, Kd 4 N m/rad) plus an IMU ankle '
                                    'balance strategy and a latched tilt monitor',
    'humanoid_policy_controller': 'Learned ONNX policy: joint/IMU observation to joint position targets',
    'joint_effort_commander': 'Fixed-rate effort command relay to a forward_command_controller (no feedback law)',
    'native_holonomic_controller': 'Mecanum / four-wheel-steer inverse kinematics from body twist to wheel commands',
    'native_tracked_controller': 'Bounded command gate to Gazebo TrackController belt contact (skid steering)',
    'simple_controller': 'Differential-drive inverse kinematics v_l,r = (v -/+ w b/2) / r and wheel odometry',
    'noisy_controller': 'Differential-drive wheel odometry with Gaussian encoder noise (sigma 0.005 rad)',
    'twist_relay': 'Twist / TwistStamped relay',
    'mavros_offboard_controller': 'Retired: constant zero-thrust AttitudeTarget (no control law)',
    'dijkstra_planner': 'Dijkstra wavefront navigation function on the costmap (NavFn with use_astar false)',
    'pd_motion_planner': 'PD law on distance and heading error to a lookahead point on the path',
    'teb_local_planner': 'Timed elastic band: sparse nonlinear least squares over poses and time intervals '
                         '(upstream; not installed)',
    'mppi_local_planner': 'Retired placeholder: random velocities scored by goal distance (no rollout)',
    'amcl': 'Adaptive Monte Carlo localization: particle filter with KLD sampling and a likelihood-field laser model',
    'slam_toolbox': 'Pose-graph SLAM: correlative scan matching, loop closure, sparse pose-graph optimization',
    'rtabmap_localization': 'RGB-D appearance-based localization: bag-of-words loop closure and graph optimization',
    'hector_slam': 'Gauss-Newton scan-to-map matching on a multi-resolution grid (upstream; no Humble release)',
    'rgbd_slam_localization': 'Retired stand-in; RGB-D localization runs as rtabmap_localization',
    'odometry_motion_model': 'Sampling odometry motion model (rotation-translation-rotation, noise alpha1-alpha4)',
    'robot_localization_ekf': 'robot_localization EKF: 15-state omnidirectional model, odometry and IMU updates',
    'ekf_localization_node': 'robot_localization EKF: 15-state omnidirectional model, odometry and IMU updates',
    'kalman_filter_1d': 'Scalar Kalman filter on yaw rate fusing wheel odometry and IMU',
    'error_state_ekf': 'Retired: filtered noiseless truth; the error-state formulation runs as attitude_eskf',
    'imu_republisher': 'IMU frame/topic relay',
    'imu_complementary_filter': 'Madgwick gradient-descent attitude filter (imu_tools, upstream C++)',
    'camera_calibration': 'Zhang planar chessboard calibration (cv2.calibrateCamera) with sub-pixel corners',
    'costmap_2d_observation': 'Nav2 ObstacleLayer: marks sensor hits and ray-traces free space into the costmap',
    'depth_image_to_pointcloud': 'Pinhole back-projection p = d K^-1 [u v 1]^T',
    'feature_detection': 'Feature matching to a keyframe (Lowe ratio test), depth lift to 3D, PnP with RANSAC',
    'laser_scan_to_pointcloud': 'Polar-to-Cartesian projection of scan rays (laserscan_to_pointcloud_node)',
    'pointcloud_segmenter': 'Ground / non-ground split by height threshold in the upright target frame',
}

# Retired and placeholder ids already say why in the dispatch.
DISPATCH_KINDS = ('node', 'plugin', 'stack', 'unavailable')


def ament_prefixes():
    prefixes = [Path(p) for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':') if p]
    if not prefixes:
        prefixes = [Path('/opt/ros/humble')] + sorted((ROOT / 'install').glob('*'))
    return prefixes


def package_prefix(package, prefixes):
    for prefix in prefixes:
        if (prefix / 'share/ament_index/resource_index/packages' / package).is_file():
            return prefix
    return None


def plugin_package(plugin):
    return re.split(r'::|/', plugin)[0]


def installed(entry, catalog, prefixes):
    """(installed, detail) for what the dispatch starts."""
    if not entry:
        return None, 'no dispatch entry'
    if 'unavailable' in entry:
        return False, 'unavailable: ' + str(entry['unavailable'])
    if 'node' in entry:
        node = entry['node']
        prefix = package_prefix(node['package'], prefixes)
        if prefix is None:
            return False, 'package %s not installed' % node['package']
        if not (prefix / 'lib' / node['package'] / node['executable']).exists():
            return False, 'executable %s/%s not installed' % (node['package'], node['executable'])
        return True, '%s/%s' % (node['package'], node['executable'])
    if 'plugin' in entry:
        package = plugin_package(entry['plugin'])
        return package_prefix(package, prefixes) is not None, entry['plugin']
    package = ((catalog or {}).get('implementation') or {}).get('package')
    if package and package_prefix(package, prefixes) is None:
        return False, 'stack %s; package %s not installed' % (entry['stack'], package)
    return True, 'stack ' + str(entry['stack'])


def dispatch_target(entry):
    if not entry:
        return None
    for kind in DISPATCH_KINDS:
        if kind in entry:
            value = entry[kind]
            if kind == 'node':
                parameters = value.get('parameters') or {}
                return '%s %s/%s%s' % (kind, value['package'], value['executable'],
                                       ' planner=%s' % parameters['planner'] if 'planner' in parameters else '')
            return '%s %s' % (kind, value)
    return None


def test_texts(root=ROOT):
    texts = {}
    for path in sorted(root.glob('src/**/test/test_*.py')):
        if '/build/' in str(path) or '/install/' in str(path):
            continue
        texts[str(path.relative_to(root))] = path.read_text(errors='replace')
    return texts


def tests_naming(method, entry, texts):
    names = {method}
    parameters = ((entry or {}).get('node') or {}).get('parameters') or {}
    if 'planner' in parameters:
        names.add(str(parameters['planner']))
    patterns = [re.compile(r'''['"]%s['"]''' % re.escape(n)) for n in names]
    return [path for path, text in texts.items() if any(p.search(text) for p in patterns)]


def load(catalog=CATALOG, dispatch=DISPATCH, subrecords=SUBRECORDS):
    entries = {e['id']: e for e in yaml.safe_load(Path(catalog).read_text())}
    routes = {}
    for category, methods in yaml.safe_load(Path(dispatch).read_text())['algorithms'].items():
        for method, entry in methods.items():
            routes[method] = (category, entry)
    maths = {}
    for category, methods in (yaml.safe_load(Path(subrecords).read_text())['method_subrecords'] or {}).items():
        for method, record in methods.items():
            maths[method] = record
    return entries, routes, maths


def records(entries, routes, maths, texts, prefixes):
    out = []
    targets = {}
    for method, (category, entry) in routes.items():
        if entry and 'unavailable' not in entry:
            targets.setdefault((category, yaml.safe_dump(entry, sort_keys=True)), []).append(method)
    for method in sorted(set(entries) | set(routes), key=lambda m: ((entries.get(m) or {}).get('category')
                                                                     or routes[m][0], m)):
        catalog = entries.get(method)
        category, entry = routes.get(method, ((catalog or {}).get('category'), None))
        target = dispatch_target(entry)
        ok, detail = installed(entry, catalog, prefixes)
        notes = []
        key = (category, yaml.safe_dump(entry, sort_keys=True)) if entry else None
        shared = [m for m in targets.get(key, []) if m != method]
        if shared:
            notes.append('same dispatch entry as ' + ', '.join(shared))
        state, note = NAME_MATH.get(method, ('mismatch' if shared else 'consistent', None))
        if note:
            notes.insert(0, note)
        math = maths.get(method, {})
        out.append(dict(
            id=method, category=category, name=(catalog or {}).get('name'),
            family=(catalog or {}).get('family'), catalog_status=(catalog or {}).get('status', 'dispatch only'),
            dispatch=target, installed=ok, installed_detail=detail,
            mathematics=math.get('mathematical_foundation') or MATHEMATICS.get(method), subrecord=bool(math),
            mathematics_source='subrecord' if math else ('curated' if method in MATHEMATICS else None),
            tests=tests_naming(method, entry, texts),
            name_math=state,
            notes=notes))
    return out


def summary(rows):
    return dict(methods=len(rows), installed=sum(r['installed'] is True for r in rows),
                not_installed=sum(r['installed'] is False and not (r['dispatch'] or '').startswith('unavailable')
                                  for r in rows),
                unavailable=sum((r['dispatch'] or '').startswith('unavailable') for r in rows),
                with_subrecord=sum(r['subrecord'] for r in rows), with_tests=sum(bool(r['tests']) for r in rows),
                **{'name_math_' + s: sum(r['name_math'] == s for r in rows)
                   for s in ('consistent', 'corrected', 'mismatch', 'not_a_method')})


def ledger_line(row):
    state = 'installed' if row['installed'] else ('unavailable' if (row['dispatch'] or '').startswith('unavailable')
                                                  else 'not installed')
    parts = ['%s; %s; %s' % (row['category'], state, row['dispatch'] or 'no dispatch'),
             'tests %d' % len(row['tests']), 'math: ' + (row['mathematics'] or 'no subrecord')]
    if row['name_math'] != 'consistent':
        parts.append('name/math %s: %s' % (row['name_math'], ' '.join(row['notes'])))
    return '; '.join(parts)


def write_ledger(path, rows, stamp):
    """Replace the R7 group's method_records block (by line span) in the ledger."""
    path = Path(path)
    lines = path.read_text().split('\n')
    start = next(i for i, l in enumerate(lines) if l == '  R7:')
    end = start + 1
    while end < len(lines) and (not lines[end].strip() or lines[end].startswith('   ')):
        end += 1
    block = lines[start:end]
    old = [i for i, l in enumerate(block) if l.startswith('    method_records:')]
    if old:
        i = old[0]
        k = i + 1
        while k < len(block) and block[k].startswith('      '):
            k += 1
        del block[i:k]
    value = dict(generated=stamp, generator='scripts/algorithm_method_records.py',
                 summary=summary(rows), records={r['id']: ledger_line(r) for r in rows})
    dumped = yaml.safe_dump({'method_records': value}, width=110, sort_keys=False, allow_unicode=True)
    insert = next(i for i, l in enumerate(block) if l.startswith('    tasks:'))
    block[insert:insert] = ['    ' + l for l in dumped.rstrip('\n').split('\n')]
    lines[start:end] = block
    text = '\n'.join(lines)
    yaml.safe_load(text)
    path.write_text(text)


def markdown(rows, stamp):
    lines = ['# Algorithm method records', '',
             'Generated %s by `scripts/algorithm_method_records.py` from the catalog, the dispatch, the R7 method '
             'subrecords and the test files. Installed and tested are facts about files on this host; they do not '
             'qualify a method.' % stamp, '',
             'Summary: ' + ', '.join('%s %d' % kv for kv in summary(rows).items()) + '.', '']
    category = None
    for row in rows:
        if row['category'] != category:
            category = row['category']
            lines += ['', '## ' + category, '', '| Method | Dispatch | Installed | Tests | Mathematics | Name/math |',
                      '|---|---|---|---|---|---|']
        lines.append('| `%s` | %s | %s | %d | %s | %s |' % (
            row['id'], (row['dispatch'] or '-').replace('|', '/'),
            'yes' if row['installed'] else 'no', len(row['tests']), row['mathematics'] or '-',
            row['name_math'] if row['name_math'] == 'consistent' else
            '**%s**: %s' % (row['name_math'], ' '.join(row['notes']).replace('|', '/'))))
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--yaml', default='')
    parser.add_argument('--markdown', default='')
    parser.add_argument('--ledger', default='')
    parser.add_argument('--stamp', default='')
    options = parser.parse_args()
    import time
    stamp = options.stamp or time.strftime('%Y-%m-%d')
    rows = records(*load(), test_texts(), ament_prefixes())
    if options.yaml:
        Path(options.yaml).write_text(yaml.safe_dump(dict(generated=stamp, summary=summary(rows), records=rows),
                                                     sort_keys=False, width=110, allow_unicode=True))
    if options.markdown:
        Path(options.markdown).write_text(markdown(rows, stamp))
    if options.ledger:
        write_ledger(options.ledger, rows, stamp)
    print(summary(rows))


if __name__ == '__main__':
    main()
