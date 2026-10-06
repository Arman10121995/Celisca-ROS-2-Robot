"""Expose imported robot modes only from recorded, matching runtime screens.

These screens have a named robot/map/backend scope. They do not complete the
whole robot roadmap, vendor firmware qualification or hardware acceptance.
"""
import hashlib
import json
import math
from pathlib import Path

from .qualification import navigation_acceptance


def _finite(value, limit):
    return isinstance(value, (int, float)) and math.isfinite(value) and 0 <= value < limit


def _workflow_passes(report, mode):
    phases = report.get('phases', {})
    for name in ('forward', 'reverse', 'turn', 'turn_reverse', 'publisher_loss'):
        phase = phases.get(name, {})
        elapsed = phase.get('sim_s', 0)
        if not phase.get('completed') or not _finite(elapsed, 3.5) or elapsed < 2.95:
            return False
        if name in ('forward', 'reverse') and not phase.get('distance_m', 0) > .25:
            return False
        if name == 'turn' and not phase.get('yaw_change_rad', 0) > .7:
            return False
        if name == 'turn_reverse' and not phase.get('yaw_change_rad', 0) < -.7:
            return False
        if name == 'publisher_loss' and not all(_finite(phase.get(key), limit) for key, limit in
                [('distance_m', .15), ('tail_speed_mps', .02), ('tail_yaw_speed_rps', .02)]):
            return False
    if not _finite(report.get('localization_error_m'), .25):
        return False
    checks = report.get('checks', {})
    if not checks or not all(value is True for value in checks.values()):
        return False
    if mode == 'slam':
        maps = report.get('map_updates', [])
        if (len(maps) < 2 or maps[-1].get('known', 0) <= 100
                or maps[-1].get('occupied', 0) <= 10 or len(report.get('accepted_slam_poses', [])) < 3):
            return False
    elif mode == '3d_slam':
        clouds = report.get('cloud_message_points', [])
        geometry = report.get('cloud_geometry') or {}
        if len(clouds) < 2 or max(clouds) <= 100:
            return False
        try:
            extent = [hi-lo for lo, hi in zip(geometry['min_xyz'], geometry['max_xyz'])]
            if len(extent) != 3 or not all(math.isfinite(x) for x in extent) or extent[2] <= .3:
                return False
        except (KeyError, TypeError):
            return False
    return True


def _navigation_passes(measured, world):
    route = measured.get('route_acceptance', {})
    clearance = route.get('min_swept_clearance_m')
    direct = route.get('direct_route_clearance_m')
    return (navigation_acceptance(measured, .15, 5)['passed']
            and measured.get('settled_sim_second') is True
            and measured.get('truth_samples', 0) >= 20
            and route.get('passed') is True
            and isinstance(clearance, (int, float)) and math.isfinite(clearance) and clearance >= .02
            and route.get('robot_radius_m', 0) >= .185
            and (world != 'nav_obstacle' or
                 isinstance(direct, (int, float)) and math.isfinite(direct) and direct < .02))


def apply_recorded_modes(profile, source_revision, support, repository):
    """Return a profile copy; reject changed control contracts or false reports.

    Evidence is immutable JSON in the repository, linked by SHA-256. Extra
    maps remain operator experiments; named screens are explicitly retained.
    """
    result = dict(profile)
    if not support:
        return result
    if (support.get('source_revision') != source_revision
            or support.get('drive') != profile.get('drive')
            or support.get('sensor_config') != profile.get('sensor_config')):
        raise ValueError('Recorded runtime support does not match the current source/control configuration')
    for contract in support.get('configuration_files', []):
        path = Path(repository)/contract['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != contract['sha256']:
            raise ValueError('Recorded runtime configuration differs: '+str(path))
    matrix = {backend: ['display'] for backend in profile['supported_simulators']}
    scopes = []
    for cell in support.get('screens', []):
        backend, mode = cell['backend'], cell['mode']
        if backend not in matrix or mode not in ('loc', 'slam', '3d_slam', 'nav'):
            raise ValueError('Unsupported runtime screen cell')
        maps = set()
        for proof in cell['reports']:
            path = Path(repository)/proof['path']
            data = path.read_bytes()
            if hashlib.sha256(data).hexdigest() != proof['sha256']:
                raise ValueError('Runtime evidence checksum differs: '+str(path))
            report = json.loads(data)
            if (report.get('robot') != profile['name'] or report.get('backend') != backend
                    or report.get('mode') != mode or report.get('passed') is not True
                    or report.get('launch_returncode') != 0):
                raise ValueError('Runtime evidence identity/outcome differs: '+str(path))
            measured = report.get('workflow', {})
            accepted = _navigation_passes(measured, report['map']) if mode == 'nav' \
                       else _workflow_passes(measured, mode)
            saved = report.get('save_map', {})
            if mode == 'slam':
                accepted = accepted and saved.get('image_bytes', 0) > 100 and bool(saved.get('yaml'))
            elif mode == '3d_slam':
                accepted = accepted and saved.get('database_nodes', 0) > 1 and saved.get('pointcloud_points', 0) > 100
            if not accepted:
                raise ValueError('Runtime measurements do not pass acceptance: '+str(path))
            maps.add(report['map'])
        if not maps or (mode == 'nav' and not {'nav_empty', 'nav_obstacle'} <= maps):
            raise ValueError('Navigation needs both clear and obstacle route evidence')
        matrix[backend].append(mode)
        scopes.append(dict(backend=backend, mode=mode, maps=sorted(maps),
                           reports=[proof['path'] for proof in cell['reports']]))
    result['supported_modes_by_simulator'] = matrix
    result['supported_modes'] = [mode for mode in ('display', 'loc', 'slam', '3d_slam', 'nav')
                                 if any(mode in modes for modes in matrix.values())]
    result['runtime_screens'] = scopes
    result['defaults'] = dict(result.get('defaults', {}), global_planning='a_star_planner',
                              local_planning='pure_pursuit')
    return result


def apply_recorded_panda_hand(profile, source_revision, support, repository):
    """Enable the Panda hand from measured contact, lift and interruption data."""
    result = dict(profile)
    if not support:
        return result
    if (profile.get('arm_control') != 'panda' or profile.get('supported_simulators') != ['mujoco']
            or support.get('source_revision') != source_revision
            or hashlib.sha256(Path(profile['native_mjcf']).read_bytes()).hexdigest()
               != support.get('native_xml_sha256')):
        raise ValueError('Panda hand evidence differs from the native model')
    for contract in support['configuration_files']:
        path = Path(repository)/contract['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != contract['sha256']:
            raise ValueError('Panda hand controller differs from the measured source: '+str(path))
    proof = support['report']
    data = (Path(repository)/proof['path']).read_bytes()
    if hashlib.sha256(data).hexdigest() != proof['sha256']:
        raise ValueError('Panda hand evidence checksum differs')
    report = json.loads(data)
    close = report['close']['status']
    lifted = report['lift_and_hold']['objects'][-1]['position']
    released = report['release']['objects'][-1]['position']
    initial = report['initial_object']
    checks = [report.get('passed') is True, report.get('launch_returncode') == 0,
        'robot_model:=menagerie_franka_emika_panda' in report['command'],
        'simulator:=mujoco' in report['command'], 'map_name:=nav_empty' in report['command'],
        'grasp_fixture:=true' in report['command'],
        .015 < close['opening_m'] < .04,
        abs(close['positions'][0]-close['positions'][1]) < .001,
        0 < close['actuator_effort_per_finger_n'] <= close['force_limit_per_finger_n']+1e-6,
        'contact stall' in close['status'],
        set(close['contacts']) == {'finger_joint1', 'finger_joint2'},
        all(sum(c['normal_force_n'] for c in contacts if c['body'] == 'manipulation_object') > .05
            for contacts in close['contacts'].values()),
        all(math.isfinite(x) for point in (initial, lifted, released) for x in point),
        lifted[2]-initial[2] > .04, released[2] < lifted[2]-.03,
        report['release']['status']['opening_m'] > .075,
        .8 <= report['heartbeat_loss_wall_s'] < 1.3,
        'canceled' in report['cancel']['status']['status'],
        'stopped' in report['stop']['status']['status'],
        'heartbeat lost' in report['publisher_loss']['status']['status'],
        all(item['accepted'] is False for item in report['rejected_commands']),
        len(report['rejected_commands']) == 3,
        math.dist(report['gui_reset']['object']['position'], initial) < .004,
        report['gui_reset']['gripper_status']['opening_m'] > .075]
    if not all(checks):
        raise ValueError('Panda hand measurements do not pass contact/lift/interruption acceptance')
    result['hand_control'] = 'panda'
    result['hand_runtime_screen'] = dict(backend='mujoco', map='nav_empty', report=proof['path'])
    return result
