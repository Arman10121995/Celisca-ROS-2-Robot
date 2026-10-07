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
            or support.get('sensor_config') != profile.get('sensor_config')
            or ('spawn_by_map' in support and
                support['spawn_by_map'] != profile.get('spawn_by_map'))
            or ('gazebo_max_physics_step_s' in support and
                support['gazebo_max_physics_step_s'] != profile.get('gazebo_max_physics_step_s'))):
        raise ValueError('Recorded runtime support does not match the current source/control configuration')
    if 'executed_urdf_sha256' in support:
        model = Path(profile['xacro'])
        if hashlib.sha256(model.read_bytes()).hexdigest() != support['executed_urdf_sha256']:
            raise ValueError('Recorded executed robot geometry/inertia differs: '+str(model))
    for generated in support.get('generated_model_files', []):
        path = Path(profile['xacro']).parent/generated['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != generated['sha256']:
            raise ValueError('Recorded generated robot controller differs: '+str(path))
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
        contracts = list(support.get('configuration_files_by_simulator', {}).get(backend, []))
        contracts += support.get('configuration_files_by_mode', {}).get(backend, {}).get(mode, [])
        for contract in contracts:
            path = Path(repository)/contract['path']
            if hashlib.sha256(path.read_bytes()).hexdigest() != contract['sha256']:
                raise ValueError('Recorded backend runtime configuration differs: '+str(path))
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
    result['default_algorithms'] = dict(result.get('default_algorithms', {}),
        global_planning='a_star_planner', local_planning='pure_pursuit')
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


def panda_planning_acceptance(report):
    """Recheck physical Cartesian, rejection and interruption measurements.

    This is the bounded native Panda / MuJoCo / static nav_empty screen. It
    does not qualify payload planning, Servo, other robots or other backends.
    Malformed or incomplete reports fail closed, even when labelled PASS.
    """
    def bounded(value, lo, hi):
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value) and lo <= value < hi)

    def vector(value, size):
        return (isinstance(value, list) and len(value) == size
                and all(bounded(v, -1e6, 1e6) for v in value))

    try:
        if not (report['robot'] == 'menagerie_franka_emika_panda'
                and report['backend'] == 'mujoco' and report['map'] == 'nav_empty'
                and report['mode'] == 'display' and report['launch_returncode'] == 0
                and report['passed'] is True and report['planner_clean_exit'] is True
                and report['joint_samples'] > 100 and report['live_monitor']['clock_samples'] > 5
                and report['scene']['ready'] is True and report['scene']['geometry_count'] >= 9
                and bounded(report['measured_fk_error_m'], 0, .003)):
            return False
        checks = report['checks']
        required = ('selected_world_scene_acknowledged', 'collision_and_unreachable_rejection',
                    'two_cartesian_targets_measured', 'real_joint_change_invalidates_plan',
                    'real_hand_change_invalidates_plan', 'invalid_offset_no_motion',
                    'cancel', 'stop', 'heartbeat_loss', 'reset_invalidates_plan_and_restores_home',
                    'owned_cleanup')
        if not all(checks.get(key) is True for key in required):
            return False
        motions = report['motions']
        if len(motions) != 2:
            return False
        for motion in motions:
            actual, target = motion['actual'], motion['target_position']
            a, b = actual['quaternion_wxyz'], motion['target_quaternion_wxyz']
            if not (vector(actual['position'], 3) and vector(target, 3)
                    and vector(a, 4) and vector(b, 4)
                    and abs(sum(x*x for x in a)-1) < 1e-4
                    and abs(sum(x*x for x in b)-1) < 1e-4):
                return False
            angle = math.degrees(2*math.acos(min(1., abs(sum(x*y for x, y in zip(a, b))))))
            if not (math.dist(actual['position'], target) < .02 and angle < 5
                    and bounded(motion['position_error_m'], 0, .02)
                    and bounded(motion['orientation_error_deg'], 0, 5)
                    and bounded(actual['age_sim_s'], -.001, .08)
                    and bounded(motion['physical_displacement_m'], .02, .3)
                    and bounded(motion['duration_sim_s'], .05, 15.001)
                    and motion['joint_samples'] > 20
                    and motion['terminal_status']['busy'] is False
                    and motion['terminal_status']['contact_blocked'] is False):
                return False
        floor, self_contact = report['floor_collision'], report['self_collision']
        if not (floor['valid'] is False and self_contact['valid'] is False
                and any(c['second'].startswith('robot_lab_world_') for c in floor['contacts'])
                and self_contact['native_contacts']
                and any(c['first'].startswith('native_body_') and c['second'].startswith('native_body_')
                        for c in self_contact['planner_contacts'])):
            return False
        negatives = report['planning_negatives']
        if {item['case'] for item in negatives} != {'floor_blocked', 'unreachable'}:
            return False
        if not all(item['error_code'] != 1 and item['waypoints'] == 0
                   and bounded(item['physical_drift_rad'], 0, .003) for item in negatives):
            return False
        for name, reason in [('cancel', 'canceled'), ('stop', 'stopped'),
                             ('heartbeat_loss', 'heartbeat lost')]:
            item = report[name]
            if not (bounded(item['drift_rad'], 0, .02)
                    and bounded(item['max_velocity_rad_s'], 0, .03)
                    and bounded(item['abort_wall_s'], .65 if name == 'heartbeat_loss' else 0, 1.3)
                    and item['status']['busy'] is False and reason in item['status']['status']):
                return False
        for name, key, lower in [('joint_invalidation', 'change_rad', .01),
                                 ('hand_invalidation', 'change_m', .005)]:
            item = report[name]
            if not (bounded(item[key], lower, .1) and item['execute_disabled'] is True
                    and item['plan_removed'] is True):
                return False
        reset = report['reset']
        return (bounded(report['invalid_offset_drift_rad'], 0, .003)
                and reset['plan_removed'] is True and reset['execute_disabled'] is True
                and reset['after_sim'] > reset['before_sim']
                and bounded(reset['home_error_rad'], 0, .015)
                and vector(reset['tcp']['position'], 3) and vector(report['initial_tcp']['position'], 3)
                and math.dist(reset['tcp']['position'], report['initial_tcp']['position']) < .004)
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
        return False


def apply_recorded_panda_planning(profile, source_revision, support, repository):
    """Expose Plan/Execute only with a matching native model and physical proof."""
    result = dict(profile)
    if not support:
        return result
    if (profile.get('name') != 'menagerie_franka_emika_panda'
            or profile.get('arm_control') != 'panda'
            or profile.get('supported_simulators') != ['mujoco']
            or support.get('source_revision') != source_revision
            or hashlib.sha256(Path(profile['native_mjcf']).read_bytes()).hexdigest()
                != support.get('native_xml_sha256')):
        raise ValueError('Panda planning evidence differs from the native model')
    for resource in support.get('native_resource_files', []):
        path = Path(profile['native_mjcf']).parent/resource['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != resource['sha256']:
            raise ValueError('Panda planning native resource differs: '+str(path))
    for contract in support['configuration_files']:
        path = Path(repository)/contract['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != contract['sha256']:
            raise ValueError('Panda planning configuration differs from measured source: '+str(path))
    proof = support['report']
    data = (Path(repository)/proof['path']).read_bytes()
    if hashlib.sha256(data).hexdigest() != proof['sha256']:
        raise ValueError('Panda planning evidence checksum differs')
    report = json.loads(data)
    expected = ('robot_model:=menagerie_franka_emika_panda', 'simulator:=mujoco',
                'map_name:=nav_empty', 'mode:=display', 'arm_control:=panda', 'arm_planning:=moveit')
    if not (panda_planning_acceptance(report) and all(token in report['command'].split() for token in expected)
            and 'grasp_fixture:=true' not in report['command'].split()
            and report['scene']['world_sha256'] == support['world_sha256']):
        raise ValueError('Panda planning measurements do not pass physical acceptance')
    result['arm_planning'] = 'moveit'
    result['arm_planning_runtime_screen'] = dict(backend='mujoco', map='nav_empty', report=proof['path'],
                                               scope='Static world; no grasp fixture or attached payload')
    return result
