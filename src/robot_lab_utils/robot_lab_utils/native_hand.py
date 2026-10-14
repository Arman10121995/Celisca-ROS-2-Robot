"""Reviewed native hand contracts, without inventing motor transmissions.

DEX-EE uses its authored mujoco.pid position plugin. Robotiq and UMI use
their authored coupled position tendons. Pose templates are experimental
operator targets, not grasp policies or a grasp-success certificate.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .native_actuation import position_channels


HAND_MODELS = {
    'menagerie_shadow_dexee': 'shadow_dexee',
    'menagerie_robotiq_2f85': 'robotiq_2f85',
    'menagerie_robotiq_2f85_v4': 'robotiq_2f85_v4',
    'menagerie_umi_gripper': 'umi_gripper',
}


def hand_profile(name, profile):
    """Annotate installed candidates; no compilation, download or promotion."""
    entry = HAND_MODELS.get(name)
    if (entry is None or profile.get('source_id') != 'mujoco_menagerie'
            or profile.get('source_entry') != entry or not profile.get('native_mjcf')):
        return profile
    result = dict(profile)
    result['native_hand'] = dict(adapter=entry, implementation_state='experimental',
        validation_state='pending', backend='mujoco',
        actuation='authored PID position plugin' if entry == 'shadow_dexee' else 'authored coupled position tendon')
    if not result.get('native_articulation'):
        result['native_articulation'] = dict(implementation_state='experimental', validation_state='pending')
    result['features'] = list(dict.fromkeys([*result.get('features', []), 'native_hand_controls_experimental']))
    result['notes'] = str(result.get('notes', '')) + ' Experimental MuJoCo Hand controls: authored PID/tendon '
    result['notes'] += 'poses and measured contact/effort stops. Enable native joint controls before Run; object grasp qualification pending.'
    return result


def _name(mj, model, kind, index):
    return mj.mj_id2name(model, kind, int(index)) or ''


def hand_contract(model, native_source):
    """Validate source-specific native channels against the executed model.

    Returns None for other robots. A recognized but altered/incompatible hand
    raises instead of guessing. World bodies may have been appended already.
    """
    import mujoco as mj
    source = Path(native_source)
    entry = source.parent.name
    if entry not in HAND_MODELS.values():
        return None
    root = ET.parse(source).getroot()
    if root.findall('.//freejoint') or root.findall(".//joint[@type='free']"):
        raise ValueError('Native hand requires an authored fixed mount')
    reference = mj.MjModel.from_xml_path(str(source))
    for name, count in (
            ('actuator_trntype', reference.nu), ('actuator_trnid', reference.nu),
            ('actuator_gear', reference.nu), ('actuator_gainprm', reference.nu),
            ('actuator_biasprm', reference.nu), ('actuator_ctrlrange', reference.nu),
            ('actuator_forcerange', reference.nu), ('actuator_ctrllimited', reference.nu),
            ('actuator_forcelimited', reference.nu), ('actuator_dyntype', reference.nu),
            ('actuator_gaintype', reference.nu), ('actuator_biastype', reference.nu),
            ('actuator_plugin', reference.nu), ('actuator_actnum', reference.nu),
            ('jnt_type', reference.njnt), ('jnt_range', reference.njnt),
            ('tendon_adr', reference.ntendon), ('tendon_num', reference.ntendon),
            ('wrap_type', reference.nwrap), ('wrap_objid', reference.nwrap), ('wrap_prm', reference.nwrap),
            ('eq_type', reference.neq), ('eq_obj1id', reference.neq),
            ('eq_obj2id', reference.neq),
            ('plugin_attr', len(reference.plugin_attr))):
        if not np.array_equal(getattr(model, name)[:count], getattr(reference, name)):
            raise ValueError('Executed hand differs from authored motor/coupling contract: '+name)
    for index in range(reference.neq):
        # Global spawn transforms recompile local connect/weld anchors with
        # round-off (~4e-16 m at a 7 m translation). This 1e-12 coordinate
        # tolerance does not apply to joint/tendon coupling coefficients.
        frames = int(reference.eq_type[index]) in (int(mj.mjtEq.mjEQ_CONNECT), int(mj.mjtEq.mjEQ_WELD))
        matches = np.allclose(model.eq_data[index], reference.eq_data[index], rtol=0, atol=1e-12) if frames else (
            np.array_equal(model.eq_data[index], reference.eq_data[index]))
        if not matches:
            raise ValueError('Executed hand differs from authored equality contract')
    if root.findall('include') or np.any(model.jnt_type == int(mj.mjtJoint.mjJNT_FREE)):
        # Environment free objects are allowed below, but never a free hand.
        robot_joint_names = {element.get('name') for element in root.findall('.//worldbody//joint')}
        for joint in range(model.njnt):
            if (int(model.jnt_type[joint]) == int(mj.mjtJoint.mjJNT_FREE)
                    and _name(mj, model, mj.mjtObj.mjOBJ_JOINT, joint) in robot_joint_names):
                raise ValueError('Native hand requires a fixed authored mount')
        if root.findall('include'):
            raise ValueError('Hand source include contract needs a reviewed adapter')
    channels = []
    groups = []
    if entry == 'shadow_dexee':
        instances = {item.get('name'): {c.get('key'): float(c.get('value')) for c in item.findall('config')}
            for item in root.findall("extension/plugin[@plugin='mujoco.pid']/instance")}
        actuators = {item.get('name'): item for item in root.findall('actuator/plugin')}
        if len(instances) != 4 or len(actuators) != 12 or model.nplugin != 4:
            raise ValueError('DEX-EE requires its four authored PID instances and twelve channels')
        for finger in range(3):
            names = []
            for number in range(4):
                joint_name = 'F%d/J%d' % (finger, number)
                name = joint_name+'_actuator'
                element = actuators.get(name)
                if element is None or element.get('plugin') != 'mujoco.pid' or element.get('joint') != joint_name:
                    raise ValueError('DEX-EE motor/plugin binding differs from source')
                config = instances.get(element.get('instance'), {})
                if (set(config) != {'kp', 'ki', 'kd', 'imax', 'slewmax'}
                        or not all(np.isfinite(value) and value > 0 for value in config.values())):
                    raise ValueError('DEX-EE PID gains/integral/slew bounds must be authored and positive')
                index = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)
                joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
                if index < 0 or joint < 0:
                    raise ValueError('DEX-EE source joint/motor missing in executed plant')
                plugin = int(model.actuator_plugin[index])
                if plugin < 0 or _name(mj, model, mj.mjtObj.mjOBJ_PLUGIN, plugin) != element.get('instance'):
                    raise ValueError('DEX-EE compiled PID instance differs from source')
                # The plugin's compiled config order is kp, ki, kd, imax,
                # slewmax (upstream plugin/actuator/pid.cc). Read raw model
                # attrs; accessing a temporary MjsActuator.plugin proxy is
                # unsafe in the installed Python binding.
                start = int(model.plugin_attradr[plugin])
                end = int(model.plugin_attradr[plugin+1]) if plugin+1 < model.nplugin else len(model.plugin_attr)
                values = bytes(model.plugin_attr[start:end]).rstrip(b'\0').split(b'\0')
                if len(values) != 5 or not np.allclose([float(x) for x in values],
                        [config[key] for key in ('kp', 'ki', 'kd', 'imax', 'slewmax')], rtol=0, atol=1e-12):
                    raise ValueError('DEX-EE executed PID settings differ from source')
                if (int(model.actuator_trntype[index]) != int(mj.mjtTrn.mjTRN_JOINT)
                        or int(model.actuator_trnid[index, 0]) != joint
                        or int(model.jnt_type[joint]) != int(mj.mjtJoint.mjJNT_HINGE)
                        or not np.array_equal(model.actuator_gear[index], [1, 0, 0, 0, 0, 0])
                        or not model.actuator_ctrllimited[index] or not model.actuator_forcelimited[index]
                        or int(model.actuator_dyntype[index]) != int(mj.mjtDyn.mjDYN_NONE)
                        or model.actuator_actnum[index] != 2 or model.actuator_actadr[index] < 0):
                    raise ValueError('DEX-EE joint/PID state/effort contract is incompatible')
                limits = np.array([max(model.jnt_range[joint, 0], model.actuator_ctrlrange[index, 0]),
                    min(model.jnt_range[joint, 1], model.actuator_ctrlrange[index, 1])])
                force_limits = model.actuator_forcerange[index]
                if (not np.all(np.isfinite(limits)) or limits[0] >= limits[1]
                        or not np.all(np.isfinite(force_limits)) or not force_limits[0] < 0 < force_limits[1]):
                    raise ValueError('DEX-EE bounds are not usable')
                channels.append(dict(index=int(index), name=name, joint=joint_name,
                    units='rad', limits=limits.tolist(), max_rate=min(.4, config['slewmax']),
                    actuation='pid_position', pid=config,
                    source_force_limits=force_limits.tolist(), source_force_units='N m'))
                names.append(name)
            groups.append(dict(name='F%d' % finger, channels=names,
                bodies=[i for i in range(model.nbody)
                        if _name(mj, model, mj.mjtObj.mjOBJ_BODY, i).startswith('F%d/' % finger)]))
        open_targets = {channel['name']: 0. for channel in channels}
        # Explicit templates within source ranges. J0 is lateral spread,
        # J1/J2/J3 curl. They make no claim to an object-specific grasp pose.
        curled = {channel['name']: (0., -.4, .9, .5)[int(channel['joint'][-1])] for channel in channels}
        poses = ['Curl', 'Pinch F0 + F1', 'Spread']
    else:
        source_joints = {element.get('name') for element in root.findall('.//worldbody//joint')}
        robot_joints = [j for j in range(model.njnt)
                        if _name(mj, model, mj.mjtObj.mjOBJ_JOINT, j) in source_joints]
        if not robot_joints or any(int(model.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE) for j in robot_joints):
            raise ValueError('Coupled hand requires authored fixed-mount joints')
        channels = position_channels(model, max(robot_joints)+1)
        hand_channels = [c for c in channels if c['name'] == 'fingers_actuator']
        if len(hand_channels) != 1:
            raise ValueError('Source fingers position channel is missing')
        hand = hand_channels[0]
        index = hand['index']
        if (int(model.actuator_trntype[index]) != int(mj.mjtTrn.mjTRN_TENDON)
                or not np.array_equal(model.actuator_gear[index], [1, 0, 0, 0, 0, 0])):
            raise ValueError('Keep the authored fingers tendon transmission')
        if entry.startswith('robotiq'):
            if not model.actuator_forcelimited[index] or not np.allclose(model.actuator_forcerange[index], [-5, 5]):
                raise ValueError('Keep Robotiq source effort limits')
            open_value, close_value = 0., 255.
            prefixes = ('right_', 'left_')
        else:
            open_value, close_value = .05, 0.
            prefixes = ('left_finger', 'right_finger')
        lower, upper = hand['limits']
        if not lower <= min(open_value, close_value) <= max(open_value, close_value) <= upper:
            raise ValueError('Source hand opening coordinates differ from reviewed template')
        open_targets, curled = {hand['name']: open_value}, {hand['name']: close_value}
        bodies = [i for i in range(model.nbody)
                  if _name(mj, model, mj.mjtObj.mjOBJ_BODY, i).startswith(prefixes)]
        if len(bodies) < 2:
            raise ValueError('Coupled hand finger bodies missing')
        groups = [dict(name='Coupled fingers', channels=[hand['name']], bodies=bodies)]
        poses = ['Close']
        for channel in channels:
            channel['source_force_units'] = 'N' if channel['units'] == 'm' or (
                channel['name'] == 'fingers_actuator' and entry == 'umi_gripper') else 'N m'
    contract = dict(adapter=entry, channels=channels, groups=groups,
        open_targets=open_targets, curled_targets=curled, poses=poses,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    # Bind the executed source motors, joint limits and coupling, independently
    # of appended selected-world bodies/objects. Do not bind a guessed TCP.
    digest = hashlib.sha256(json.dumps(contract, sort_keys=True).encode())
    for array in (model.actuator_trntype, model.actuator_trnid, model.actuator_gear,
                  model.actuator_gainprm, model.actuator_biasprm, model.actuator_ctrlrange,
                  model.actuator_forcerange, model.actuator_plugin, model.plugin_attr,
                  model.tendon_adr, model.tendon_num, model.wrap_type, model.wrap_objid, model.wrap_prm):
        digest.update(np.ascontiguousarray(array).tobytes())
    contract['contract_sha256'] = digest.hexdigest()
    return contract


def hand_pose_targets(contract, pose, closure, spread=0., finger='All'):
    closure, spread = float(closure), float(spread)
    if not np.isfinite(closure+spread) or not 0 <= closure <= 1 or not -1 <= spread <= 1:
        raise ValueError('Closure must be 0–1 and spread −1–1')
    if pose not in [*contract['poses'], 'Open']:
        raise ValueError('Choose an available source hand pose')
    groups = contract['groups']
    if finger != 'All' and finger not in {group['name'] for group in groups}:
        raise ValueError('Choose an available finger group')
    names = {name for group in groups if finger in ('All', group['name']) for name in group['channels']}
    targets = {}
    for name in names:
        start, end = contract['open_targets'][name], contract['curled_targets'][name]
        amount = 0. if pose in ('Open', 'Spread') else closure
        if pose == 'Pinch F0 + F1' and name.startswith('F2/'):
            amount = 0.
        value = start+amount*(end-start)
        if contract['adapter'] == 'shadow_dexee' and name.endswith('J0_actuator') and pose != 'Open':
            value = spread*.25*(1. if name.startswith('F0/') else -1. if name.startswith('F1/') else 0.)
        channel = next(c for c in contract['channels'] if c['name'] == name)
        if not channel['limits'][0] <= value <= channel['limits'][1]:
            raise ValueError('Pose template exceeds the executed source bounds')
        targets[name] = float(value)
    return targets
