"""Why a robot is display-only: derived from its own source model (ROADMAP R3.6).

For every installed profile without a control path, the source model is
inspected (the MJCF a native profile names, else its URDF): floating base,
the actuators the vendor declared, ``ros2_control``/transmission tags, legs,
rotors and wheels. The reason states what a controller would need and what
the source does or does not provide, so the GUI never shows a bare
"display only". Regenerate after provisioning:

    python3 -m robot_lab_utils.display_reasons --write

The result lives in ``robot_lab_robots/config/display_reasons.yaml``.
"""
import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

LEG = re.compile(r'(foot|leg|calf|thigh|knee|ankle|hip|shin)', re.I)
ROTOR = re.compile(r'(rotor|prop|thrust)', re.I)
WHEEL = re.compile(r'wheel', re.I)
CONTROL_KEYS = ('drive_in_display', 'articulation', 'arm_control', 'native_articulation', 'mobile_control_config',
                'native_hand', 'marine_model', 'gait_model')


def _mjcf_facts(path):
    root = ET.parse(path).getroot()
    included = []
    for include in root.iter('include'):
        candidate = Path(path).parent / include.get('file', '')
        if candidate.is_file():
            included.append(ET.parse(candidate).getroot())
    roots = [root] + included
    bodies = [b for r in roots for b in r.iter('body')]
    actuators = {}
    for r in roots:
        for block in r.iter('actuator'):
            for actuator in block:
                actuators[actuator.tag] = actuators.get(actuator.tag, 0) + 1
    names = ' '.join((b.get('name') or '') for b in bodies)
    return dict(format='mjcf', floating=any(r.find('.//freejoint') is not None or
                                            any(j.get('type') == 'free' for j in r.iter('joint')) for r in roots),
                actuators=actuators, joints=sum(1 for r in roots for j in r.iter('joint') if j.get('type') != 'free'),
                legs=len(LEG.findall(names)), rotors=len(ROTOR.findall(names)), wheels=len(WHEEL.findall(names)),
                ros2_control=False, transmissions=0)


def _urdf_facts(path):
    text = Path(path).read_text(errors='replace')
    root = ET.fromstring(text)
    links = [l.get('name', '') for l in root.findall('link')]
    joints = [j for j in root.findall('joint') if j.get('type') in ('revolute', 'continuous', 'prismatic')]
    names = ' '.join(links)
    world = any(j.find('parent') is not None and j.find('parent').get('link') == 'world' for j in root.findall('joint'))
    # A URDF root without a world joint says nothing about how it is spawned;
    # legs decide the locomotion reason instead.
    return dict(format='urdf', floating=None if not world else False, actuators={}, joints=len(joints),
                legs=len(LEG.findall(names)), rotors=len(ROTOR.findall(names)), wheels=len(WHEEL.findall(names)),
                ros2_control=root.find('ros2_control') is not None, transmissions=len(root.findall('transmission')))


def model_facts(profile):
    for key in ('native_mjcf',):
        if profile.get(key) and Path(profile[key]).is_file():
            return _mjcf_facts(profile[key])
    source = profile.get('xacro') or profile.get('urdf')
    if source and Path(source).is_file() and Path(source).suffix in ('.urdf', '.xml'):
        return _urdf_facts(source)
    return None


def reason(facts, profile):
    """(code, text) for a display-only profile."""
    if facts is None:
        return 'model_unavailable', 'The source model file is not installed here, so it cannot be inspected or controlled.'
    position = facts['actuators'].get('position', 0)
    other = sum(v for k, v in facts['actuators'].items() if k != 'position')
    thrusters = facts['actuators'].get('motor', 0) + facts['actuators'].get('general', 0)
    if facts['floating'] and (facts['rotors'] >= 2 or (facts['joints'] == 0 and thrusters >= 3)):
        return 'needs_flight_controller', ('A free-flying vehicle with %d thrust actuators: it needs a flight '
                                           'controller and state estimate (PX4 or the vendor\'s); none is provisioned '
                                           'for this model.' % thrusters)
    if facts['floating'] and facts['joints'] >= 8 and facts['legs'] < 4:
        return 'needs_base_controller', ('A floating-base model with %d joints: its servos alone cannot keep it '
                                         'upright or move its base; it needs a balance/base controller for this model, '
                                         'which is not provisioned.' % facts['joints'])
    if facts['floating'] is not False and facts['legs'] >= 4:
        kind = 'humanoid' if facts['legs'] < 8 and facts['joints'] >= 18 else 'legged robot'
        return 'needs_locomotion_policy', ('A floating-base %s: standing or walking needs a balance/locomotion '
                                           'controller or trained policy for exactly this model; none is provisioned. '
                                           'Joint jogging alone would let it fall.' % kind)
    if facts['format'] == 'mjcf' and not facts['actuators']:
        return 'no_source_actuators', 'The source MJCF declares no actuators; any controller would invent gains.'
    if facts['format'] == 'mjcf' and other and not position:
        return 'torque_actuators_only', ('The vendor model drives %d joints with motor/torque actuators only; it needs '
                                         'a torque-level controller (gravity compensation, impedance), not position '
                                         'jogging.' % other)
    if facts['format'] == 'urdf' and not (facts['ros2_control'] or facts['transmissions']):
        return 'no_control_interface', ('The source URDF has %d movable joints but no ros2_control or transmission '
                                        'description, so there is no vendor controller interface to use; gains would '
                                        'be invented.' % facts['joints'])
    if facts['format'] == 'urdf':
        return 'interface_not_wired', ('The source URDF declares %s; wiring it to a simulator controller is pending '
                                       'and needs a joint-state and motion check first.'
                                       % ('ros2_control' if facts['ros2_control'] else '%d transmissions' % facts['transmissions']))
    return 'not_enabled', ('The model has %d position actuators but no control path is enabled for it yet.' % position)


def display_only(profile):
    return set(profile.get('supported_modes', [])) <= {'display'} and not any(profile.get(k) for k in CONTROL_KEYS)


def generate(profiles):
    result = {}
    for name, profile in sorted(profiles.items()):
        if not display_only(profile):
            continue
        try:
            facts = model_facts(profile)
        except (ET.ParseError, OSError, ValueError) as exc:
            facts, code, text = None, 'model_unreadable', 'The source model could not be parsed: %s' % exc
        else:
            code, text = reason(facts, profile)
        result[name] = dict(code=code, reason=text, facts=facts)
    return result


def load(path=None):
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('robot_lab_robots')) / 'config' / 'display_reasons.yaml'
        except Exception:  # noqa: BLE001 - not installed
            return {}
    try:
        return yaml.safe_load(Path(path).read_text()) or {}
    except OSError:
        return {}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--write', action='store_true')
    options = parser.parse_args(argv)
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_utils.installed_assets import merge_installed_profiles
    share = Path(get_package_share_directory('robot_lab_robots'))
    profiles = merge_installed_profiles(yaml.safe_load((share / 'config' / 'robots.yaml').read_text()), 'robots')['robots']
    result = generate(profiles)
    counts = {}
    for row in result.values():
        counts[row['code']] = counts.get(row['code'], 0) + 1
    print(yaml.safe_dump(dict(display_only=len(result), by_reason=counts), sort_keys=False))
    if options.write:
        target = Path(__file__).resolve().parents[2] / 'robot_lab_robots' / 'config' / 'display_reasons.yaml'
        target.write_text('# Generated by python3 -m robot_lab_utils.display_reasons --write; do not edit by hand.\n'
                          + yaml.safe_dump(result, sort_keys=True, width=110))
        print('wrote', target)


if __name__ == '__main__':
    main()
