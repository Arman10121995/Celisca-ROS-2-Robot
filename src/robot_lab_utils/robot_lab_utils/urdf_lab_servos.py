"""Declared lab position servos for URDF arms that ship no control interface.

The same rule as the TIAGo and URDF Panda adapters: every revolute or
prismatic joint with finite source limits (and a continuous joint whose
source declares a range) becomes a bounded position channel
of the shared URDF articulation contract (``urdf_articulation``), with
kp = 40 x effort, kd = 8 x sqrt(effort) and the source velocity capped at
0.35 rad/s (0.05 m/s for prismatic joints). Source mimic joints become
couplings. These are lab simulation servos, not vendor tuning.
"""
import math
import xml.etree.ElementTree as ET

ANGULAR_CAP = .35       # rad/s
LINEAR_CAP = .05        # m/s


# No real joint actuator exceeds this (N m or N); larger source values are placeholders.
MAX_SOURCE_EFFORT = 1e4


def has_source_range(joint):
    """True when the joint's own ``<limit>`` declares a finite lower < upper.

    A revolute or prismatic joint always reads its range from there. A
    continuous joint has none unless its source writes one: Kinova's
    j2s6s200 declares +-2 pi on its continuous joints, Fetch declares none.
    """
    limit = joint.find('limit')
    if limit is None:
        return False
    if joint.get('type') == 'continuous' and (limit.get('lower') is None or limit.get('upper') is None):
        return False
    try:
        low, high = float(limit.get('lower', 0)), float(limit.get('upper', 0))
    except ValueError:
        return False
    return math.isfinite(low) and math.isfinite(high) and low < high


def lab_servo_articulation(urdf_text, home=None):
    """robot_lab.urdf_articulation.v1 config for every limited movable joint; raises if any is unlimited."""
    root = ET.fromstring(urdf_text)
    joints, couplings = {}, {}
    for joint in root.findall('joint'):
        kind = joint.get('type')
        if kind not in ('revolute', 'prismatic', 'continuous'):
            continue
        name = joint.get('name')
        limit = joint.find('limit')
        if limit is None or not has_source_range(joint):
            raise ValueError('Joint %s has no finite source position limits' % name)
        low, high = float(limit.get('lower', 0)), float(limit.get('upper', 0))
        effort, rate = float(limit.get('effort', 0)), float(limit.get('velocity', 0))
        if not (low < high and effort > 0 and rate > 0 and all(map(math.isfinite, (low, high, effort, rate)))):
            raise ValueError('Joint %s lacks positive source effort/velocity or a range' % name)
        if effort > MAX_SOURCE_EFFORT:
            # R2's hand joints declare 1e7 and +-4 rad: placeholders, not
            # actuator limits; servos built from them freeze the joint.
            raise ValueError('Joint %s effort %g is a placeholder, not a source limit' % (name, effort))
        mimic = joint.find('mimic')
        if mimic is not None:
            couplings[name] = dict(joint=mimic.get('joint'), multiplier=float(mimic.get('multiplier', 1)),
                                   offset=float(mimic.get('offset', 0)))
            continue
        cap = LINEAR_CAP if kind == 'prismatic' else ANGULAR_CAP
        joints[name] = dict(mode='position', lower=low, upper=high, effort=effort,
                            max_velocity=min(rate, cap), kp=round(40. * effort, 6),
                            kd=round(8. * math.sqrt(effort), 6))
    if not joints:
        raise ValueError('No limited movable joints')
    home = dict(home or {})
    rest = {n: min(max(float(home.get(n, 0.)), j['lower']), j['upper']) for n, j in joints.items()}
    return dict(schema='robot_lab.urdf_articulation.v1', panel='arm', home=rest, joints=joints,
                couplings=couplings)
