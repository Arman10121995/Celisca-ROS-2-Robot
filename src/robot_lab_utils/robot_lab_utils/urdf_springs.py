"""Original Gazebo joint springs as a portable passive-force description."""
import math
import xml.etree.ElementTree as ET


def joint_springs(urdf):
    if not urdf:
        return {}
    root = ET.fromstring(urdf)
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    springs = {}
    for gazebo in root.findall('gazebo'):
        value = gazebo.findtext('springStiffness')
        if value is None:
            continue
        name = gazebo.get('reference')
        joint = joints.get(name)
        if joint is None or joint.get('type') not in ('prismatic', 'revolute', 'continuous'):
            raise ValueError('Spring requires a movable joint: '+str(name))
        dynamics = joint.find('dynamics')
        damping = float(dynamics.get('damping', '0')) if dynamics is not None else 0.0
        spring = dict(stiffness=float(value), reference=float(gazebo.findtext('springReference', '0')),
                      damping=damping, type=joint.get('type'))
        if not all(math.isfinite(spring[key]) for key in ('stiffness','reference','damping')) \
                or spring['stiffness'] <= 0 or damping < 0:
            raise ValueError('Invalid passive spring: '+name)
        limit = joint.find('limit')
        if limit is not None and joint.get('type') != 'continuous':
            if not float(limit.get('lower', '-inf')) <= spring['reference'] <= float(limit.get('upper', 'inf')):
                raise ValueError('Spring reference outside joint limits: '+name)
        if name in springs:
            raise ValueError('Duplicate passive spring: '+name)
        springs[name] = spring
    return springs


def spring_force(spring, position, velocity):
    """N for a slide, Nm for a hinge; no position/state manipulation."""
    return spring['stiffness'] * (spring['reference'] - position) - spring['damping'] * velocity
