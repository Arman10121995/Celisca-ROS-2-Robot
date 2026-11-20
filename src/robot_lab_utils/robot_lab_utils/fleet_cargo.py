"""One physically loose reference payload on the stowed MOLAR fleet deck."""
import itertools
import math
import xml.etree.ElementTree as ET

from .sdf_world import compose, quaternion_from_rpy
from .sim_frames import invert, offset_from_root

CARGO = 'molar_reference_box'
DECK_FRAME = 'telescope_stage2'
SIZE = (.20, .15, .10)
MASS = .5
BACKSET = .10


def load_config(member, urdf):
    root = ET.fromstring(urdf)
    plate = root.find(f"./link[@name='{DECK_FRAME}']/collision[@name='{DECK_FRAME}_collision_2']")
    home = offset_from_root(urdf, DECK_FRAME)
    if plate is None or home is None or plate.find('origin') is None or plate.find('geometry/box') is None:
        raise ValueError('MOLAR reference payload needs its source deck collision')
    origin = plate.find('origin')
    xyz = tuple(map(float, origin.get('xyz').split()))
    size = tuple(map(float, plate.find('geometry/box').get('size').split()))
    deck, orientation = home
    if (orientation != (1., 0., 0., 0.) or any(float(v) for v in origin.get('rpy', '0 0 0').split())
            or size[0] < SIZE[0]+2*BACKSET or size[1] < SIZE[1]):
        raise ValueError('MOLAR reference payload needs its source horizontal deck plate')
    name = 'fleet_cargo_' + member['id']
    return dict(name=name, frame=name, topic='/fleet/cargo/'+member['id']+'/odom',
                mass_kg=MASS, size=SIZE, deck_center=xyz, deck_size=size, deck_home=deck)


def spawn_position(config, spawn):
    center, size = config['deck_center'], config['deck_size']
    relative = tuple(a+b+c for a, b, c in zip(config['deck_home'], center,
                     (-BACKSET, 0., size[2]/2+SIZE[2]/2+.003)))
    rotation = quaternion_from_rpy(0., 0., spawn['yaw'])
    position, _ = compose(((spawn['x'], spawn['y'], spawn['z']), rotation),
                           (relative, (1., 0., 0., 0.)))
    return position


def model_sdf(config):
    root = ET.Element('sdf', version='1.7')
    model = ET.SubElement(root, 'model', name=config['name'])
    link = ET.SubElement(model, 'link', name='box')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = str(MASS)
    tensor = ET.SubElement(inertial, 'inertia')
    for name, pair in zip(('ixx', 'iyy', 'izz'), ((1, 2), (0, 2), (0, 1))):
        ET.SubElement(tensor, name).text = str(MASS*sum(SIZE[index]**2 for index in pair)/12)
    for kind in ('visual', 'collision'):
        part = ET.SubElement(link, kind, name=kind)
        box = ET.SubElement(ET.SubElement(part, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = ' '.join(map(str, SIZE))
        if kind == 'visual':
            material = ET.SubElement(part, 'material')
            ET.SubElement(material, 'diffuse').text = '0.87 0.54 0.22 1'
        else:
            friction = ET.SubElement(ET.SubElement(ET.SubElement(part, 'surface'), 'friction'), 'ode')
            for axis in ('mu', 'mu2'):
                ET.SubElement(friction, axis).text = '0.8'
    plugin = ET.SubElement(model, 'plugin', filename='ignition-gazebo-odometry-publisher-system',
                           name='ignition::gazebo::systems::OdometryPublisher')
    for key, value in dict(odom_frame='map', robot_base_frame=config['frame'],
                           odom_topic=config['topic'], dimensions='3', odom_publish_frequency='25').items():
        ET.SubElement(plugin, key).text = value
    return ET.tostring(root, encoding='unicode')


def retention(config, base, deck, cargo):
    """Check the stow transform and all box corners against the measured deck."""
    relative, rotation = compose(invert(compose(base, deck)), cargo)
    deck_origin, deck_quaternion = deck
    if (math.dist(deck_origin, config['deck_home']) > .02
            or abs(deck_quaternion[0]) < math.cos(.02/2)):
        return False, relative, 'MOLAR deck is not stowed'
    points = [compose((relative, rotation), (corner, (1., 0., 0., 0.)))[0]
              for corner in itertools.product(*[(-extent/2, extent/2) for extent in config['size']])]
    center, size = config['deck_center'], config['deck_size']
    inside = all(abs(point[axis]-center[axis]) <= size[axis]/2 for point in points for axis in (0, 1))
    gap = min(point[2] for point in points)-center[2]-size[2]/2
    supported = inside and -.012 <= gap <= .025
    return supported, relative, 'Box retained on stowed deck' if supported else 'Box outside deck support envelope'
