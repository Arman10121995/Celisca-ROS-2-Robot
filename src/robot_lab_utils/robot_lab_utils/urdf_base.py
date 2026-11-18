"""Whether a URDF describes a fixed-base robot (shared by the simulator bridges)."""
import xml.etree.ElementTree as ET


def fixed_to_world(urdf_text):
    """True when the URDF itself anchors the robot: its root link is 'world'
    and no joint leaving it floats (e.g. the robot_assets Franka Panda,
    UR5/UR10, YuMi with a fixed mount; Valkyrie's forearm test stands, whose
    first joint turns against the world). Bridges then keep the base welded
    instead of floating it."""
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return False
    children = {j.find('child').get('link') for j in root.findall('joint') if j.find('child') is not None}
    roots = [link.get('name') for link in root.findall('link') if link.get('name') not in children]
    leaving = [j for j in root.findall('joint')
               if j.find('parent') is not None and j.find('parent').get('link') == 'world']
    return roots == ['world'] and bool(leaving) and all(j.get('type') not in ('floating', 'planar') for j in leaving)


def mount_to_world(urdf_text):
    """URDF text with a 'world' link fixed to its single root link, for robots
    that are mounted but whose source URDF does not say so (a profile's
    articulation ``fixed_base: true``: the R2 upper bodies, the KUKA iiwa)."""
    root = ET.fromstring(urdf_text)
    children = {j.find('child').get('link') for j in root.findall('joint') if j.find('child') is not None}
    roots = [link.get('name') for link in root.findall('link') if link.get('name') not in children]
    if len(roots) != 1:
        raise ValueError('Expected one URDF root link to mount, found %s' % roots)
    end = urdf_text.rfind('</robot>')
    mount = ('<link name="world"/><joint name="robot_lab_world_mount" type="fixed">'
             '<parent link="world"/><child link="%s"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>\n' % roots[0])
    return urdf_text[:end] + mount + urdf_text[end:]
