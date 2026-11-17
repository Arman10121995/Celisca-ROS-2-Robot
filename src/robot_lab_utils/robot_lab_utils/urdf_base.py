"""Whether a URDF describes a fixed-base robot (shared by the simulator bridges)."""
import xml.etree.ElementTree as ET


def fixed_to_world(urdf_text):
    """True when the URDF itself anchors the robot: its root link is 'world'
    and every joint leaving it is fixed (e.g. the robot_assets Franka Panda,
    UR5/UR10, YuMi). Bridges then keep the base welded instead of floating it."""
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return False
    children = {j.find('child').get('link') for j in root.findall('joint') if j.find('child') is not None}
    roots = [link.get('name') for link in root.findall('link') if link.get('name') not in children]
    leaving = [j for j in root.findall('joint')
               if j.find('parent') is not None and j.find('parent').get('link') == 'world']
    return roots == ['world'] and bool(leaving) and all(j.get('type') == 'fixed' for j in leaving)
