"""Contact parameters a URDF carries for Gazebo, for the other backends.

Descriptions put per-link friction in ``<gazebo reference="link"><mu1>``,
which only Gazebo reads.  Bumperbot's casters use mu 0.1 so they slide
sideways while the robot turns, and its wheels 1e15 for grip.  The other
bridges strip ``<gazebo>`` blocks before loading the URDF and fell back to
engine defaults (PyBullet: 0.5 on every link), so the casters dragged and a
0.6 rad/s turn command produced 0.4 rad/s.

Pure Python, no ROS or simulator imports.
"""
import xml.etree.ElementTree as ET


def gazebo_link_friction(urdf_text):
    """``{link: mu}`` from ``<gazebo reference=...>`` friction blocks.

    ``mu1`` is used, falling back to ``mu2``; links without either are
    omitted, so the engine default applies to them.
    """
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return {}
    friction = {}
    for block in root.findall("gazebo"):
        link = block.get("reference")
        if not link:
            continue
        for tag in ("mu1", "mu2"):
            element = block.find(tag)
            if element is None or not (element.text or "").strip():
                continue
            try:
                friction[link] = float(element.text)
            except ValueError:
                continue
            break
    return friction
