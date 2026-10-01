"""Which URDF joints are passive parts of a driven wheel assembly.

A mecanum hub is a driven wheel body carrying fifteen passive roller bodies.
A simulator's display-mode "hold" (a spring-damper or position motor that
keeps a joint at its spawn pose) must skip those rollers: a pinned roller
cannot spin, so the roller wheel behaves as a solid tire and resists the
lateral motion the drive model commands.  Measured 2026-10-01 on MuJoCo:
0.014 m/s of a commanded 0.30 m/s strafe with the rollers held, against
0.28 m/s with them free in the same model.
"""

import xml.etree.ElementTree as ET


def link_parents(urdf_text):
    """Child link -> parent link, from the URDF joint tree."""
    parents = {}
    for joint in ET.fromstring(urdf_text).findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None and child is not None:
            parents[child.get("link")] = parent.get("link")
    return parents


def driven_assembly_joints(urdf_text, wheel_joint_names):
    """Joint names of the passive parts hanging below the driven wheels.

    The wheel joints themselves are never included: they are commanded by
    the drive model.  Returns an empty set when no wheel joint from
    ``wheel_joint_names`` exists in the description.
    """
    wheels = set(wheel_joint_names or ())
    if not wheels:
        return set()
    root = ET.fromstring(urdf_text)
    wheel_links = set()
    for joint in root.findall("joint"):
        if joint.get("name") in wheels:
            child = joint.find("child")
            if child is not None:
                wheel_links.add(child.get("link"))
    if not wheel_links:
        return set()
    parents = link_parents(urdf_text)
    passive = set()
    for joint in root.findall("joint"):
        if joint.get("name") in wheels:
            continue
        child = joint.find("child")
        link, seen = (child.get("link") if child is not None else None), set()
        while link is not None and link not in seen:
            if link in wheel_links:
                passive.add(joint.get("name"))
                break
            seen.add(link)
            link = parents.get(link)
    return passive
