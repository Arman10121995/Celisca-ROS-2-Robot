"""Gazebo systems a robot needs to be displayed like in the other backends.

PyBullet, MuJoCo and Isaac publish ``/joint_states`` for every robot they
load.  Gazebo only does so through a controller: robots with a
``<ros2_control>`` block get it from ``joint_state_broadcaster``, while every
other robot (the Unitree and Berkeley Humanoid Lite descriptions) spawned
with no joint state at all, so RViz showed its links unplaced.  This module
adds, by editing the URDF text before it is spawned:

* the Gazebo ``JointStatePublisher`` system, for robots without
  ``<ros2_control>``, bridged to ``/joint_states`` by the launch file;
* optionally a ``JointPositionController`` per movable joint holding it at
  its spawn angle (zero, the URDF rest pose) through velocity commands,
  the Gazebo counterpart of the display hold in the other backends. Given
  the spawned model name, each controller listens on
  ``hold_command_topic(model, joint)``, a name ROS can bridge (Gazebo's
  default ``.../joint/<j>/0/cmd_pos`` has a token starting with a digit),
  and mimic joints get a controller too: Gazebo does not enforce
  ``<mimic>``, so the URDF articulation relay drives them as couplings.

Pure Python; the plugin blocks are inserted as text so the rest of the
description is passed through byte for byte.
"""
import re
import xml.etree.ElementTree as ET

JOINT_STATE_PUBLISHER = (
    '<gazebo><plugin filename="ignition-gazebo-joint-state-publisher-system" '
    'name="ignition::gazebo::systems::JointStatePublisher"/></gazebo>')

# Velocity servo gain (1/s) of the display hold: the joint velocity command
# is gain * (target - angle), applied within the joint's effort limit.
HOLD_GAIN = 20.0
HOLD_MAX_SPEED = 5.0  # rad/s


def joint_state_topic(world_name, model_name):
    """Gazebo topic the JointStatePublisher system publishes on."""
    return "/world/%s/model/%s/joint_state" % (world_name, model_name)


def hold_command_topic(model_name, joint_name):
    """Topic (valid in Gazebo and ROS) on which a joint's display hold takes its position target."""
    def token(text):
        text = re.sub(r'[^A-Za-z0-9_]', '_', text)
        return text if text[:1].isalpha() else 'j' + text
    return '/model/%s/joint/%s/cmd_pos' % (token(model_name), token(joint_name))


def has_ros2_control(urdf_text):
    try:
        return ET.fromstring(urdf_text).find("ros2_control") is not None
    except ET.ParseError:
        return False


def holdable_joints(urdf_text, include_mimic=False):
    """Movable joints of a URDF (without mimic joints unless asked), in document order."""
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return []
    return [joint.get("name") for joint in root.findall("joint")
            if joint.get("type") in ("revolute", "continuous", "prismatic")
            and (include_mimic or joint.find("mimic") is None) and joint.get("name")]


def add_display_plugins(urdf_text, hold=False, model_name=''):
    """URDF text with the systems a display run needs; see the module doc.

    Returns *urdf_text* unchanged for robots that already carry
    ``<ros2_control>``: their controllers publish joint states and own the
    joints.
    """
    if has_ros2_control(urdf_text):
        return urdf_text
    blocks = [JOINT_STATE_PUBLISHER]
    if hold:
        # Joint servos alone cannot balance a free humanoid torso. This is
        # a passive preview: disable gravity while retaining measured joint
        # state and the simulator's pose. Physical runs use hold=False.
        for link in ET.fromstring(urdf_text).findall('link'):
            block = ET.Element('gazebo', reference=link.get('name'))
            ET.SubElement(block, 'gravity').text = 'false'
            blocks.append(ET.tostring(block, encoding='unicode'))
        for name in holdable_joints(urdf_text, include_mimic=bool(model_name)):
            topic = ('<topic>%s</topic>' % hold_command_topic(model_name, name)
                     if model_name else '')
            blocks.append(
                '<gazebo><plugin filename="ignition-gazebo-joint-position-'
                'controller-system" name="ignition::gazebo::systems::'
                'JointPositionController"><joint_name>%s</joint_name>%s'
                '<use_velocity_commands>true</use_velocity_commands>'
                '<p_gain>%g</p_gain><cmd_max>%g</cmd_max><cmd_min>%g</cmd_min>'
                '</plugin></gazebo>' % (name, topic, HOLD_GAIN, HOLD_MAX_SPEED,
                                        -HOLD_MAX_SPEED))
    end = urdf_text.rfind("</robot>")
    if end < 0:
        return urdf_text
    return urdf_text[:end] + "\n".join(blocks) + "\n" + urdf_text[end:]
