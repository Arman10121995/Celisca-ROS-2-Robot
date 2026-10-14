"""End-to-end native-hand grasp fixture: the display authors a free object from
measured distal pads, and grasp/held/released flows through the native controller.

The object stays free: the display only authors it and exposes measured two-sided
contact, support and slip; the controller's grasp/held/ release state follows
measured contact evidence, and the physics engine owns the object after release.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_utils'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_mujoco/python'))

mj = pytest.importorskip('mujoco')
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('std_msgs.msg')

from std_msgs.msg import String  # noqa: E402
from robot_lab_mujoco.native_asset_display import NativeAssetDisplay  # noqa: E402
from robot_lab_mujoco.native_hand_control import NativeHandControl  # noqa: E402


def _robotiq_2f85_v4_spec(tmp_path, gap=0.09):
    """Minimal coupled-tendon pad hand with v4 distal-pad naming.

    The directory name chooses the reviewed adapter; the pads are the exact
    'left_pad'/'right_pad' bodies that object_geometry keys on. The driver
    joints are slides along the closing (x) axis so that increasing closure
    translates both pads inward onto the object — a hinge would only rotate
    the pads in place and never close the gap, so the measured grasp would
    never see two-sided contact.
    """
    folder = tmp_path / 'robotiq_2f85_v4'
    folder.mkdir()
    source = folder / 'robotiq_2f85_v4.xml'
    source.write_text(
        '<mujoco><compiler angle="radian" autolimits="true"/>'
        '<worldbody><body name="mount" pos="0 0 .05">'
        '<body name="left_pad" pos="%f 0 0.1">'
        '<joint name="left_driver_joint" type="slide" axis="1 0 0" range="0 .05"/>'
        '<geom type="box" size=".004 .012 .012"/></body>'
        '<body name="right_pad" pos="%f 0 0.1">'
        '<joint name="right_driver_joint" type="slide" axis="1 0 0" range="-.05 0"/>'
        '<geom type="box" size=".004 .012 .012"/></body>'
        '</body></worldbody>'
        '<tendon><fixed name="split"><joint joint="left_driver_joint" coef=".5"/>'
        '<joint joint="right_driver_joint" coef=".5"/></fixed></tendon>'
        '<actuator><general name="fingers_actuator" tendon="split" gainprm=".3137255" '
        'biastype="affine" biasprm="0 -100 -10" ctrlrange="0 255" forcerange="-5 5"/>'
        '</actuator></mujoco>' % (-gap / 2, gap / 2))
    model = mj.MjModel.from_xml_path(str(source))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    return source, model, data


def _minimal_urdf(path):
    path.write_text('<robot name="native_hand_display"><link name="base"/></robot>')


def _send(control, operation):
    return String(
        data=json.dumps(dict(
            operation=operation,
            plant_id=control.plant_id,
            source_sha256=control.source_sha256,
            contract_sha256=control.contract['contract_sha256'],
            closure=1., effort_fraction=.8, contact_limit_n=2., guarded=True)))

def test_native_hand_grasp_fixture_authors_object_between_pads_and_flows_held_release(
        tmp_path):
    source, model, data = _robotiq_2f85_v4_spec(tmp_path)
    model_path = tmp_path / 'display.urdf'
    _minimal_urdf(model_path)

    # Launch-time flags: gui/hold are bools in the real launch; set the node
    # attributes to the equivalent Python values so the timer steps physics.
    rclpy.init(args=['--ros-args',
                     '-p', 'native_mjcf:=' + str(source),
                     '-p', 'model:=' + str(model_path),
                     '-p', 'arm_control:=none',
                     '-p', 'grasp_fixture:=true',
                     '-p', 'adapter:=robotiq_2f85_v4',
                     '-p', 'articulation_control:=true',
                     '-p', 'gui:=false',
                     '-p', 'hold_position:=false'])
    node = None
    try:
        node = NativeAssetDisplay()
        node.gui = False
        node.hold = False
        control = node.articulation
        assert isinstance(control, NativeHandControl)

        # The display authors the object from the measured distal pads.
        assert control.object_body >= 0
        assert control.object_sides is not None
        left_ids, right_ids = control.object_sides
        assert left_ids and right_ids
        left_pos = node.data.xpos[int(list(left_ids)[0])]
        right_pos = node.data.xpos[int(list(right_ids)[0])]
        obj_pos = node.data.xpos[control.object_body]
        assert abs(obj_pos[0]) < 1e-6, 'object must sit on the pad midline'
        assert abs(obj_pos[2] - left_pos[2]) < 1e-6, 'object must sit at pad height'

        span = float(np.linalg.norm(right_pos - left_pos))
        half = np.asarray(node.object_size, dtype=float)
        assert half[0] <= min(.032, .32 * span) + 1e-9
        assert half[1] <= min(.03, .32 * span) + 1e-9
        assert abs(obj_pos[2] - left_pos[2]) < 1e-6

        # Grasp: close on the documented object and evaluate measured contact.
        control.heartbeat(None)
        control.receive(_send(control, 'hand_grasp'))
        assert control.grasp['state'] == 'closing'
        for _ in range(400):
            node.tick()
            if control.grasp['state'] in ('holding', 'slipping'):
                break
        assert control.grasp['state'] in ('holding', 'slipping'), (
            'two-sided contact expected while closing; got %r' % control.grasp['state'])
        assert control.grasp['left_force_n'] > 0 and control.grasp['right_force_n'] > 0
        assert control.grasp['right_force_n'] >= .2 * control.grasp['contact_limit_n']

        # Hold is the two-sided object contact, not any external pedestal touch.
        held_between_pads = abs(obj_pos[0]) < .32 * span and abs(obj_pos[2] - left_pos[2]) < 1e-3
        assert held_between_pads, 'object must remain between the pads while held'

        # Release: open and let the physics engine own the object again.
        control.heartbeat(None)
        control.receive(_send(control, 'hand_release'))
        assert control.grasp['state'] == 'releasing'
        for _ in range(300):
            node.tick()
            if control.grasp['state'] == 'released':
                break
        assert control.grasp['state'] == 'released', (
            'object must become free after release; got %r' % control.grasp['state'])
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

