"""A merged fixed camera follows its original moving head, not the base."""
import math
import xml.etree.ElementTree as ET

import mujoco
import pytest

from robot_lab_mujoco.mujoco_spawner import _add_camera_to_base
from robot_lab_utils.sim_frames import offset_from_root, rigid_sensor_offsets

URDF = '''<robot name="test"><link name="base"/><link name="head"/>
<link name="camera"/><joint name="pan" type="revolute">
<parent link="base"/><child link="head"/><origin xyz="0 0 1"/>
<axis xyz="0 0 1"/></joint><joint name="mount" type="fixed">
<parent link="head"/><child link="camera"/><origin xyz=".1 0 .2"/>
</joint></robot>'''
MJCF = '''<mujoco><worldbody><body name="robot"><freejoint/>
<geom type="box" size=".1 .1 .1"/><body name="head" pos="0 0 1">
<joint name="pan" axis="0 0 1"/><geom type="sphere" size=".03"/>
</body></body></worldbody></mujoco>'''


def test_camera_tracks_actual_head_pan_and_fixed_mount():
    xml = _add_camera_to_base(MJCF, 'cam', offset_from_root(URDF, 'camera'),
                              40., urdf=URDF, link='camera')
    assert ET.fromstring(xml).find("worldbody/body/body[@name='head']/camera") is not None
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'cam')
    pan = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'pan')
    for angle in (0., math.pi/2, -.7):
        data.qpos[model.jnt_qposadr[pan]] = angle
        mujoco.mj_forward(model, data)
        assert data.cam_xpos[camera] == pytest.approx([.1*math.cos(angle), .1*math.sin(angle), 1.2])
        assert -data.cam_xmat[camera].reshape(3, 3)[:, 2] == pytest.approx([math.cos(angle), math.sin(angle), 0.], abs=1e-6)


def test_lost_movable_carrier_is_refused_instead_of_static_fallback():
    missing = MJCF.replace('name="head"', 'name="wrong"')
    with pytest.raises(ValueError, match='movable carrier was lost'):
        _add_camera_to_base(missing, 'cam', offset_from_root(URDF, 'camera'),
                             40., urdf=URDF, link='camera')


def test_fused_camera_offsets_stop_at_the_moving_head():
    offsets = rigid_sensor_offsets(URDF, 'camera')
    assert list(offsets) == ['camera', 'head']
    assert offsets['head'][0] == pytest.approx([.1, 0., .2])
    assert 'base' not in offsets
    assert not rigid_sensor_offsets(URDF, 'missing')
