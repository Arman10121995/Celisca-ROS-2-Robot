"""The robot is its own named free joint, not a loose world body that precedes it (R11 payload scenes)."""
from types import SimpleNamespace

import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("rclpy")

from robot_lab_mujoco.mujoco_spawner import MuJoCoSpawner  # noqa: E402

# World first (as _merge_mjcf orders it): a loose payload, then the wrapped robot.
XML = """<mujoco><worldbody>
  <body name="payload" pos="0 0 1"><freejoint/><geom type="box" size=".1 .1 .1"/></body>
  <body name="molar_hri_base"><freejoint name="molar_hri_freejoint"/><geom type="box" size=".2 .2 .1"/>
    <body name="wheel"><joint name="drive_left_joint" type="hinge"/><geom type="sphere" size=".05"/></body></body>
</worldbody></mujoco>"""


def spawner(robot_name):
    params = dict(robot_name=robot_name, left_wheel_joint='drive_left_joint', right_wheel_joint='drive_right_joint')
    return SimpleNamespace(_model=mujoco.MjModel.from_xml_string(XML), _lw_name='', _rw_name='',
                           get_parameter=lambda name: SimpleNamespace(value=params[name]),
                           get_logger=lambda: SimpleNamespace(info=lambda *_: None))


def test_named_robot_free_joint_wins_over_an_earlier_world_body():
    s = spawner('molar_hri')
    MuJoCoSpawner._find_body_and_joints(s)
    assert mujoco.mj_id2name(s._model, mujoco.mjtObj.mjOBJ_BODY, s._body_id) == 'molar_hri_base'
    assert s._free_joint_qpos_adr == 7                      # after the payload's 7 free-joint qpos
    assert s._joint_names == ['drive_left_joint'] and s._lw_qpos_adr == 14


def test_without_a_matching_name_the_first_free_joint_is_used():
    s = spawner('other')
    MuJoCoSpawner._find_body_and_joints(s)
    assert mujoco.mj_id2name(s._model, mujoco.mjtObj.mjOBJ_BODY, s._body_id) == 'payload'
