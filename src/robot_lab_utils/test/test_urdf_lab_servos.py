"""Declared lab servos derived from URDF limits (urdf_lab_servos)."""
import pytest

from robot_lab_utils.urdf_articulation import URDFArticulation
from robot_lab_utils.urdf_lab_servos import lab_servo_articulation

URDF = '''<robot name="arm"><link name="world"/><link name="a"/><link name="b"/><link name="c"/>
<joint name="fix" type="fixed"><parent link="world"/><child link="a"/></joint>
<joint name="shoulder" type="revolute"><parent link="a"/><child link="b"/>
  <limit lower="-3" upper="3" effort="100" velocity="2"/></joint>
<joint name="finger" type="prismatic"><parent link="b"/><child link="c"/>
  <limit lower="0" upper="0.04" effort="20" velocity="0.2"/></joint>
<joint name="finger_m" type="prismatic"><parent link="b"/><child link="c"/>
  <limit lower="0" upper="0.04" effort="20" velocity="0.2"/><mimic joint="finger"/></joint></robot>'''


def test_limits_gains_caps_and_mimic(tmp_path):
    config = lab_servo_articulation(URDF, home={'shoulder': 5.})
    shoulder, finger = config['joints']['shoulder'], config['joints']['finger']
    assert shoulder['kp'] == 4000. and shoulder['kd'] == pytest.approx(80.)
    assert shoulder['max_velocity'] == .35 and finger['max_velocity'] == .05
    assert config['couplings'] == {'finger_m': dict(joint='finger', multiplier=1., offset=0.)}
    assert config['home'] == {'shoulder': 3., 'finger': 0.}          # clamped into the source range
    model = tmp_path / 'arm.urdf'
    model.write_text(URDF)
    URDFArticulation(config, URDF, model)                            # the contract accepts it


def test_unlimited_joints_are_refused():
    with pytest.raises(ValueError, match='finite source position limits'):
        lab_servo_articulation(URDF.replace('type="revolute"', 'type="continuous"'))


def test_placeholder_source_efforts_are_refused():
    import pytest
    from robot_lab_utils.urdf_lab_servos import lab_servo_articulation
    urdf = ('<robot name="r"><link name="a"/><link name="b"/><joint name="finger" type="revolute">'
            '<parent link="a"/><child link="b"/><limit lower="-4" upper="4" effort="10000000" velocity="1"/>'
            '</joint></robot>')
    with pytest.raises(ValueError, match='placeholder'):
        lab_servo_articulation(urdf)
