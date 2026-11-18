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
    # A continuous joint without a declared range (Fetch's rolls).
    unlimited = URDF.replace('type="revolute"', 'type="continuous"').replace('lower="-3" upper="3" ', '')
    with pytest.raises(ValueError, match='finite source position limits'):
        lab_servo_articulation(unlimited)
    with pytest.raises(ValueError, match='finite source position limits'):
        lab_servo_articulation(URDF.replace('lower="-3" upper="3"', 'lower="3" upper="-3"'))


def test_continuous_joint_with_a_declared_source_range_is_a_servo(tmp_path):
    # Kinova's j2s6s200 writes +-2 pi on its continuous joints.
    text = URDF.replace('type="revolute"', 'type="continuous"')
    config = lab_servo_articulation(text)
    assert (config['joints']['shoulder']['lower'], config['joints']['shoulder']['upper']) == (-3., 3.)
    model = tmp_path / 'arm.urdf'
    model.write_text(text)
    URDFArticulation(config, text, model)                            # the contract accepts it


def test_placeholder_source_efforts_are_refused():
    import pytest
    from robot_lab_utils.urdf_lab_servos import lab_servo_articulation
    urdf = ('<robot name="r"><link name="a"/><link name="b"/><joint name="finger" type="revolute">'
            '<parent link="a"/><child link="b"/><limit lower="-4" upper="4" effort="10000000" velocity="1"/>'
            '</joint></robot>')
    with pytest.raises(ValueError, match='placeholder'):
        lab_servo_articulation(urdf)


def test_joints_whose_range_excludes_zero_spawn_at_their_home(tmp_path):
    # Kinova's shoulder ranges 0.82-5.46 rad: a plant at 0 starts out of bounds.
    text = URDF.replace('lower="-3" upper="3"', 'lower="0.5" upper="3"')
    config = lab_servo_articulation(text, home={'shoulder': 1.5})
    model = tmp_path / 'arm.urdf'
    model.write_text(text)
    contract = URDFArticulation(config, text, model)
    assert contract.spawn_positions == {'shoulder': 1.5}             # the finger's 0 is in range
