"""Passive springs retain physical units and reject broken model declarations."""
import pytest
from robot_lab_utils.urdf_springs import joint_springs, spring_force


URDF = '''<robot name="suspension"><link name="base"/><link name="wheel"/>
<joint name="drop" type="prismatic"><parent link="base"/><child link="wheel"/>
<limit lower="0" upper="0.03" effort="0" velocity="0"/>
<dynamics damping="50" friction="0.1"/></joint>
<gazebo reference="drop"><springStiffness>450</springStiffness>
<springReference>0.03</springReference></gazebo></robot>'''


def test_passive_elastic_and_damping_force_despite_zero_actuator_limit():
    spring = joint_springs(URDF)['drop']
    assert spring_force(spring, 0.0, 0.0) == pytest.approx(13.5)
    assert spring_force(spring, 0.03, 0.0) == pytest.approx(0.0)
    assert spring_force(spring, 0.03, 0.1) == pytest.approx(-5.0)


@pytest.mark.parametrize('old,new', [
    ('>450<','>nan<'), ('>450<','>-1<'), ('>0.03<','>0.04<'),
    ('reference="drop"','reference="missing"'), ('damping="50"','damping="-1"'),
])
def test_bad_spring_cannot_silently_become_a_free_joint(old,new):
    with pytest.raises(ValueError):
        joint_springs(URDF.replace(old,new))


def test_absent_springs_preserve_legacy_models():
    assert joint_springs('<robot name="legacy"/>') == {}
    assert joint_springs('') == {}
