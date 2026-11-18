"""Fixed-base detection and world mounting of URDFs (R3.6)."""
import pytest

from robot_lab_utils.urdf_base import fixed_to_world, mount_to_world

LOOSE = ('<robot name="arm"><link name="base"/><link name="a"/>'
         '<joint name="j" type="revolute"><parent link="base"/><child link="a"/>'
         '<limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>')


def test_world_root_with_a_moving_first_joint_is_fixed_but_a_floating_one_is_not():
    stand = LOOSE.replace('link name="base"', 'link name="world"').replace('parent link="base"', 'parent link="world"')
    assert fixed_to_world(stand)                       # Valkyrie forearm stands
    assert not fixed_to_world(stand.replace('type="revolute"', 'type="floating"'))
    assert not fixed_to_world(LOOSE)


def test_mount_to_world_anchors_the_single_root():
    mounted = mount_to_world(LOOSE)
    assert fixed_to_world(mounted)
    assert 'parent link="world"/><child link="base"/>' in mounted
    with pytest.raises(ValueError):
        mount_to_world('<robot name="two"><link name="a"/><link name="b"/></robot>')
