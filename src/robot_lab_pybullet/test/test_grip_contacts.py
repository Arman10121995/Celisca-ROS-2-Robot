from unittest.mock import Mock

import pytest

from robot_lab_pybullet.grip_contacts import GripContacts


def test_force_reader_distinguishes_pad_points_from_the_same_links_hull():
    bullet = pytest.importorskip('pybullet')
    plant = Mock(wraps=bullet)
    urdf = '''<robot name="gripper">
      <link name="left"><collision name="left_pad">
        <origin xyz="0 .03 0" rpy="0 0 1.57079632679"/>
        <geometry><box size=".002 .02 .02"/></geometry>
      </collision></link>
      <link name="right"><collision name="right_pad">
        <geometry><box size=".002 .02 .02"/></geometry>
      </collision></link>
    </robot>'''
    reader = GripContacts(plant, 4, {'left': 2, 'right': 3}, urdf,
                          {'left': 'left_pad', 'right': 'right_pad'})
    yaw = bullet.getQuaternionFromEuler((0., 0., 1.57))
    identity = (0., 0., 0., 1.)
    plant.getLinkState.side_effect = lambda _body, index, **_: (
        None, None, None, None, (1., 2., .5), yaw) if index == 2 else (
        None, None, None, None, (1., 2., .5), identity)

    def contact(object_id, link, point, force):
        return (0, 4, object_id, link, -1, point, None, None, None, force)

    on_left_pad, _ = bullet.multiplyTransforms((1., 2., .5), yaw, (.008, .031, 0.), identity)
    on_left_hull, _ = bullet.multiplyTransforms((1., 2., .5), yaw, (0., .039, 0.), identity)
    plant.getContactPoints.return_value = [
        contact(8, 2, on_left_pad, 1.5), contact(8, 2, on_left_hull, 12.),
        contact(8, 3, (1.001, 2., .5), 2.), contact(12, 3, (1.001, 2., .5), 30.),
        contact(9, 1, (1., 2., .5), 10.),
    ]
    assert reader.measure({'block': 8, 'other': 9}, 3.5) == {
        'sim_time_s': 3.5,
        'normal_force_n': {'block': {'left': 1.5, 'right': 2.}, 'other': {'left': 0., 'right': 0.}},
    }
