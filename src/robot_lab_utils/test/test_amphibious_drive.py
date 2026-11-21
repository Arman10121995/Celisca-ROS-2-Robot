"""Ported amphibious rover plant (R10.7, 2026-11-21)."""
from pathlib import Path

import numpy as np
import pytest

from robot_lab_utils.amphibious_drive import AmphibiousDrive, WHEELS, SUSPENSION, collision_shapes

SOURCE = Path(__file__).resolve().parents[2] / 'robot_lab_robots/lab_amphibious/urdf/lab_amphibious.urdf.xacro'


@pytest.fixture(scope='module')
def description():
    from robot_lab_utils.robot_description import load_description
    return load_description(str(SOURCE), 'model_name:=lab_amphibious')


def state(z, wheel_rate=0.):
    joints = {name: (0., wheel_rate) for name in WHEELS}
    joints.update({name: (0., 0.) for name in SUSPENSION})
    poses = {'base_link': (np.array([0., 0., z]), np.eye(3))}
    for i, name in enumerate(('front_left_wheel', 'rear_left_wheel', 'front_right_wheel', 'rear_right_wheel')):
        poses[name] = (np.array([.43 if i % 2 == 0 else -.43, .49 if i < 2 else -.49, z - .12]), np.eye(3))
    return joints, poses


def test_shapes_fold_fixed_links_into_the_hull(description):
    links = [shape[0] for shape in collision_shapes(description)]
    assert links.count('base_link') == 2 and len(links) == 6        # hull box + OAK-D housing + four wheels


def test_wheel_servo_and_lease(description):
    drive = AmphibiousDrive(description, dict(surface_z=0.))
    joints, poses = state(.73)
    drive.command_wheels(.22, 0., 0., 0., 0.)
    efforts, _, _ = drive.step(.1, np.eye(3), [0, 0, .73], [0, 0, .61], [0, 0, 0], [0, 0, 0], joints, poses)
    assert efforts['front_left_wheel_joint'] == pytest.approx(8.)    # 12 * (1 rad/s) at the 8 Nm cap
    efforts, _, _ = drive.step(.5, np.eye(3), [0, 0, .73], [0, 0, .61], [0, 0, 0], [0, 0, 0], joints, poses)
    assert efforts['front_left_wheel_joint'] == 0.                   # 0.4 s lease expired
    drive.command_wheels(.2, 0., .1, 0., 1.)                         # strafe is refused
    assert drive.drive == (0., 0.)


def test_buoyancy_floats_the_rover_near_the_gazebo_draft(description):
    drive = AmphibiousDrive(description, dict(surface_z=0., density=1025.))
    def lift(z):
        joints, poses = state(z)
        _, forces, _ = drive.step(0., np.eye(3), [0, 0, z], [0, 0, z - .12], [0, 0, 0], [0, 0, 0], joints, poses)
        return sum(f[1][2] for f in forces)
    weight = 54.56 * 9.81
    # Gazebo floats it at z 0.133: lift brackets the weight around there.
    assert lift(.10) > weight > lift(.18)


def test_propellers_push_only_when_immersed(description):
    drive = AmphibiousDrive(description, dict(surface_z=0.))
    for joint in drive.mounts:
        drive.command_thrust(joint, 50., 0.)
    joints, poses = state(.14)
    _, forces, _ = drive.step(.1, np.eye(3), [0, 0, .14], [0, 0, .02], [0, 0, 0], [0, 0, 0], joints, poses)
    # Capped at 35 N, then gated: the mounts sit 0.01 m under water, (0.01 + 0.065) / 0.13 of it.
    assert drive.state['applied_thrust_N'] == [pytest.approx(35. * .075 / .13), pytest.approx(35. * .075 / .13)]
    joints, poses = state(.73)
    drive.step(.2, np.eye(3), [0, 0, .73], [0, 0, .61], [0, 0, 0], [0, 0, 0], joints, poses)
    assert drive.state['applied_thrust_N'] == [0., 0.]
