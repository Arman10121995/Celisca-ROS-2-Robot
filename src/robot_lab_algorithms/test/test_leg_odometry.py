"""Leg kinematic odometry (robot_lab_algorithms.leg_odometry)."""
import math
import os

import numpy as np
import pytest

from robot_lab_algorithms.leg_odometry import (
    LegKinematics, LegOdometry, quaternion_matrix, stance_velocity)

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
G1 = os.path.join(_SRC, 'robot_lab_robots', 'unitree', 'g1_description', 'g1_29dof.urdf')

# A planar two-link leg on each side of a box base: hip pitch, knee pitch.
TOY = """<robot name="toy">
  <link name="base"/>
  %s
</robot>"""
LEG = """
  <joint name="{n}_hip" type="revolute"><parent link="base"/><child link="{n}_thigh"/>
    <origin xyz="{x} {y} 0" rpy="0 0 0"/><axis xyz="0 1 0"/></joint>
  <link name="{n}_thigh"/>
  <joint name="{n}_knee" type="revolute"><parent link="{n}_thigh"/><child link="{n}_calf"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/><axis xyz="0 1 0"/></joint>
  <link name="{n}_calf"/>
  <joint name="{n}_foot_fixed" type="fixed"><parent link="{n}_calf"/><child link="{n}_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/></joint>
  <link name="{n}_foot"/>"""


def toy_robot():
    legs = ''.join(LEG.format(n=n, x=x, y=y) for n, x, y in
                   (('FL', 0.2, 0.1), ('FR', 0.2, -0.1), ('RL', -0.2, 0.1), ('RR', -0.2, -0.1)))
    return TOY % legs


def test_foot_positions_and_velocities_match_finite_differences():
    kinematics = LegKinematics(toy_robot())
    assert kinematics.base_link == 'base'
    assert sorted(kinematics.foot_links) == ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']
    q = {'FL_hip': 0.3, 'FL_knee': -0.6}
    qd = {'FL_hip': 0.5, 'FL_knee': -1.0}
    p, v = kinematics.foot('FL_foot', q, qd)
    # Straight leg rotated 0.3 then 0.3 back at the knee: the foot is at
    # x = 0.2 - 0.2 sin(0.3) - 0.2 sin(-0.3) = 0.2.
    assert p == pytest.approx([0.2 - 0.2 * math.sin(0.3) - 0.2 * math.sin(-0.3), 0.1,
                               -0.2 * math.cos(0.3) - 0.2 * math.cos(-0.3)], abs=1e-9)
    eps = 1e-6
    q2 = {k: q[k] + qd[k] * eps for k in q}
    p2, _ = kinematics.foot('FL_foot', q2, qd)
    assert v == pytest.approx((p2 - p) / eps, abs=1e-5)


def test_stance_feet_give_the_base_velocity():
    rng = np.random.default_rng(0)
    v_base = np.array([0.3, -0.1, 0.0])
    omega = np.array([0.0, 0.0, 0.4])
    feet = {}
    for name, p in (('a', [0.2, 0.1, -0.3]), ('b', [-0.2, -0.1, -0.3]),
                    ('c', [0.2, -0.1, -0.24]), ('d', [-0.2, 0.1, -0.25])):
        p = np.array(p)
        stance = p[2] < -0.29
        v_rel = -(v_base + np.cross(omega, p)) if stance else rng.normal(size=3)
        feet[name] = (p, v_rel)
    velocity, stance = stance_velocity(feet, omega, np.eye(3), stance_height=0.02)
    assert stance == ['a', 'b']
    assert velocity == pytest.approx(v_base, abs=1e-12)


def test_swing_feet_close_to_the_ground_are_gated_out():
    """A shuffling trot: swing feet only 4 mm up, moving at about twice the body speed."""
    v_base = np.array([0.3, 0.0, 0.0])
    feet = {}
    for name, p, stance in (('FL', [0.2, 0.1, -0.300], True), ('RR', [-0.2, -0.1, -0.301], True),
                            ('FR', [0.2, -0.1, -0.297], False), ('RL', [-0.2, 0.1, -0.297], False)):
        p = np.array(p)
        v_rel = -v_base if stance else np.array([0.3, 0.0, 0.05])  # swing: forward relative to the body
        feet[name] = (p, v_rel)
    # Height alone counts all four feet and halves the speed.
    velocity, stance = stance_velocity(feet, np.zeros(3), np.eye(3), stance_height=0.02)
    assert len(stance) == 4 and velocity[0] < 0.1
    # With the previous estimate the swing feet fall outside the gate.
    velocity, stance = stance_velocity(feet, np.zeros(3), np.eye(3), previous=[0.28, 0.0, 0.0])
    assert stance == ['FL', 'RR']
    assert velocity == pytest.approx(v_base, abs=1e-12)


def test_tilt_decides_which_feet_touch():
    # Pitched nose-down 0.2 rad: in the base frame the front feet look higher
    # than they are; gravity alignment must still pick the front feet.
    pitch = 0.2
    orientation = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
                            [-math.sin(pitch), 0, math.cos(pitch)]])
    feet = {'front': (np.array([0.3, 0, -0.30]), np.zeros(3)),
            'rear': (np.array([-0.3, 0, -0.33]), np.zeros(3))}
    _, stance = stance_velocity(feet, np.zeros(3), orientation, stance_height=0.02)
    assert stance == ['front']


def test_odometry_integrates_a_walk_without_truth():
    """Feet planted in the world, base moving: the odometry follows the base."""
    kinematics = LegKinematics(toy_robot())
    odometry = LegOdometry(kinematics, stance_height=0.02)
    # Base moves forward at 0.25 m/s with all feet planted; solve the planar
    # two-link IK for each foot at every step.
    def ik(dx, dz):
        d = math.hypot(dx, dz)
        knee = -2.0 * math.acos(min(1.0, d / 0.4))
        hip = math.atan2(-dx, -dz) - knee / 2.0
        return hip, knee
    dt, speed = 0.02, 0.25
    previous = None
    for step in range(21):  # 0.4 s: the planted feet stay within reach
        t = step * dt
        q = {}
        for name, x in (('FL', 0.2), ('FR', 0.2), ('RL', -0.2), ('RR', -0.2)):
            foot_world_x = x + 0.05  # planted 5 cm ahead of the hip at t=0
            hip, knee = ik(foot_world_x - (x + speed * t), -0.35)
            q[name + '_hip'], q[name + '_knee'] = hip, knee
        qd = {k: (q[k] - previous[k]) / dt for k in q} if previous else {k: 0.0 for k in q}
        previous = q
        odometry.update(t, q, qd, np.eye(3), (0.0, 0.0, 0.0))
    assert odometry.velocity[0] == pytest.approx(speed, abs=1e-3)
    # Joint velocities are backward differences, so each step integrates the
    # velocity over the interval that just ended.
    assert odometry.x == pytest.approx(speed * 20 * dt, abs=1e-4)


@pytest.mark.skipif(not os.path.exists(G1), reason='G1 description not present')
def test_humanoid_feet_are_found_from_the_description():
    kinematics = LegKinematics(open(G1).read())
    assert kinematics.base_link == 'pelvis'
    assert sorted(kinematics.foot_links) == ['left_ankle_roll_link', 'right_ankle_roll_link']
    p, _ = kinematics.foot('left_ankle_roll_link', {}, {})
    assert p[2] < -0.6  # well below the pelvis at the zero pose


def test_quaternion_matrix_matches_a_yaw():
    yaw = 0.7
    assert quaternion_matrix(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(
        np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]))
