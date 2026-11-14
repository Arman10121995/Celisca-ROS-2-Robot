"""Booster T1 vendor-policy adapter conventions (R3.6)."""
import numpy as np
import pytest

pytest.importorskip("mujoco")

from robot_lab_mujoco.booster_policy import _gains, gravity_in_body  # noqa: E402
from robot_lab_mujoco.unitree_policy import projected_gravity  # noqa: E402


def test_gravity_matches_the_vendor_quat_rotate_inverse():
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        # booster_gym rotates by (x, y, z, w); Unitree's closed form is the same projection.
        np.testing.assert_allclose(gravity_in_body(q), projected_gravity(q), atol=1e-6)
    np.testing.assert_allclose(gravity_in_body([1, 0, 0, 0]), [0, 0, -1])


def test_gains_follow_booster_substring_matching():
    names = ['Left_Hip_Pitch', 'Left_Knee_Pitch', 'Left_Ankle_Roll']
    np.testing.assert_allclose(_gains({'Hip': 200., 'Knee': 200., 'Ankle': 50.}, names, 'stiffness'), [200, 200, 50])
    np.testing.assert_allclose(_gains({'Hip_Pitch': -0.2, 'Knee_Pitch': 0.4, 'default': 0.}, names, 'angle'),
                               [-0.2, 0.4, 0.])
    with pytest.raises(ValueError):
        _gains({'Hip': 1.}, names, 'stiffness')
