"""Numerical and loss contracts, distinct from physical power qualification."""
import hashlib
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_energy import EnergyEstimate, qualified_source

CONFIG = dict(capacity_J=1000., initial_fraction=.8, hotel_W=3.,
              drive_efficiency=.5, charge_W=100.)


def test_complete_joints_and_signed_work():
    model = EnergyEstimate(CONFIG, ['left', 'right'], True)
    for stamp in (1., 1.5):
        model.observe(['right', 'left'], [-2., 2.], [-3., -3.], stamp, stamp, stamp)
    # Positive drive work on right; left is braking, no regeneration modeled.
    assert model.state(1.5)['power_W'] == 15.
    assert model.joules == 792.5


@pytest.mark.parametrize('names,v,e', [(['left'], [1.], [2.]),
    (['left', 'right'], [1., 2.], []),
    (['left', 'left'], [1., 2.], [3., 4.]),
    (['left', 'right'], [1., float('nan')], [3., 4.])])
def test_missing_duplicate_nonfinite_feedback_invalidates_coverage(names, v, e):
    model = EnergyEstimate(CONFIG, ['left', 'right'], True)
    for stamp in (1., 1.5):
        model.observe(['left', 'right'], [0., 0.], [0., 0.], stamp, stamp, stamp)
    model.observe(names, v, e, 1.6, 1.6, 1.6)
    assert model.state(1.6)['fraction'] is None
    assert model.coverage_lost


def test_duplicate_does_not_refresh_lease():
    model = EnergyEstimate(CONFIG, ['left'], True)
    model.observe(['left'], [1.], [2.], 1., 1., 1.)
    model.observe(['left'], [1.], [2.], 1.5, 1.5, 1.5)
    model.observe(['left'], [1.], [2.], 1.5, 1.5, 2.2)
    assert model.state(2.2)['fraction'] is None


def test_source_certificate_binds_description_and_runtime(tmp_path):
    library = tmp_path / 'plugin.so'
    library.write_bytes(b'qualification-fixture')
    member = dict(source_description_sha256='description-A')
    certificate = dict(source_description_sha256='description-A', evidence='named-measurement',
        runtime_library=str(library), runtime_library_sha256=hashlib.sha256(library.read_bytes()).hexdigest())
    assert qualified_source(member, certificate)
    assert not qualified_source(dict(source_description_sha256='changed'), certificate)
    library.write_bytes(b'new-runtime')
    assert not qualified_source(member, certificate)


def test_floor_frame_preserves_localization_owner():
    from robot_lab_utils.fleet_session import navigation_parameters
    member = dict(id='alpha', model='bumperbot', map_frame='alpha/map', spawn={},
                  max_linear=.25, max_angular=.5, radius=.118)
    params = navigation_parameters(ROOT / 'src/robot_lab_navigation/config',
        ROOT / 'src/robot_lab_localization/config', member, '/floor.yaml')['/alpha']
    assert params['amcl']['ros__parameters']['global_frame_id'] == 'alpha/map'
    assert params['amcl']['ros__parameters']['tf_broadcast'] is True
    assert params['bt_navigator']['ros__parameters']['global_frame'] == 'alpha/map'
