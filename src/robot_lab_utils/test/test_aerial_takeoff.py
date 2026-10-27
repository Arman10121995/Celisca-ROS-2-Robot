"""Unit tests for aerial takeoff envelope checking; no Gazebo runtime required."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
import sys
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.aerial_takeoff import check_takeoff, extend_envelope  # noqa: E402


def _make_scene(shapes):
    return SimpleNamespace(shapes=shapes)


def _make_box(position, size, rotation=None):
    if rotation is None:
        rotation = np.eye(3)
    return dict(type='box', position=np.asarray(position, dtype=float),
                size=np.asarray(size, dtype=float), rotation=rotation)


def _make_plane(position, size, rotation=None):
    if rotation is None:
        rotation = np.eye(3)
    return dict(type='plane', position=np.asarray(position, dtype=float),
                size=np.asarray(size, dtype=float), rotation=rotation)


def test_empty_scene_allows_full_takeoff():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    result = check_takeoff(_make_scene([]), [0., 0., 2.], envelope,
                           desired_rise=3., minimum_rise=.8)
    assert result['rise_m'] == 3.
    assert result['ceiling_limited'] is False
    assert result['start_enu'] == [0., 0., 2.]
    assert result['target_enu'] == [0., 0., 5.]
    assert result['unchecked_support_band_m'] == .05


def test_box_blocks_minimum_takeoff_rejected():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    scene = _make_scene([_make_box([0., 0., 2.], [.5, .5, 10.])])
    with pytest.raises(ValueError, match='blocks the minimum 0.8 m takeoff corridor'):
        check_takeoff(scene, [0., 0., 2.], envelope, desired_rise=3.)


def test_box_offset_allows_takeoff():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    scene = _make_scene([_make_box([5., 0., 0.], [2., 2., 10.])])
    result = check_takeoff(scene, [0., 0., 2.], envelope,
                           desired_rise=3., minimum_rise=.8)
    assert result['rise_m'] == 3.
    assert result['ceiling_limited'] is False


def test_ceiling_limits_takeoff_rise():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    scene = _make_scene([_make_box([0., 0., 5.], [10., 10., .1])])
    result = check_takeoff(scene, [0., 0., 2.], envelope,
                           desired_rise=10., minimum_rise=.8)
    assert result['ceiling_limited'] is True
    assert .8 <= result['rise_m'] < 10.
    assert abs(result['target_enu'][2] - 5.) < 1e-9


def test_support_band_validation():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., .04], resources={})
    with pytest.raises(ValueError, match='Support band must lie within'):
        check_takeoff(_make_scene([]), [0., 0., 2.], envelope,
                      desired_rise=3., support_band=.05)


def test_invalid_inputs_rejected():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    with pytest.raises(ValueError):
        check_takeoff(_make_scene([]), [float('nan'), 0., 2.], envelope)
    bad_env = dict(lower=[-1., -1., 1.], upper=[1., 1., 0.], resources={})
    with pytest.raises(ValueError):
        check_takeoff(_make_scene([]), [0., 0., 2.], bad_env)
    with pytest.raises(ValueError):
        check_takeoff(_make_scene([]), [0., 0., 2.], envelope, desired_rise=.5)


def test_takeoff_result_schema():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    result = check_takeoff(_make_scene([]), [1., 2., 3.], envelope,
                           desired_rise=2.5)
    required_keys = ['start_enu', 'target_enu', 'rise_m', 'requested_rise_m',
                     'ceiling_limited', 'envelope', 'unchecked_support_band_m',
                     'geometry_scope']
    for key in required_keys:
        assert key in result, f'Missing key: {key}'
    assert result['requested_rise_m'] == 2.5
    assert result['rise_m'] == 2.5
    assert result['geometry_scope'].startswith('Static selected-world')


def test_takeoff_timeout_propagates():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    scene = _make_scene([_make_box([0., 0., 5.], [10., 10., .1])])
    canceled = lambda: False
    with pytest.raises(TimeoutError):
        check_takeoff(scene, [0., 0., 2.], envelope,
                      desired_rise=10., timeout=.001, canceled=canceled)


def test_takeoff_returned_target_is_above_start():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    result = check_takeoff(_make_scene([]), [0., 0., 0.], envelope,
                           desired_rise=3.)
    assert result['target_enu'][2] > result['start_enu'][2]
    assert result['rise_m'] > 0


def test_takeoff_with_plane_obstacle():
    envelope = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.], resources={})
    scene = _make_scene([_make_plane([0., 0., 4.], [10., 10.])])
    result = check_takeoff(scene, [0., 0., 2.], envelope,
                           desired_rise=10., minimum_rise=.8)
    assert result['ceiling_limited'] is True
    assert result['rise_m'] < 10.


def test_extend_envelope_grows_radius():
    base = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.],
                source_sha256='abc', resources={})
    points = np.array([[10., 0., 0.], [-10., 0., 0.]])
    extended = extend_envelope(base, points, margin=.06)
    assert extended['lower'][0] <= -10.06
    assert extended['upper'][0] >= 10.06
    assert extended.get('source_sha256') is not None


def test_extend_envelope_rejects_invalid_points():
    base = dict(lower=[-1., -1., 0.], upper=[1., 1., 1.],
                source_sha256='abc', resources={})
    with pytest.raises(ValueError):
        extend_envelope(base, np.array([[1., 2.]]))
    with pytest.raises(ValueError):
        extend_envelope(base, np.array([[float('nan'), 0., 0.]]))
