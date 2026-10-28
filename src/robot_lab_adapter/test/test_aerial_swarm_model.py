import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_lab_adapter.aerial_swarm_model import identities, velocities


@pytest.mark.parametrize('count', [2, 3, 4])
def test_identities_2_to_4(count):
    members = identities(count)
    assert len(members) == count
    for item, index in zip(members, range(count)):
        assert item['id'] == 'air' + str(index + 1)
        assert item['instance'] == index
        assert item['system_id'] == index + 1
        assert item['port'] == 14580 + index
        assert item['entity'] == 'x500_' + str(index)
        assert item['namespace'] == '/swarm/air' + str(index + 1)


@pytest.mark.parametrize('bad', [1, 5, 0, 2.5, '2', None])
def test_identities_rejects_out_of_range(bad):
    with pytest.raises(ValueError, match='2–4'):
        identities(bad)


def test_identities_uses_fresh_dicts():
    members = identities(2)
    members[0]['port'] = 9999
    assert members[1]['port'] == 14581


@pytest.mark.parametrize('mode,positions,expected', [
    ('formation', {'a': [0., 0., 0.], 'b': [0., 6., 0.]},
     {'a': [0., 0.75, 0.], 'b': [0., -0.75, 0.]}),
    ('rendezvous', {'a': [10., 0., 0.], 'b': [-10., 0., 0.]},
     {'a': [-1., 0., 0.], 'b': [1., 0., 0.]}),
])
def test_velocities_formation_and_rendezvous(mode, positions, expected):
    result = velocities(mode, positions, spacing=3., maximum=1.)
    assert set(result) == set(expected)
    for name in result:
        assert len(result[name]) == 3
        assert result[name] == pytest.approx(expected[name], abs=1e-6)


def test_velocities_dispersion_opens_bounded():
    positions = {'a': [0., 0., 0.], 'b': [1., 0., 0.], 'c': [0., 1., 0.]}
    result = velocities('dispersion', positions, spacing=3., maximum=1.)
    for name, v in result.items():
        assert v != [0., 0., 0.]
        assert np.linalg.norm(v) <= 1.0 + 1e-6
    def dist(x, y):
        return float(np.linalg.norm(np.array(x) - np.array(y)))
    before = sum(dist(positions[x], positions[y])
                 for x, y in [('a', 'b'), ('a', 'c'), ('b', 'c')])
    after = sum(dist(result[x], result[y]) for x, y in [('a', 'b'), ('a', 'c'), ('b', 'c')])
    assert after > before


def test_velocities_flocking_include_forward_bias():
    positions = {'a': [0., 0., 0.], 'b': [1., 0., 0.]}
    result = velocities('flocking', positions, spacing=3., maximum=1.)
    assert 'a' in result and 'b' in result
    assert result['a'][0] > 0 or result['b'][0] > 0


@pytest.mark.parametrize('mode', ['cooperative_coverage', 'merge', 'hold', 'hover'])
def test_velocities_rejects_unsupported_kernel(mode):
    with pytest.raises(ValueError):
        velocities(mode, {'a': [0., 0., 0.], 'b': [0., 3., 0.]})


@pytest.mark.parametrize('positions', [
    {'a': [0., 0., 0.], 'b': [float('nan'), 0., 0.]},
    {'a': [0., 0., 0.]},
    {'a': [0., 0., 0.], 'b': [0., 3., 0.], 'c': [0., 6., 0.], 'd': [0., 9., 0.], 'e': [0., 12., 0.]},
])
def test_velocities_rejects_invalid_positions(positions):
    with pytest.raises(ValueError):
        velocities('formation', positions, spacing=3.)
