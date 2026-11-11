"""Reject swept collisions even when sampled endpoints and action results look clear."""
import copy
import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts/r9_3_support_matrix.py'
spec = importlib.util.spec_from_file_location('camera_clearance_screen', SCRIPT)
screen = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = screen
spec.loader.exec_module(screen)


def report():
    points = [[-.5, 1., 0.]] * 5 + [[.5, 1., 0.]] * 5
    return {'result': 'completed', 'coverage_pct': 99., 'mode': 'zones', 'final_state': 'docked',
            'dock': points[-1], 'final_truth': points[-1], 'robot_radius': .165,
            'trail': [{'truth': p, 'truth_age_s': .03} for p in points], 'final_truth_age_s': .03,
            'object_truth': {'manifest': 'world.objects.json', 'sha256': 'a' * 64,
                             'objects': [{'class': 'shoe', 'x': 0., 'y': 0.,
                                          'footprint': {'kind': 'box', 'half_extents': [.14, .055],
                                                        'yaw': 0.}}]}}


def test_clear_job_has_independently_derived_body_clearance():
    data = report()
    assert screen.cleaning_screen(data)
    measured = screen.cleaning_object_clearance(data)
    assert measured['measured'] and abs(measured['minimum_body_clearance_m'] - .78) < 1e-9


def test_between_sample_collision_fails_despite_clear_endpoints():
    data = report()
    for sample in data['trail']:
        sample['truth'] = [sample['truth'][0], 0., 0.]
    data['final_truth'] = data['dock'] = [.5, 0., 0.]
    assert screen.cleaning_object_clearance(data)['minimum_body_clearance_m'] == -.165
    assert not screen.cleaning_screen(data)


def test_rotated_box_circle_and_missing_truth_cannot_claim_pass():
    data = report()
    obj = data['object_truth']['objects'][0]
    obj['footprint']['yaw'] = 1.5707963267948966
    assert abs(screen.cleaning_object_clearance(data)['minimum_body_clearance_m'] - .695) < 1e-9
    obj['footprint'] = {'kind': 'circle', 'radius': .025}
    assert abs(screen.cleaning_object_clearance(data)['minimum_body_clearance_m'] - .81) < 1e-9
    for mutated in (dict(data, robot_radius=float('nan')), dict(data, object_truth={})):
        assert not screen.cleaning_screen(mutated)
    assert not screen.cleaning_screen(dict(data, final_truth_age_s=2.))
    broken = copy.deepcopy(data)
    broken['trail'][3]['truth'] = None
    assert not screen.cleaning_screen(broken)
    historical = dict(data)
    del historical['object_truth']
    assert not screen.cleaning_object_clearance(historical)['measured']
    assert screen.cleaning_screen(historical)
