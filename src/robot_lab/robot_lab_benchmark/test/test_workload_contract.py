"""The Nav2 workload/result contract: only a source/trace-bound result counts."""
import hashlib
import json
import math
import sys
from pathlib import Path

import pytest

_benchmark_pkg = Path(__file__).resolve().parent.parent
if str(_benchmark_pkg) not in sys.path:
    sys.path.insert(0, str(_benchmark_pkg))

from robot_lab_benchmark import workload_contract as contract  # noqa: E402

GOALS = [[1.0, 2.0, 0.0], [3.0, 1.0, 1.5]]


def _spec():
    return contract.validate_workload(dict(
        schema=contract.WORKLOAD_SCHEMA, truth_coordinates='selected_world', goals=GOALS))


def _write(directory, success=True, errors=(0.05, 0.05), **overrides):
    (directory/'workload.json').write_text(json.dumps(_spec()))
    (directory/'mission-trace.jsonl').write_text('{"kind": "truth"}\n')
    producer = Path(contract.__file__).with_name('nav2_workload.py')
    goals = [dict(requested=goal, physical_pose=[goal[0]+error, goal[1], goal[2]],
                  nav2_status=4, settled_sim_s=1.2) for goal, error in zip(GOALS, errors)]
    result = dict(schema=contract.RESULT_SCHEMA, run_id='run', ros_domain_id=7, success=success,
                  goal_contract=dict(position_m=.15, heading_rad=math.radians(5), settle_sim_s=1.),
                  goals=goals, nav2_terminal=True, truth_samples=50,
                  workload_sha256=hashlib.sha256((directory/'workload.json').read_bytes()).hexdigest(),
                  producer_sha256=hashlib.sha256(producer.read_bytes()).hexdigest(),
                  trace_sha256=hashlib.sha256((directory/'mission-trace.jsonl').read_bytes()).hexdigest())
    result.update(overrides)
    (directory/'mission-result.json').write_text(json.dumps(result))


def test_workload_goals_are_validated_and_yaw_wrapped():
    spec = contract.validate_workload(dict(
        schema=contract.WORKLOAD_SCHEMA, truth_coordinates='selected_world', goals=[[0, 0, 7.0]]))
    assert spec['goals'][0][2] == pytest.approx(math.atan2(math.sin(7.0), math.cos(7.0)))
    for bad in ([], [[0, 0]], [[math.inf, 0, 0]], [[0, 0, 0]]*33):
        with pytest.raises(ValueError):
            contract.validate_workload(dict(schema=contract.WORKLOAD_SCHEMA,
                                            truth_coordinates='selected_world', goals=bad))
    with pytest.raises(ValueError):
        contract.validate_workload(dict(schema=contract.WORKLOAD_SCHEMA, goals=GOALS))


def test_a_bound_successful_result_is_accepted(tmp_path):
    _write(tmp_path)
    result, error = contract.read_result(tmp_path, 'run', 7, _spec())
    assert error is None and result['success']


@pytest.mark.parametrize('change, message', [
    (dict(run_id='other'), 'identity'),
    (dict(trace_sha256='0'*64), 'identity'),
    (dict(goal_contract=dict(position_m=.3, heading_rad=1, settle_sim_s=0)), 'goal contract'),
    (dict(truth_samples=1), 'independent truth'),
])
def test_unbound_or_weak_results_are_refused(tmp_path, change, message):
    _write(tmp_path, **change)
    result, error = contract.read_result(tmp_path, 'run', 7, _spec())
    assert result is None and message in error


def test_a_claimed_success_outside_tolerance_is_refused(tmp_path):
    _write(tmp_path, errors=(0.05, 0.2))
    result, error = contract.read_result(tmp_path, 'run', 7, _spec())
    assert result is None and 'tolerance' in error


def test_reset_invalidates_and_missing_result_is_reported(tmp_path):
    assert contract.read_result(tmp_path, 'run', 7, _spec())[1] == 'No measured workload result was produced.'
    _write(tmp_path)
    result, error = contract.read_result(tmp_path, 'run', 7, _spec(), invalidated=True)
    assert result is None and 'reset' in error
