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
    (dict(producer_sha256='0'*64), 'identity'),
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


def test_producer_is_bound_before_actuation_and_retained_in_result(tmp_path):
    producer = hashlib.sha256(Path(contract.__file__).with_name('nav2_workload.py').read_bytes()).hexdigest()
    specification = dict(_spec(), expected_producer_sha256=producer)
    assert contract.verify_producer(specification, producer) == producer
    with pytest.raises(ValueError, match='changed'):
        contract.verify_producer(specification, '0'*64)
    with pytest.raises(ValueError, match='SHA256'):
        contract.validate_workload(dict(_spec(), expected_producer_sha256='wrong'))
    _write(tmp_path)
    # Rebind a document to another startup producer, even with a matching
    # workload file digest: a late file read cannot bless different code.
    (tmp_path/'workload.json').write_text(json.dumps(dict(_spec(), expected_producer_sha256='0'*64)))
    path=tmp_path/'mission-result.json';result=json.loads(path.read_text())
    result['workload_sha256']=hashlib.sha256((tmp_path/'workload.json').read_bytes()).hexdigest()
    path.write_text(json.dumps(result))
    assert 'startup identity' in contract.read_result(tmp_path,'run',7,_spec())[1]


def _fleet_spec():
    return contract.validate_workload(dict(schema=contract.FLEET_WORKLOAD_SCHEMA,
        truth_coordinates='selected_world',fleet_goals=[dict(id='alpha',x=1.,y=0.,yaw=0.)]))


def _write_fleet(directory, **changes):
    (directory/'workload.json').write_text(json.dumps(_fleet_spec()))
    (directory/'fleet-trace.jsonl').write_text('{"kind":"software_contract_fixture"}\n')
    result=dict(schema=contract.FLEET_RESULT_SCHEMA,run_id='run',ros_domain_id=7,
        fleet_id='actual-session',success=True,goal_contract=contract.FLEET_GOAL_CONTRACT,
        owned_cleanup=True,truth_samples=dict(alpha=10),
        member_results=[dict(member_id='alpha',index=0,requested=_fleet_spec()['fleet_goals'][0],
            physical_pose=[1.,0.,0.],linear_m_s=.01,angular_rad_s=.01,settled_sim_s=1.2,
            task='owned-task',source='route:request-id',nav2_terminal=True,success=True)],
        producer_sha256=hashlib.sha256(Path(contract.__file__).with_name('fleet_workload.py').read_bytes()).hexdigest(),
        workload_sha256=hashlib.sha256((directory/'workload.json').read_bytes()).hexdigest(),
        trace_sha256=hashlib.sha256((directory/'fleet-trace.jsonl').read_bytes()).hexdigest())
    result.update(changes)
    (directory/'fleet-mission-result.json').write_text(json.dumps(result))
    return result


def test_fleet_requires_named_real_members_finite_goals_and_selected_world():
    _fleet_spec()
    for goals in ([],[dict(id=0,x=0,y=0,yaw=0)],[dict(id='alpha',x=float('nan'),y=0,yaw=0)],
                  [dict(id='alpha',x=0,y=0,yaw=float('inf'))]):
        with pytest.raises(ValueError):
            contract.validate_workload(dict(schema=contract.FLEET_WORKLOAD_SCHEMA,
                truth_coordinates='selected_world',fleet_goals=goals))


def test_fleet_result_rejects_old_simulated_and_unbound_completion(tmp_path):
    _write_fleet(tmp_path,producer_sha256=None,trace_sha256=None)
    result,error=contract.read_result(tmp_path,'run',7,_fleet_spec())
    assert result is None and 'identity' in error


def test_fleet_result_requires_physical_settling_and_terminal_cleanup(tmp_path):
    _write_fleet(tmp_path)
    result,error=contract.read_result(tmp_path,'run',7,_fleet_spec())
    assert error is None and result['success']
    row=_write_fleet(tmp_path)['member_results'][0]
    for change in (dict(physical_pose=[1.5,0.,0.]),dict(linear_m_s=.1),dict(settled_sim_s=0.),
                   dict(nav2_terminal=False),dict(requested=dict(id='beta',x=1.,y=0.,yaw=0.))):
        _write_fleet(tmp_path,member_results=[dict(row,**change)])
        assert contract.read_result(tmp_path,'run',7,_fleet_spec())[0] is None
    _write_fleet(tmp_path,owned_cleanup=False)
    assert contract.read_result(tmp_path,'run',7,_fleet_spec())[0] is None
