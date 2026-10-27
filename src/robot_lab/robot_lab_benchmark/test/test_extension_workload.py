"""Contract tests; fixtures are not robot mission evidence."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_lab_benchmark.extension_contract import SCHEMA, RESULT_SCHEMA, CONTRACTS, endpoint_ok
from robot_lab_benchmark.workload_contract import validate_workload, read_result
from robot_lab_benchmark.concurrent_runner import RunSpec


def specification(**values):
    return validate_workload(dict(schema=SCHEMA,workflow='gait',task='navigation',
        truth_coordinates='selected_world',goals=[[1,2,0]],**values))


@pytest.mark.parametrize('values',[
    dict(workflow='unknown',task='navigation'),dict(workflow='marine',task='formation'),
    dict(workflow='gait',task='navigation',goals=[]),
    dict(workflow='gait',task='mapping',distance_m=float('nan')),
    dict(workflow='aerial_swarm',task='formation',timeout_s=10000),
])
def test_invalid_tasks_rejected(values):
    with pytest.raises(ValueError):validate_workload(dict(schema=SCHEMA,truth_coordinates='selected_world',**values))


def test_physical_motion_and_heading_are_part_of_success():
    c=CONTRACTS['gait']
    assert endpoint_ok([1,2,0],[1,2,0],.01,.01,c)
    assert not endpoint_ok([1.2,2,0],[1,2,0],.01,.01,c)
    assert not endpoint_ok([1,2,.2],[1,2,0],.01,.01,c)
    assert not endpoint_ok([1,2,0],[1,2,0],.1,.01,c)
    assert not endpoint_ok([1,2,0],[1,2,0],None,.01,c)


def test_native_launch_must_match_workload():
    RunSpec(command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','mode:=gait','gait_task:=nav'],workload=specification()).validate()
    with pytest.raises(ValueError):RunSpec(command=['ros2','launch','robot_lab_bringup','gait_simulator.launch.py','task:=slam'],workload=specification()).validate()


def test_result_requires_source_identity_settling_and_release(tmp_path):
    spec=specification()
    (tmp_path/'workload.json').write_text(json.dumps(spec));(tmp_path/'extension-trace.jsonl').write_text('{}\n')
    from robot_lab_benchmark import extension_workload
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    result=dict(schema=RESULT_SCHEMA,workflow='gait',task='navigation',session_id='session',run_id='run',
        ros_domain_id=20,success=True,control_released=True,truth_samples=dict(body=5),goal_contract=CONTRACTS['gait'],
        measurements=[dict(requested=[1,2,0],physical_pose=[1,2,0],linear_m_s=.01,angular_rad_s=.01,
                           settled_sim_s=1.2,native_terminal=True,nav2_status=4)],
        producer_sha256=sha(Path(extension_workload.__file__)),workload_sha256=sha(tmp_path/'workload.json'),
        trace_sha256=sha(tmp_path/'extension-trace.jsonl'))
    path=tmp_path/'extension-mission-result.json'
    path.write_text(json.dumps(result));assert read_result(tmp_path,'run',20,spec)[1] is None
    for key,value in [('control_released',False),('producer_sha256','bad')]:
        path.write_text(json.dumps(dict(result,**{key:value})));assert read_result(tmp_path,'run',20,spec)[0] is None
    result['measurements'][0]['physical_pose']=[3,4,0]
    path.write_text(json.dumps(result));assert read_result(tmp_path,'run',20,spec)[0] is None
