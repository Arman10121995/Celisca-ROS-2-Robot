"""Contract tests for extension result validation; fixtures are not robot mission evidence."""
import hashlib
import json
import sqlite3
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_lab_benchmark.extension_contract import RESULT_SCHEMA, CONTRACTS
from robot_lab_benchmark.workload_contract import validate_workload, read_result


def _make_spec(workflow='gait', task='navigation', **extra):
    doc = dict(schema='robot_lab.extension_workload.v1',
        workflow=workflow, task=task, truth_coordinates='selected_world')
    if task == 'navigation':
        doc['goals'] = [[1, 2, 0]]
    if task == 'mapping':
        doc['distance_m'] = 2.0
    doc.update(extra)
    return validate_workload(doc)


def _sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _base_result(tmp_path, spec, success=True, control_released=True,
                 workload_path=None, trace_path=None, producer_path=None):
    workload_path = workload_path or tmp_path / 'workload.json'
    trace_path = trace_path or tmp_path / 'extension-trace.jsonl'
    producer_path = producer_path or Path(__file__).resolve().parents[1] / 'robot_lab_benchmark' / 'extension_workload.py'
    workload_path.write_text(json.dumps(spec))
    trace_path.write_text('{}\n')
    result = dict(schema=RESULT_SCHEMA, workflow=spec['workflow'], task=spec['task'],
        session_id='session-xyz', run_id='run-123', ros_domain_id=20,
        success=success, control_released=control_released,
        truth_samples=dict(body=5), goal_contract=CONTRACTS[spec['workflow']],
        measurements=[
            dict(requested=[1, 2, 0], physical_pose=[1, 2, 0], linear_m_s=.01,
                 angular_rad_s=.01, settled_sim_s=1.2, native_terminal=True, nav2_status=4)],
        producer_sha256=_sha(producer_path),
        workload_sha256=_sha(workload_path),
        trace_sha256=_sha(trace_path))
    path = tmp_path / 'extension-mission-result.json'
    path.write_text(json.dumps(result))
    return tmp_path, result


def test_missing_result_file_reported(tmp_path):
    spec = _make_spec()
    (tmp_path / 'workload.json').write_text(json.dumps(spec))
    (tmp_path / 'extension-trace.jsonl').write_text('{}\n')
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'No measured extension result' in error


def test_invalid_schema_rejected(tmp_path):
    spec = _make_spec()
    tmp_path, result = _base_result(tmp_path, spec)
    result['schema'] = 'wrong'
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'identity' in error


def test_identity_mismatch_rejected(tmp_path):
    spec = _make_spec()
    for field in ('run_id', 'ros_domain_id', 'workflow', 'task'):
        tmp_path, result = _base_result(tmp_path, spec)
        bad = dict(result, **{field: 'changed'})
        (tmp_path / 'extension-mission-result.json').write_text(json.dumps(bad))
        result, error = read_result(tmp_path, 'run-123', 20, spec, False)
        assert result is None and 'identity' in error, f'{field} mismatch not rejected'




def test_missing_success_field_rejected(tmp_path):
    spec = _make_spec()
    tmp_path, result = _base_result(tmp_path, spec)
    result.pop('success')
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'outcome' in error


def test_wrong_contract_rejected(tmp_path):
    spec = _make_spec()
    tmp_path, result = _base_result(tmp_path, spec)
    result['goal_contract'] = dict(position_m=.99, linear_m_s=.99, settle_sim_s=.99)
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'contract' in error


def test_success_requires_control_release_and_observations(tmp_path):
    spec = _make_spec()
    tmp_path, result_dict = _base_result(tmp_path, spec)
    result_dict['control_released'] = False
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result_dict))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'control release' in error

    result_dict['control_released'] = True
    result_dict['truth_samples'] = {}
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result_dict))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'observations' in error


def test_mapping_requires_actual_travel_and_map_artifact(tmp_path):
    spec = _make_spec(task='mapping', distance_m=2.0)
    tmp_path, result_dict = _base_result(tmp_path, spec, success=True)
    result_dict['measurements'] = [dict(distance_m=1.0, physical_start=[0.,0.,-2.], physical_pose=[1.,0.,-2.], map_artifact={},
        observation=dict(kind='database', nodes=5, sha256='abc'))]
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result_dict))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'travel' in error

    result_dict['measurements'] = [dict(distance_m=2.5, physical_start=[0.,0.,-2.], physical_pose=[2.5,0.,-2.], map_artifact={'database': '/tmp/no.db'},
        observation=dict(kind='database', nodes=5, sha256='abc'))]
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result_dict))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'SSD' in error


def test_navigation_requires_every_goal_measured(tmp_path):
    spec = _make_spec(goals=[[1, 2, 0], [3, 4, 0]])
    tmp_path, result_dict = _base_result(tmp_path, spec)
    result_dict['measurements'] = [result_dict['measurements'][0]]
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result_dict))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'goal' in error


def test_formation_requires_every_member_measured(tmp_path):
    spec = _make_spec(workflow='aerial_swarm', task='formation')
    tmp_path, result_dict = _base_result(tmp_path, spec, success=True)
    result_dict['truth_samples'] = dict(air1=5, air2=5)
    result_dict['measurements'] = [dict(member_id='air1', requested=[0, 0, 2],
        physical_pose=[0, 0, 2], linear_m_s=.01, angular_rad_s=.01,
        settled_sim_s=2.0)]
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result_dict))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'member' in error


def test_endpoint_outside_tolerance_rejected(tmp_path):
    spec = _make_spec()
    tmp_path, result_dict = _base_result(tmp_path, spec)
    result_dict['measurements'][0]['physical_pose'] = [5, 6, 0]
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result_dict))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'contract' in error


def test_inspect_map_database_requires_nodes(tmp_path):
    from robot_lab_benchmark.extension_result import inspect_map
    db = tmp_path / 'empty.db'
    with sqlite3.connect(db) as conn:
        conn.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY)')
    with pytest.raises(ValueError, match='No acquired'):
        inspect_map({'database': str(db)})


def test_inspect_map_occupancy_requires_known_cells(tmp_path):
    from robot_lab_benchmark.extension_result import inspect_map
    import numpy as np
    from PIL import Image
    spec = _make_spec()
    yaml_path = tmp_path / 'map.yaml'
    img_path = tmp_path / 'map.pgm'
    yaml_path.write_text('image: map.pgm\nresolution: .1\norigin: [0, 0, 0]\n')
    Image.fromarray(np.full((10, 10), 205, dtype=np.uint8)).save(img_path)
    with pytest.raises(ValueError, match='No acquired'):
        inspect_map({'map_yaml': str(yaml_path)})


def test_successful_gait_navigation_accepted(tmp_path):
    spec = _make_spec()
    tmp_path, result_dict = _base_result(tmp_path, spec)
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert error is None and result is not None and result['success'] is True


def test_missing_session_id_rejected(tmp_path):
    spec = _make_spec()
    tmp_path, result = _base_result(tmp_path, spec)
    result.pop('session_id')
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'identity' in error


def test_hash_mismatch_rejected(tmp_path):
    spec = _make_spec()
    tmp_path, result = _base_result(tmp_path, spec)
    bad_workload = tmp_path / 'bad-workload.json'
    bad_workload.write_text(json.dumps({'schema': 'x'}))
    result['workload_sha256'] = _sha(bad_workload)
    (tmp_path / 'extension-mission-result.json').write_text(json.dumps(result))
    result, error = read_result(tmp_path, 'run-123', 20, spec, False)
    assert result is None and 'identity' in error


def test_invalidated_result_rejected(tmp_path):
    spec = _make_spec()
    tmp_path, result = _base_result(tmp_path, spec)
    result, error = read_result(tmp_path, 'run-123', 20, spec, True)
    assert result is None and 'Reset invalidated' in error
