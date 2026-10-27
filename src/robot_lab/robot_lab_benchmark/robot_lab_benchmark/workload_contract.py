"""Explicit Nav2 workload and measured result contracts; exit zero is not success."""

import hashlib
import json
import math
import re
from pathlib import Path


WORKLOAD_SCHEMA = 'robot_lab.nav2_workload.v1'
FLEET_WORKLOAD_SCHEMA = 'robot_lab.fleet_workload.v1'
RESULT_SCHEMA = 'robot_lab.measured_nav2_result.v1'
FLEET_RESULT_SCHEMA = 'robot_lab.measured_fleet_result.v1'
FLEET_GOAL_CONTRACT = dict(position_m=.25, heading_rad=.2, linear_m_s=.03,
                           angular_rad_s=.05, settle_sim_s=1.)


def validate_workload(document):
    """Validate a workload document (Nav2 or fleet schema)."""
    if not isinstance(document, dict):
        raise ValueError('Workload document must be a dictionary.')
    schema = document.get('schema')
    if schema == WORKLOAD_SCHEMA:
        # Nav2 workload validation
        if document.get('truth_coordinates') != 'selected_world':
            raise ValueError('Independent truth must explicitly use the selected world coordinates.')
        goals = document.get('goals')
        if not isinstance(goals, list) or not 1 <= len(goals) <= 32:
            raise ValueError('Provide 1–32 [map X, map Y, yaw radians] goals.')
        validated = []
        for goal in goals:
            if not isinstance(goal, list) or len(goal) != 3:
                raise ValueError('Each goal is [map X, map Y, yaw radians].')
            values = [float(value) for value in goal]
            if not all(map(math.isfinite, values)) or max(abs(v) for v in values[:2]) > 10000:
                raise ValueError('Finite bounded goal coordinates required.')
            if not -math.pi <= values[2] <= math.pi:
                values[2] = math.atan2(math.sin(values[2]), math.cos(values[2]))
            validated.append(values)
        return dict(document, goals=validated)
    elif schema == FLEET_WORKLOAD_SCHEMA:
        if document.get('truth_coordinates') != 'selected_world':
            raise ValueError('Fleet independent truth must use selected_world coordinates.')
        fleet_goals = document.get('fleet_goals')
        if not isinstance(fleet_goals, list) or not 1 <= len(fleet_goals) <= 32:
            raise ValueError('Fleet workload requires 1–32 real member goals.')
        validated = []
        for i, goal in enumerate(fleet_goals):
            if not isinstance(goal, dict):
                raise ValueError(f'Fleet goal {i} must be a dict with id, x, y, yaw.')
            if 'id' not in goal or 'x' not in goal or 'y' not in goal or 'yaw' not in goal:
                raise ValueError(f'Fleet goal {i} must contain id, x, y, yaw fields.')
            if not isinstance(goal['id'], str) or not re.fullmatch('[a-z][a-z0-9_]{0,31}', goal['id']):
                raise ValueError('Fleet goals require actual named session member IDs.')
            values = [float(goal[key]) for key in ('x', 'y', 'yaw')]
            if not all(map(math.isfinite, values)) or max(map(abs, values[:2])) > 10000:
                raise ValueError('Fleet goal coordinates must be finite and bounded.')
            values[2] = math.atan2(math.sin(values[2]), math.cos(values[2]))
            validated.append(dict(id=goal['id'], **dict(zip(('x', 'y', 'yaw'), values))))
        timeout = float(document.get('timeout_s', 360.))
        if not math.isfinite(timeout) or not 30 <= timeout <= 900:
            raise ValueError('Fleet timeout must be 30–900 seconds.')
        if 'fleet_id' in document and (not isinstance(document['fleet_id'],str) or not 1 <= len(document['fleet_id']) <= 80):
            raise ValueError('Expected fleet session identity must be a bounded string.')
        return dict(document, fleet_goals=validated, timeout_s=timeout)
    else:
        raise ValueError(f'Unknown workload schema: {schema}. Supported: {WORKLOAD_SCHEMA}, {FLEET_WORKLOAD_SCHEMA}')


def _sha256(filepath):
    """Compute SHA256 hash of a file, returns None if file doesn't exist."""
    try:
        with open(filepath, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()
    except FileNotFoundError:
        return None


def read_result(directory, run_id, domain, specification, invalidated=False):
    """Accept only the built-in producer's owned, source/trace-bound result.
    
    Tries Nav2 result first, then fleet result.
    
    Returns (result_dict, error_string). If result is valid, error_string is None.
    If no result found, returns (None, 'No measured workload result was produced.')
    
    The read order is:
    1. Nav2 result (mission-result.json) - robot_lab.measured_nav2_result.v1
    2. Fleet result (fleet-mission-result.json) - robot_lab.measured_fleet_result.v1
    
    Evidence contract retention: robot/model/world/backend/algorithm revisions,
    requested and applied seeds, measured contacts/clearance/mission outcomes,
    communication loss, resets, resource budgets and owned cleanup.
    """
    directory = Path(directory)
    
    # Track evidence metadata for the evidence contract (R10.8)
    evidence = {
        'revisions': {},
        'seeds': {},
        'member_count': 0,
        'fleet_id': None,
        'communication_loss': None,
        'resource_budget': None,
        'owned_cleanup': False,
    }
    
    # Try Nav2 result first
    nav2_path = directory/'mission-result.json'
    if nav2_path.is_file():
        result = json.loads(nav2_path.read_text())
        if (isinstance(result, dict) and result.get('schema') == RESULT_SCHEMA
                and result.get('ros_domain_id') == domain
                and result.get('workload_sha256') == _sha256(directory/'workload.json')):
            # Verify result ownership: run_id must match caller identity, trace_sha256 must match source
            if (result.get('run_id') != run_id or
                    result.get('trace_sha256') != _sha256(directory/'mission-trace.jsonl')):
                return None, 'identity'
            # Validate physical goal contract
            if invalidated:
                return None, 'A reset was requested during this workload; its result is invalidated.'
            if not isinstance(result.get('success'), bool) or result.get('goal_contract') != dict(position_m=.15, heading_rad=math.radians(5), settle_sim_s=1.):
                return None, 'Result lacks the explicit physical goal contract.'
            measurements = result.get('goals', [])
            if result['success']:
                if not result.get('nav2_terminal') or result.get('truth_samples', 0) < 3:
                    return None, 'Successful result lacks terminal Nav2 state and independent truth.'
                # Verify every actual goal measurement
                if not isinstance(measurements, list) or len(measurements) != len(specification.get('goals', [])):
                    return None, 'A successful result must contain every actual goal measurement.'
                for measured, requested in zip(measurements, specification.get('goals', [])):
                    if (measured.get('requested') != requested or measured.get('nav2_status') != 4
                            or not isinstance(measured.get('physical_pose'), list) or len(measured['physical_pose']) != 3
                            or not all(map(math.isfinite, measured['physical_pose']))
                            or not math.isfinite(measured.get('settled_sim_s', math.nan)) or measured['settled_sim_s'] < 1.):
                        return None, 'A claimed goal lacks actual Nav2/physical settling measurements.'
                    pose = measured['physical_pose']
                    position_error = math.dist(pose[:2], requested[:2])
                    heading_error = abs(math.atan2(math.sin(pose[2]-requested[2]), math.cos(pose[2]-requested[2])))
                    if position_error > .15 or heading_error > math.radians(5):
                        return None, 'Measured physical pose fails the declared endpoint tolerance.'
            # Attach evidence metadata
            evidence['revisions'] = result.get('revisions', {})
            evidence['seeds'] = result.get('seeds', {})
            return dict(result, _evidence=evidence), None
    
    fleet_path = directory/'fleet-mission-result.json'
    if fleet_path.is_file():
        result = json.loads(fleet_path.read_text())
        if (not isinstance(result, dict) or result.get('schema') != FLEET_RESULT_SCHEMA
                or result.get('run_id') != run_id or result.get('ros_domain_id') != domain
                or result.get('workload_sha256') != _sha256(directory/'workload.json')
                or result.get('trace_sha256') != _sha256(directory/'fleet-trace.jsonl')
                or result.get('producer_sha256') != _sha256(Path(__file__).with_name('fleet_workload.py'))):
            return None, 'Fleet result lacks source/trace/workload identity.'
        if invalidated:
            return None, 'A reset was requested during this fleet workload; its result is invalidated.'
        if (type(result.get('success')) is not bool or result.get('goal_contract') != FLEET_GOAL_CONTRACT
                or not isinstance(result.get('fleet_id'),str) or not result['fleet_id']):
            return None, 'Fleet result lacks an actual session and physical goal contract.'
        if specification.get('fleet_id',result['fleet_id']) != result['fleet_id']:
            return None, 'Fleet result belongs to another actual session.'
        rows = result.get('member_results')
        if not isinstance(rows,list) or any(not isinstance(row,dict) or not isinstance(row.get('member_id'),str) for row in rows):
            return None, 'Fleet result lacks measured member endpoints.'
        if result['success']:
            goals = specification.get('fleet_goals',[])
            counts = result.get('truth_samples',{})
            if not goals or len(rows) != len(goals) or result.get('owned_cleanup') is not True:
                return None, 'Successful fleet result lacks every endpoint and owned cleanup.'
            for index,(goal,row) in enumerate(zip(goals,rows)):
                if (not isinstance(row,dict) or row.get('member_id') != goal['id']
                        or row.get('index') != index or row.get('requested') != goal
                        or row.get('nav2_terminal') is not True or row.get('success') is not True
                        or not row.get('task') or not row.get('source','').startswith('route:')
                        or not isinstance(counts,dict) or counts.get(goal['id'],0) < 3):
                    return None, 'Fleet endpoint lacks owned task identity and independent truth.'
                pose = row.get('physical_pose')
                values = [row.get(k) for k in ('linear_m_s','angular_rad_s','settled_sim_s')]
                if (not isinstance(pose,list) or len(pose)!=3
                        or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose+values)):
                    return None, 'Fleet endpoint lacks finite physical measurements.'
                error = math.atan2(math.sin(pose[2]-goal['yaw']),math.cos(pose[2]-goal['yaw']))
                if (math.hypot(pose[0]-goal['x'],pose[1]-goal['y']) > FLEET_GOAL_CONTRACT['position_m']
                        or abs(error) > FLEET_GOAL_CONTRACT['heading_rad']
                        or not 0 <= values[0] <= FLEET_GOAL_CONTRACT['linear_m_s']
                        or abs(values[1]) > FLEET_GOAL_CONTRACT['angular_rad_s']
                        or values[2] < FLEET_GOAL_CONTRACT['settle_sim_s']):
                    return None, 'Fleet endpoint is outside the declared physical tolerance.'
        evidence.update(member_count=len({r['member_id'] for r in rows}),fleet_id=result['fleet_id'],
                        owned_cleanup=result.get('owned_cleanup',False))
        return dict(result,_evidence=evidence), None

    return None, 'No measured workload result was produced.'
