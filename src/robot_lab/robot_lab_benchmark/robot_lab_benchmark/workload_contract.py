"""Explicit Nav2 workload and measured result contracts; exit zero is not success."""

import hashlib
import json
import math
from pathlib import Path


WORKLOAD_SCHEMA = 'robot_lab.nav2_workload.v1'
FLEET_WORKLOAD_SCHEMA = 'robot_lab.fleet_workload.v1'
RESULT_SCHEMA = 'robot_lab.measured_nav2_result.v1'
FLEET_RESULT_SCHEMA = 'robot_lab.measured_fleet_result.v1'


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
        # Fleet/swarm experiment workload validation
        fleet_goals = document.get('fleet_goals')
        if not isinstance(fleet_goals, list):
            raise ValueError('Fleet workload requires fleet_goals list.')
        for i, goal in enumerate(fleet_goals):
            if not isinstance(goal, dict):
                raise ValueError(f'Fleet goal {i} must be a dict with id, x, y, yaw.')
            if 'id' not in goal or 'x' not in goal or 'y' not in goal or 'yaw' not in goal:
                raise ValueError(f'Fleet goal {i} must contain id, x, y, yaw fields.')
            if not isinstance(goal['x'], (int, float)) or not isinstance(goal['y'], (int, float)):
                raise ValueError(f'Fleet goal {i} x and y must be numeric.')
            if not -math.pi <= goal.get('yaw', 0) <= math.pi:
                raise ValueError(f'Fleet goal {i} yaw must be within [-pi, pi].')
        return dict(document, fleet_goals=fleet_goals)
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
        'communication_loss': False,
        'resource_budget': None,
        'owned_cleanup': False,
    }
    
    # Try Nav2 result first
    nav2_path = directory/'mission-result.json'
    if nav2_path.is_file():
        result = json.loads(nav2_path.read_text())
        if (isinstance(result, dict) and result.get('schema') == RESULT_SCHEMA
                and result.get('run_id') == run_id and result.get('ros_domain_id') == domain
                and result.get('workload_sha256') == _sha256(directory/'workload.json')
                and result.get('trace_sha256') == _sha256(directory/'mission-trace.jsonl')):
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
    
    # Try fleet result
    fleet_path = directory/'fleet-mission-result.json'
    if fleet_path.is_file():
        # Validate fleet result schema and source identity
        result = json.loads(fleet_path.read_text())
        # Check required fields exist
        has_schema = result.get('schema') == FLEET_RESULT_SCHEMA
        has_run_id = result.get('run_id') == run_id
        has_domain = result.get('ros_domain_id') == domain
        has_fleet_id = 'fleet_id' in result
        has_member_results = isinstance(result.get('member_results'), list)
        
        # Validate member results fields
        member_fields_ok = True
        if has_member_results:
            for mr in result['member_results']:
                if not isinstance(mr, dict):
                    member_fields_ok = False
                    break
                if 'member_id' not in mr:
                    member_fields_ok = False
                    break
                if 'success' not in mr:
                    member_fields_ok = False
                    break
                if 'physical_pose' not in mr or not isinstance(mr['physical_pose'], list) or len(mr['physical_pose']) != 3:
                    member_fields_ok = False
                    break
                if not all(map(math.isfinite, mr['physical_pose'])):
                    member_fields_ok = False
                    break
        
        if (has_schema and has_run_id and has_domain and has_fleet_id and has_member_results and member_fields_ok):
            # Validate result
            if invalidated:
                return None, 'A reset was requested during this workload; its result is invalidated.'
            if not isinstance(result.get('success'), bool):
                return None, 'Fleet result must have success field.'
            # Attach comprehensive evidence metadata (R10.8 evidence contract)
            evidence['revisions'] = result.get('revisions', {})
            evidence['seeds'] = result.get('seeds', {})
            evidence['member_count'] = len(result.get('member_results', []))
            evidence['fleet_id'] = result.get('fleet_id')
            # Fleet-specific evidence fields
            evidence['communication_loss'] = result.get('communication_loss', False)
            evidence['resource_budget'] = result.get('resource_budget')
            evidence['owned_cleanup'] = result.get('owned_cleanup', False)
            # Return result with evidence attached
            result_with_evidence = dict(result, _evidence=evidence)
            return result_with_evidence, None
    
    return None, 'No measured workload result was produced.'
