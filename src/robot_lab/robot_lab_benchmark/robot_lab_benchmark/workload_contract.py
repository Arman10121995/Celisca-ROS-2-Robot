"""Explicit Nav2 workload and measured result contracts; exit zero is not success."""
import hashlib
import json
import math
from pathlib import Path


WORKLOAD_SCHEMA = 'robot_lab.nav2_workload.v1'
RESULT_SCHEMA = 'robot_lab.measured_nav2_result.v1'


def validate_workload(document):
    if not isinstance(document, dict) or document.get('schema') != WORKLOAD_SCHEMA:
        raise ValueError('Choose a Robot Lab Nav2 workload document.')
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


def read_result(directory, run_id, domain, specification, invalidated=False):
    """Accept only the built-in producer's owned, source/trace-bound result."""
    directory = Path(directory)
    path = directory/'mission-result.json'
    if not path.is_file():
        return None, 'No measured workload result was produced.'
    if path.stat().st_size > 2*1024*1024:
        return None, 'Workload result exceeds the 2 MB bound.'
    result = json.loads(path.read_text())
    producer = Path(__file__).with_name('nav2_workload.py')
    trace = directory/'mission-trace.jsonl'
    if (not isinstance(result, dict) or result.get('schema') != RESULT_SCHEMA
            or result.get('run_id') != run_id or result.get('ros_domain_id') != domain
            or result.get('workload_sha256') != hashlib.sha256((directory/'workload.json').read_bytes()).hexdigest()
            or result.get('producer_sha256') != hashlib.sha256(producer.read_bytes()).hexdigest()
            or not trace.is_file() or trace.stat().st_size > 256*1024**2
            or result.get('trace_sha256') != hashlib.sha256(trace.read_bytes()).hexdigest()):
        return None, 'Result/trace/source identity differs from the owned workload.'
    if invalidated:
        return None, 'A reset was requested during this workload; its result is invalidated.'
    if not isinstance(result.get('success'), bool) or result.get('goal_contract') != dict(position_m=.15, heading_rad=math.radians(5), settle_sim_s=1.):
        return None, 'Result lacks the explicit physical goal contract.'
    measurements = result.get('goals', [])
    if result['success']:
        if not result.get('nav2_terminal') or result.get('truth_samples', 0) < 3:
            return None, 'Successful result lacks terminal Nav2 state and independent truth.'
        if not isinstance(measurements, list) or len(measurements) != len(specification['goals']):
            return None, 'A successful result must contain every actual goal measurement.'
        for measured, requested in zip(measurements, specification['goals']):
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
    return result, None
